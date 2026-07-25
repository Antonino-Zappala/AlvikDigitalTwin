# =============================================================================
# selezione_waypoint_v1_7.gd
# Script principale Camera3D — Alvik Digital Twin v1.7
#
# Questo script gestisce:
#   - Le tre prospettive camera (alto, dietro, prima persona)
#   - Tutte le modalità operative del Digital Twin
#   - La comunicazione WebSocket con bridge_v1_7.py
#   - L'invio della posa corrente di Alvik al bridge
#
# Architettura comunicazione:
#   Godot → WebSocket :8765 → bridge_v1_7.py → Alvik / ROS2
#   Godot ← UDP DDS :4444  ← bridge_v1_7.py ← Alvik (posa, ToF, tick)
#
# Il DDS (autoload) riceve la telemetria da Alvik via UDP e la mette a
# disposizione tramite DDS.read("X"), DDS.read("Y"), DDS.read("Theta").
# Questo script legge quei valori e li invia al bridge via WebSocket
# affinché il path following possa calcolare i comandi di velocità.
# =============================================================================

extends Camera3D

# ── Parametri esportabili dall'editor Godot ───────────────────────────────────

# URL del server WebSocket del bridge — modificabile dall'editor
@export var ws_url    : String = "ws://127.0.0.1:8765"

# Numero massimo di righe nel log a schermo
@export var max_lines : int    = 10

# ── Costanti globali ──────────────────────────────────────────────────────────

# Lunghezza del raggio per il raycast (in unità Godot)
# Deve essere abbastanza lungo da attraversare tutta la scena
const RAY_LENGTH = 1000

# Fattore di conversione centimetri → metri (unità Godot)
# Alvik lavora in cm, Godot in metri
const CM_TO_M = 0.01

# =============================================================================
# ENUM: Prospettive camera
# =============================================================================
# TOP         — vista dall'alto, segue il robot (utile per waypoint)
# THIRD_PERSON — vista da dietro il robot a distanza
# FIRST_PERSON — vista dal punto di vista del robot
# RIGHT — vista laterale destra
# LEFT — vista laterale sinistra
enum CameraView { TOP, THIRD_PERSON, FIRST_PERSON, RIGHT, LEFT }
var current_view : CameraView = CameraView.TOP

# =============================================================================
# ENUM: Modalità operative
# =============================================================================
# Il sistema usa una macchina a stati con una sola modalità attiva alla volta.
# Questo previene conflitti tra modalità (es. teleop + waypoint contemporanei).
#
# NONE         — stato neutro, nessuna modalità attiva
# WAYPOINT     — selezione manuale punti di destinazione con click
# TELEOP       — controllo diretto W/X avanza/indietro, Q/E ruota
# ALIGN        — allineamento Digital Twin con robot fisico (frecce)
# PATH         — percorsi predefiniti (quadrato, cerchio, triangolo)
# EXPLORER     — SLAM + PID: costruisce mappa esplorando con waypoint
# EXPLORER_EXIT — stato intermedio dialogo salvataggio mappa Esploratore
# SLAM         — mappatura standalone: SLAM senza PID, solo teleop
# SLAM_EXIT    — stato intermedio dialogo salvataggio mappa SLAM
# NAV2         — navigazione autonoma su mappa pre-esistente con 2D Goal
enum Mode {
	NONE,
	WAYPOINT,
	TELEOP,
	ALIGN,
	PATH,
	EXPLORER,
	EXPLORER_EXIT,
	SLAM,
	SLAM_EXIT,
	NAV2,
}
var current_mode : Mode = Mode.NONE

# =============================================================================
# ALLINEAMENTO
# =============================================================================
# Il Digital Twin (posizione virtuale in Godot) può non corrispondere
# perfettamente alla posizione fisica di Alvik nello spazio reale.
# La modalità ALIGN permette di regolare:
#   - align_offset: traslazione X/Z del Digital Twin
#   - align_rot_y:  rotazione Y (yaw) in gradi
#
# Quando si esce dalla modalità ALIGN, l'offset viene inviato al bridge
# tramite il messaggio {"type": "alignment", "rot_y": rad} in modo che
# alvik_ros_bridge.py possa applicarlo al topic /odom di ROS2.

var align_offset  : Vector3 = Vector3.ZERO  # offset traslazione (m)
var align_rot_y   : float   = 0.0           # offset rotazione (gradi)

# Velocità di spostamento in modalità allineamento
# Questi valori determinano quanto velocemente si muove il robot virtuale
# tenendo premuto un tasto freccia
const ALIGN_MOVE_SPEED = 0.02   # m per frame
const ALIGN_ROT_SPEED  = 1.5    # gradi per frame

# Proprietà booleana calcolata — per compatibilità con digital_twin_v1_5.gd
# che chiama set_alignment() solo quando align_mode è true
var align_mode : bool:
	get: return current_mode == Mode.ALIGN

# =============================================================================
# WAYPOINT
# =============================================================================
# Lista dei punti 3D selezionati con click sul pavimento.
# Quando si preme SPAZIO, la lista viene convertita in coordinate Alvik
# (con correzione dell'offset di allineamento) e inviata al bridge come
# messaggio {"type": "waypoints", "points": [{"x": cm, "y": cm}, ...]}.
# Il bridge esegue il path following proporzionale verso ogni punto.

var waypoint_list : Array[Vector3] = []

# Proprietà booleana calcolata — usata in _unhandled_input per il raycast
var is_selecting  : bool:
	get: return current_mode == Mode.WAYPOINT

# var is_selecting : bool: Dichiara una variabile booleana (true o false).
# Il due punti : esegue la tipizzazione statica
# get: : Indica che si sta definendo un comportamento personalizzato 
# per quando viene letto il valore di is_selecting.
# return current_mode == Mode.WAYPOINT: Ogni volta che qualcosa nel codice legge 
# is_selecting, Godot valuta questa condizione. Restituisce true se la modalità 
# attuale corrisponde a XYZ, altrimenti restituisce false.	
# Serve a creare una variabile "virtuale" che non memorizza un dato fisso in memoria, 
# ma riflette in tempo reale lo stato di un'altra variabile (current_mode).
# Invece di scrivere ogni volta un controllo manuale come if current_mode == Mode.WAYPOINT:,
# basta leggere if is_selecting:. Inoltre, non c'è il rischio che is_selecting rimanga 
# disallineata rispetto a current_mode.


# =============================================================================
# WEBSOCKET
# =============================================================================
# Godot usa WebSocketPeer per comunicare con bridge_v1_7.py.
# Il WebSocket è bidirezionale:
#   → Godot invia: waypoints, cmd, stop, reset, pose, alignment,
#                  explorer_start/stop, slam_start/stop, nav2_start/stop/goal
#   ← Bridge invia: log, waypoint_reached, explorer_started/stopped,
#                   slam_started/stopped, nav2_started/stopped

var _socket          : WebSocketPeer = WebSocketPeer.new()
var _reconnect_timer : float         = 0.0
const RECONNECT_DELAY = 3.0   # secondi prima di tentare riconnessione

# =============================================================================
# INVIO POSA
# =============================================================================
# La posa corrente di Alvik (letta dal DDS) viene inviata al bridge ogni
# POSE_SEND_EVERY frame. Il bridge la usa per il path following proporzionale.
# Non serve inviarla ogni frame (50Hz) — ogni 3 frame (≈16Hz) è sufficiente.

var _pose_frame_counter : int = 0
const POSE_SEND_EVERY   = 3   # ogni N frame

# =============================================================================
# COORDINATE LABEL
# =============================================================================
# Etichetta che mostra le coordinate (Godot e Alvik) vicino al cursore
# durante la selezione waypoint. Aiuta a capire la posizione reale di Alvik.
# Aggiornata in tempo reale tramite raycast durante il movimento del mouse.

@onready var coord_label : Label = \
	get_node("/root/AlvikRobotEnv/CanvasLayer2/CoordLabel")

# =============================================================================
# ZOOM CAMERA
# =============================================================================
# La rotellina del mouse controlla la distanza della camera dal robot.
# Funziona in tutte e tre le prospettive.

const ZOOM_SPEED = 0.1   # incremento per scatto rotellina
const ZOOM_MIN   = 0.5   # distanza minima (zoom massimo)
const ZOOM_MAX   = 5.0   # distanza massima (zoom minimo)
var _zoom_distance : float = 2.0

# =============================================================================
# TELEOP
# =============================================================================
# Controllo manuale diretto di Alvik tramite tastiera.
# I comandi vengono inviati al bridge a frequenza fissa (20Hz = ogni 0.05s).
#
# Nota tecnica: i tasti vengono salvati in variabili booleane invece di
# essere letti direttamente in _process(), perché _input() processa i tasti
# molto più velocemente e con state.pressed/released distinto.
# Questo evita il problema dei tasti "persi" durante l'elaborazione.
#
# Mapping tasti:
#   W → avanza (vlin = +TELEOP_VEL_LIN)
#   X → indietro (vlin = -TELEOP_VEL_LIN)
#   Q → ruota sinistra (vang = +TELEOP_VEL_ANG)
#   E → ruota destra (vang = -TELEOP_VEL_ANG)

var teleop_mode : bool:
	get: return current_mode == Mode.TELEOP

const TELEOP_VEL_LIN    = 10.0    # cm/s velocità lineare teleop
const TELEOP_VEL_ANG    = 45.0    # deg/s velocità angolare teleop
var _teleop_timer       : float = 0.0
const TELEOP_SEND_EVERY = 0.05    # secondi tra un comando teleop e il successivo

# Stato corrente dei tasti teleop (true = premuto, false = rilasciato)
var _key_w : bool = false
var _key_x : bool = false
var _key_q : bool = false
var _key_e : bool = false

# =============================================================================
# PATH PREDEFINITI
# =============================================================================
# Genera automaticamente una lista di waypoint per percorsi geometrici:
#   Quadrato:   4 punti agli angoli
#   Cerchio:    12 punti equidistanti sulla circonferenza
#   Triangolo:  3 vertici del triangolo equilatero
#
# L'utente seleziona prima il tipo (4/5/6) poi la dimensione (1/2/3/4).
# _path_menu_step tiene traccia di quale sotto-menu è attivo:
#   0 = inattivo
#   1 = selezione tipo
#   2 = selezione dimensione

var _path_type      : String = ""
var _path_menu_step : int    = 0
const PATH_SIZES = [25.0, 50.0, 75.0, 100.0]   # dimensioni disponibili in cm

# =============================================================================
# MODALITÀ ESPLORATORE (v1.6)
# =============================================================================
# Avvia SLAM (slam_toolbox) + PID controller nel container ROS2.
# Alvik costruisce la mappa esplorando l'ambiente tramite waypoint manuali.
# All'uscita viene proposto il salvataggio della mappa.
#
# Flusso:
#   E → bridge invia explorer_start → avvia slam_toolbox + alvik_pid_controller
#   ESC → apre dialogo salvataggio (S=salva, N=no, C=annulla)
#   S/N → bridge invia explorer_stop con save_map=true/false

var explorer_mode : bool:
	get: return current_mode == Mode.EXPLORER

# =============================================================================
# MODALITÀ SLAM STANDALONE (v1.7 — NUOVO)
# =============================================================================
# Mappatura pura senza PID. Alvik viene controllato con teleop (W/X/Q/E)
# mentre slam_toolbox costruisce la mappa in background.
# Utile per creare mappe accurate prima di usare Nav2.
#
# Differenza con Esploratore:
#   SLAM standalone → teleop manuale, nessun PID, solo mappatura
#   Esploratore     → waypoint automatici, PID attivo, SLAM in background
#
# Flusso:
#   L → bridge invia slam_start → avvia slam_toolbox + robot_state_publisher
#   ESC → apre dialogo salvataggio (S=salva, N=no, C=annulla)
#   S/N → bridge invia slam_stop con save_map=true/false

var slam_mode : bool:
	get: return current_mode == Mode.SLAM

# =============================================================================
# MODALITÀ NAV2 (v1.7 — NUOVO)
# =============================================================================
# Navigazione autonoma su mappa pre-esistente.
# Richiede che una mappa sia già stata creata con SLAM o Esploratore.
#
# Flusso:
#   N → bridge invia nav2_start → avvia localization_launch + navigation_launch
#       + pubblica automaticamente la posa iniziale (0,0,π)
#   Click su mappa → raycast → bridge invia nav2_goal con coordinate in metri
#   ESC → bridge invia nav2_stop → ferma tutti i nodi Nav2
#
# Nota: le coordinate del goal vengono convertite da Godot (metri) a ROS2 (metri)
# applicando l'inversione del sistema di riferimento:
#   ROS2_x = -(Godot_z) / 100  (asse Z Godot → X negativo ROS2)
#   ROS2_y = -(Godot_x) / 100  (asse X Godot → Y negativo ROS2)

var nav2_mode : bool:
	get: return current_mode == Mode.NAV2

# =============================================================================
# FLAG TELEOP DENTRO ESPLORATORE
# =============================================================================
# Quando l'utente attiva Teleop mentre è in Esploratore, questo flag è true.
# ESC in quella situazione esce da Teleop + Esploratore insieme,
# mostrando il dialogo salvataggio mappa.
# Se Teleop è attivo fuori dall'Esploratore, ESC esce solo dal Teleop.

var _explorer_teleop : bool = false

var menu_visible : bool = true

# Nodi UI — vengono risolti all'avvio tramite path assoluto nella scena
@onready var menu_label  : RichTextLabel = \
	get_node("/root/AlvikRobotEnv/CanvasLayer3/MenuLabel")
@onready var info_label  : RichTextLabel = \
	get_node("/root/AlvikRobotEnv/CanvasLayer2/InfoLabel")

# robot_node: riferimento al nodo 3D del Digital Twin (script digital_twin_v1_5.gd)
# Usato per leggere la posizione/rotazione e per chiamare metodi come
# set_alignment(), reset_pose_godot(), clear_obstacle_map()
@onready var robot_node  = get_node("/root/AlvikRobotEnv/Alvik")

# marker_scene: pallino 3D che appare sul pavimento quando si seleziona un waypoint
@onready var marker_scene = preload("res://select_pointer.tscn")

# Buffer log — array di stringhe con le ultime max_lines righe di log
var _log_history : Array = []

# Label coordinate mouse — toggle con tasto H
# Quando attivo, mostra le coordinate del punto sotto il cursore in tutte le modalità
var _mouse_coords_visible : bool = false

# =============================================================================
# GRIGLIA E FRECCIA POSA INIZIALE
# =============================================================================
# G toggle: mostra/nasconde griglia 1m×1m (marcata) + 25cm×25cm (leggera)
# e una freccia rossa che indica la posa iniziale di Alvik al momento del reset.
#
# La griglia viene disegnata con ImmediateMesh — linee sul piano Y=0.
# La freccia è un MeshInstance3D con CylinderMesh orientato.

var _grid_visible      : bool = false
var _grid_instance     : MeshInstance3D = null   # linee griglia
var _origin_arrow      : Node3D = null            # freccia posa iniziale
var _origin_pose       : Vector3 = Vector3.ZERO  # posizione iniziale salvata
var _origin_rot        : float   = 0.0            # rotazione iniziale salvata

# Colori griglia
const GRID_COLOR_MAJOR = Color(0.5, 0.8, 0.5, 0.6)   # verde chiaro — linee 1m
const GRID_COLOR_MINOR = Color(0.3, 0.5, 0.3, 0.25)  # verde scuro semitrasparente — 25cm
const GRID_SIZE        = 20.0    # estensione griglia in metri (±20m)
const GRID_MAJOR       = 1.0     # passo linee marcate (m)
const GRID_MINOR       = 0.25    # passo linee leggere (m)

# =============================================================================
# LIFECYCLE
# =============================================================================

## Inizializzazione: connessione WebSocket, UI, DDS, nodi scena.
func _ready() -> void:
	# Mostra il menu principale all'avvio
	_show_menu()

	# Debug: verifica che il nodo robot sia raggiungibile

	# Connette il WebSocket al bridge
	_ws_connect()

	# Imposta la vista iniziale dall'alto
	_set_camera_view(CameraView.TOP)

	# Aggiorna l'etichetta info con lo stato iniziale
	_update_info_label()
	

## Loop principale — WebSocket polling, invio posa, teleop, camera.
func _process(delta: float) -> void:
	# Poll necessario ogni frame per ricevere/inviare dati WebSocket
	_socket.poll()

	# Gestisce lo stato della connessione WebSocket (riconnessione automatica)
	_handle_ws(delta)

	# Aggiorna la posizione della camera in base alla modalità e alla vista
	_update_camera(delta)

	# Invia la posa di Alvik al bridge ogni POSE_SEND_EVERY frame
	# (il bridge la usa per il path following proporzionale)
	_pose_frame_counter += 1
	if _pose_frame_counter >= POSE_SEND_EVERY:
		_pose_frame_counter = 0
		_send_pose()

	# Processa i pacchetti WebSocket in arrivo dal bridge
	_process_ws_packets()

	# In modalità teleop: invia comandi a frequenza fissa (20Hz)
	if current_mode == Mode.TELEOP:
		_teleop_timer += delta
		if _teleop_timer >= TELEOP_SEND_EVERY:
			_teleop_timer = 0.0
			_send_teleop()

# =============================================================================
# CAMERA
# =============================================================================

## Aggiorna posizione e rotazione della camera in base alla vista corrente.
## Gestisce 5 viste: TOP, THIRD_PERSON, FIRST_PERSON, RIGHT, LEFT.
func _update_camera(_delta: float) -> void:
	if robot_node == null:
		return

	# In modalità ALIGN: le frecce spostano/ruotano il Digital Twin
	# Il robot si muove mentre la camera rimane ferma per facilitare l'allineamento
	if current_mode == Mode.ALIGN:
		var rot_rad = robot_node.rotation.y
		# Calcola il vettore "avanti" del robot per muoverlo nella direzione giusta
		var forward = Vector3(sin(rot_rad), 0, cos(rot_rad))
		if Input.is_action_pressed("ui_up"):    align_offset += forward * ALIGN_MOVE_SPEED
		if Input.is_action_pressed("ui_down"):  align_offset -= forward * ALIGN_MOVE_SPEED
		if Input.is_action_pressed("ui_left"):  align_rot_y  -= ALIGN_ROT_SPEED
		if Input.is_action_pressed("ui_right"): align_rot_y  += ALIGN_ROT_SPEED
		# Applica l'offset al Digital Twin ogni frame
		robot_node.set_alignment(align_offset, align_rot_y)

	var robot_pos = robot_node.global_position
	var robot_rot = robot_node.rotation.y

	match current_view:
		CameraView.TOP:
			# Vista dall'alto: camera direttamente sopra il robot, sempre orientata a nord
			global_position  = robot_pos + Vector3(0, _zoom_distance, 0)
			rotation_degrees = Vector3(-90, 0, 0)

		CameraView.THIRD_PERSON:
			# Vista da dietro: camera leggermente rialzata e dietro al robot
			# Il fattore 0.25 riduce la distanza orizzontale rispetto allo zoom
			var behind = Vector3(sin(robot_rot), 0, cos(robot_rot)) * _zoom_distance * 0.25
			global_position = robot_pos + Vector3(0, _zoom_distance * 0.2, 0) + behind
			look_at(robot_pos + Vector3(0, 0.05, 0), Vector3.UP)

		CameraView.FIRST_PERSON:
			# Vista in prima persona: camera posizionata sulla "fronte" del robot
			# Il vettore fwd è invertito perché il robot punta nella direzione -Z
			var fwd = Vector3(-sin(robot_rot), 0, -cos(robot_rot)) * 0.05
			global_position = robot_pos + Vector3(0, 0.08, 0) + fwd
			rotation.y = robot_rot + PI

		CameraView.RIGHT:
			# Vista laterale destra: camera sul lato destro del robot
			# "destra" = perpendicolare alla direzione di marcia, lato +X locale
			var right = Vector3(cos(robot_rot), 0, -sin(robot_rot)) * _zoom_distance * 0.25
			global_position = robot_pos + Vector3(0, _zoom_distance * 0.15, 0) + right
			look_at(robot_pos + Vector3(0, 0.05, 0), Vector3.UP)

		CameraView.LEFT:
			# Vista laterale sinistra: camera sul lato sinistro del robot
			var left = Vector3(-cos(robot_rot), 0, sin(robot_rot)) * _zoom_distance * 0.25
			global_position = robot_pos + Vector3(0, _zoom_distance * 0.15, 0) + left
			look_at(robot_pos + Vector3(0, 0.05, 0), Vector3.UP)

## Cambia la vista camera corrente e aggiorna current_view.
func _set_camera_view(view: CameraView) -> void:
	current_view = view
	_update_info_label()

# =============================================================================
# INPUT PRINCIPALE
# =============================================================================
# Gestisce tutti gli eventi tastiera.
# L'ordine dei controlli è importante per evitare conflitti tra modalità:
#
#  1. Rilascio tasti teleop (sempre, anche se non in teleop)
#  2. M e ENTER (sempre attivi tranne in dialoghi)
#  3. Dialoghi uscita (EXPLORER_EXIT, SLAM_EXIT)
#  4. ESC (esce dalla modalità corrente)
#  5. Tasti teleop pressed (solo se in TELEOP, poi return)
#  6. Selezione dimensione path (solo se in PATH step 2, poi return)
#  7. Tasti viste camera (1/2/3)
#  8. Toggle modalità (A, T, S, E, L, N, P)
#  9. Selezione tipo path (solo se in PATH step 1)
# 10. Tasti allineamento (R in ALIGN)
# 11. Tasti generali (SHIFT, R, C, SPACE)

## Gestisce tutti gli input da tastiera e rotellina mouse.
## Implementa la macchina a stati delle modalità operative.
## Gestisce: tasti vista camera (1-5), zoom (rotellina), modalità (S/T/P/A/E/L/N),
## teleop (W/X/Q/E), sistema (R/C/G/H/M/Shift/Esc).
func _input(event: InputEvent) -> void:

	if not event is InputEventKey:
		return
	# ── 1. Rilascio tasti teleop ──────────────────────────────────────────────
	# Gestito per primo perché si applica al rilascio (not event.pressed).
	# Aggiorna lo stato booleano dei tasti teleop quando vengono rilasciati.
	if not event.pressed:
		if current_mode == Mode.TELEOP:
			match event.keycode:
				KEY_W: _key_w = false
				KEY_X: _key_x = false
				KEY_Q: _key_q = false
				KEY_E: _key_e = false
		return


	# ── 2. M — apri menu ─────────────────────────────────────────────────────
	# Sempre attivo tranne nei dialoghi di uscita (dove servono S/N/C)
	if event.keycode == KEY_M:
		if current_mode != Mode.EXPLORER_EXIT and current_mode != Mode.SLAM_EXIT:
			menu_visible = true
			_show_menu()
		return

	# ── ENTER — chiudi menu ───────────────────────────────────────────────────
	if event.keycode == KEY_ENTER:
		if menu_visible and current_mode != Mode.EXPLORER_EXIT \
				and current_mode != Mode.SLAM_EXIT:
			menu_visible = false
			menu_label.visible = false
		return

	# ── 3. Dialogo uscita Esploratore ─────────────────────────────────────────
	# Intercetta S/N/C solo quando si sta uscendo dall'Esploratore
	if current_mode == Mode.EXPLORER_EXIT:
		match event.keycode:
			KEY_S:
				# Ferma Esploratore e salva la mappa in /ros2_ws/config/
				_ws_send({"type": "explorer_stop", "save_map": true})
				_log("[color=yellow]Uscita Esploratore — salvataggio mappa...[/color]")
				menu_label.visible = false
				current_mode = Mode.NONE
				_update_info_label()
			KEY_N:
				# Ferma Esploratore senza salvare
				_ws_send({"type": "explorer_stop", "save_map": false})
				_log("[color=yellow]Uscita Esploratore — mappa non salvata[/color]")
				menu_label.visible = false
				current_mode = Mode.NONE
				_update_info_label()
			KEY_C:
				# Annulla uscita — rimane in modalità Esploratore
				current_mode = Mode.EXPLORER
				menu_label.visible = false
				_log("[color=gray]Uscita annullata — Esploratore attivo[/color]")
		return

	# ── 3b. Dialogo uscita SLAM ───────────────────────────────────────────────
	# Identico all'Esploratore ma per la modalità SLAM standalone
	if current_mode == Mode.SLAM_EXIT:
		match event.keycode:
			KEY_S:
				_ws_send({"type": "slam_stop", "save_map": true})
				_log("[color=yellow]Uscita SLAM — salvataggio mappa...[/color]")
				menu_label.visible = false
				current_mode = Mode.NONE
				_update_info_label()
			KEY_N:
				_ws_send({"type": "slam_stop", "save_map": false})
				_log("[color=yellow]Uscita SLAM — mappa non salvata[/color]")
				menu_label.visible = false
				current_mode = Mode.NONE
				_update_info_label()
			KEY_C:
				current_mode = Mode.SLAM
				menu_label.visible = false
				_log("[color=gray]Uscita annullata — SLAM attivo[/color]")
		return

	# ── 4. ESC — esce dalla modalità corrente ─────────────────────────────────
	# Comportamento diverso per ogni modalità:
	#   TELEOP (normale)          → torna a NONE
	#   TELEOP (dentro Esploratore) → esce da Teleop + Esploratore, dialogo mappa
	#   ALIGN/WAYPOINT/PATH       → torna a NONE
	#   EXPLORER → apre dialogo salvataggio mappa
	#   SLAM     → apre dialogo salvataggio mappa
	#   NAV2     → ferma Nav2 e torna a NONE
	#   NONE     → ferma robot (sicurezza)
	if event.keycode == KEY_ESCAPE:
		match current_mode:
			Mode.TELEOP:
				_key_w = false; _key_x = false; _key_q = false; _key_e = false
				_ws_send({"type": "stop"})
				_log("[color=magenta]Teleop disattivo[/color]")
				if _explorer_teleop:
					# Teleop era dentro Esploratore → apre subito dialogo mappa
					_explorer_teleop = false
					current_mode = Mode.EXPLORER_EXIT
					_show_explorer_exit_dialog()
					return
				else:
					current_mode = Mode.NONE
			Mode.ALIGN:
				# Conferma l'allineamento e invia l'offset a ROS2
				_log("[color=green]Allineamento confermato[/color]")
				_ws_send({"type": "alignment", "rot_y": deg_to_rad(align_rot_y)})
				current_mode = Mode.NONE
				# Aggiorna la freccia posa iniziale con la nuova rotazione
				if _grid_visible:
					_draw_origin_arrow()
			Mode.WAYPOINT:
				# Cancella tutti i waypoint selezionati e i marker 3D
				waypoint_list.clear()
				_clear_markers()
				coord_label.visible = false
				_ws_send({"type": "stop"})
				_log("[color=red]Selezione annullata[/color]")
				current_mode = Mode.NONE
			Mode.PATH:
				_path_menu_step = 0
				menu_label.visible = false
				_log("[color=gray]Path annullato[/color]")
				current_mode = Mode.NONE
			Mode.EXPLORER:
				# Non ferma subito — chiede se salvare la mappa
				current_mode = Mode.EXPLORER_EXIT
				_show_explorer_exit_dialog()
				return
			Mode.SLAM:
				# Non ferma subito — chiede se salvare la mappa
				current_mode = Mode.SLAM_EXIT
				_show_slam_exit_dialog()
				return
			Mode.NAV2:
				# Ferma Nav2 immediatamente senza chiedere conferma
				_ws_send({"type": "nav2_stop"})
				_log("[color=yellow]Nav2 fermato[/color]")
				current_mode = Mode.NONE
				_update_info_label()
				return
			_:
				# Stato NONE: ESC è sempre un "ferma robot" di sicurezza
				_ws_send({"type": "stop"})
				_clear_markers()
				_log("[color=red]Robot fermato[/color]")
		_update_info_label()
		return

	# ── 5. Tasti teleop pressed ───────────────────────────────────────────────
	# Deve stare PRIMA dei tasti generali per evitare che KEY_E
	# (ruota destra in teleop) attivi accidentalmente la modalità Esploratore.
	# Il return finale blocca qualsiasi altro handler.
	if current_mode == Mode.TELEOP:
		match event.keycode:
			KEY_W: _key_w = true
			KEY_X: _key_x = true
			KEY_Q: _key_q = true
			KEY_E: _key_e = true   # E in teleop = ruota destra, NON esploratore
		return

	# ── 6. Selezione dimensione path ──────────────────────────────────────────
	# Deve stare PRIMA dei tasti vista (1/2/3) per evitare che la selezione
	# della dimensione "1", "2", "3" cambi accidentalmente la vista camera.
	if current_mode == Mode.PATH and _path_menu_step == 2:
		match event.keycode:
			KEY_1:
				_send_path(_path_type, PATH_SIZES[0])
				current_mode = Mode.NONE
				_path_menu_step = 0
				menu_label.visible = false
			KEY_2:
				_send_path(_path_type, PATH_SIZES[1])
				current_mode = Mode.NONE
				_path_menu_step = 0
				menu_label.visible = false
			KEY_3:
				_send_path(_path_type, PATH_SIZES[2])
				current_mode = Mode.NONE
				_path_menu_step = 0
				menu_label.visible = false
			KEY_4:
				_send_path(_path_type, PATH_SIZES[3])
				current_mode = Mode.NONE
				_path_menu_step = 0
				menu_label.visible = false
		return

	# ── 7. Viste camera ───────────────────────────────────────────────────────
	if event.keycode == KEY_1:
		_set_camera_view(CameraView.TOP)
		_log("[color=gray]Vista: dall'alto[/color]")
		return
	if event.keycode == KEY_2:
		_set_camera_view(CameraView.THIRD_PERSON)
		_log("[color=gray]Vista: dietro[/color]")
		return
	if event.keycode == KEY_3:
		_set_camera_view(CameraView.FIRST_PERSON)
		_log("[color=gray]Vista: prima persona[/color]")
		return
	if event.keycode == KEY_4 and current_mode != Mode.PATH:
		_set_camera_view(CameraView.RIGHT)
		_log("[color=gray]Vista: laterale destra[/color]")
		return
	if event.keycode == KEY_5 and current_mode != Mode.PATH:
		_set_camera_view(CameraView.LEFT)
		_log("[color=gray]Vista: laterale sinistra[/color]")
		return

	# ── 8. Toggle modalità ────────────────────────────────────────────────────
	# Ogni tasto entra nella modalità se si è in NONE, o esce se già attiva.

	# A — Allineamento Digital Twin
	if event.keycode == KEY_A:
		if current_mode == Mode.ALIGN:
			current_mode = Mode.NONE
			_ws_send({"type": "alignment", "rot_y": deg_to_rad(align_rot_y)})
			_log("[color=green]Allineamento confermato[/color]")
			# Aggiorna la freccia posa iniziale con la nuova rotazione
			if _grid_visible:
				_draw_origin_arrow()
		else:
			current_mode = Mode.ALIGN
			_log("[color=yellow]Modalita allineamento attiva[/color]")
		_update_info_label()
		return

	# T — Teleop manuale W/X/Q/E
	# Se attivato mentre Esploratore è attivo, imposta _explorer_teleop=true
	# così ESC saprà di dover uscire da entrambi contemporaneamente
	if event.keycode == KEY_T:
		if current_mode == Mode.TELEOP:
			_key_w = false; _key_x = false; _key_q = false; _key_e = false
			_ws_send({"type": "stop"})
			_log("[color=magenta]Teleop disattivo[/color]")
			# Se eravamo in Teleop dentro Esploratore → torna a Esploratore
			if _explorer_teleop:
				current_mode = Mode.EXPLORER
				_explorer_teleop = false
				_log("[color=cyan]Esploratore attivo[/color]")
			else:
				current_mode = Mode.NONE
		else:
			# Attivazione Teleop — ricorda se siamo dentro Esploratore
			_explorer_teleop = (current_mode == Mode.EXPLORER)
			current_mode = Mode.TELEOP
			if _explorer_teleop:
				_log("[color=magenta]Teleop attivo in Esploratore — ESC esce da entrambi[/color]")
			else:
				_log("[color=magenta]Teleop attivo — W/X avanza/indietro  Q/E ruota[/color]")
		_update_info_label()
		return

	# S — Selezione Waypoint (non disponibile in teleop perché KEY_X è già usato)
	if event.keycode == KEY_S and current_mode != Mode.TELEOP:
		if current_mode == Mode.WAYPOINT:
			current_mode = Mode.NONE
			waypoint_list.clear()
			_clear_markers()
			coord_label.visible = false
			_log("[color=red]Selezione annullata[/color]")
		else:
			current_mode = Mode.WAYPOINT
			_log("[color=cyan]Selezione attiva[/color] — clicca sul pavimento")
		_update_info_label()
		return

	# E — Esploratore (SLAM + PID, non disponibile in teleop)
	if event.keycode == KEY_E and current_mode != Mode.TELEOP:
		if current_mode == Mode.EXPLORER:
			current_mode = Mode.EXPLORER_EXIT
			_show_explorer_exit_dialog()
		else:
			current_mode = Mode.EXPLORER
			_ws_send({"type": "explorer_start"})
			_log("[color=cyan]Modalita Esploratore avviata...[/color]")
		_update_info_label()
		return

	# L — SLAM standalone (mappatura senza PID, controllo teleop)
	# Usa L per evitare conflitti con altri tasti
	if event.keycode == KEY_L:
		if current_mode == Mode.SLAM:
			current_mode = Mode.SLAM_EXIT
			_show_slam_exit_dialog()
		else:
			current_mode = Mode.SLAM
			_ws_send({"type": "slam_start"})
			_log("[color=cyan]Modalita SLAM avviata — usa teleop per mappare[/color]")
		_update_info_label()
		return

	# N — Nav2 navigazione autonoma (richiede mappa pre-esistente)
	# Non disponibile in Teleop
	if event.keycode == KEY_N and current_mode != Mode.TELEOP:
		if current_mode == Mode.NAV2:
			_ws_send({"type": "nav2_stop"})
			_log("[color=yellow]Nav2 fermato[/color]")
			current_mode = Mode.NONE
			_update_info_label()
		else:
			current_mode = Mode.NAV2
			_ws_send({"type": "nav2_start"})
			_log("[color=cyan]Nav2 avviato — attendi inizializzazione...[/color]")
			_log("[color=gray]Poi clicca sulla mappa per il goal[/color]")
			_update_info_label()
		return

	# P — Path predefiniti (solo da NONE per non interferire con altri menu)
	if event.keycode == KEY_P and current_mode == Mode.NONE:
		current_mode = Mode.PATH
		_path_menu_step = 1
		_show_path_type_menu()
		return

	# ── 9. Selezione tipo path ────────────────────────────────────────────────
	if current_mode == Mode.PATH and _path_menu_step == 1:
		match event.keycode:
			KEY_4:
				_path_type = "quadrato"
				_path_menu_step = 2
				_show_path_size_menu()
			KEY_5:
				_path_type = "cerchio"
				_path_menu_step = 2
				_show_path_size_menu()
			KEY_6:
				_path_type = "triangolo"
				_path_menu_step = 2
				_show_path_size_menu()
		return

	# ── 10. Tasti allineamento ────────────────────────────────────────────────
	# Solo R è gestito qui (azzera offset) — le frecce sono in _update_camera()
	if current_mode == Mode.ALIGN:
		if event.keycode == KEY_R:
			align_offset = Vector3.ZERO
			align_rot_y  = 0.0
			_log("[color=yellow]Offset allineamento azzerato[/color]")
		return

	# ── 11. Tasti generali ────────────────────────────────────────────────────

	# H — toggle label coordinate mouse (funziona in tutte le modalità)
	if event.keycode == KEY_H:
		_mouse_coords_visible = !_mouse_coords_visible
		if not _mouse_coords_visible:
			coord_label.visible = false
		_log("[color=gray]Coordinate mouse: %s[/color]" % ("ON" if _mouse_coords_visible else "OFF"))
		return

	# SHIFT — nasconde/mostra l'etichetta info a destra
	if event.keycode == KEY_SHIFT:
		info_label.visible = !info_label.visible
		return

	# R — Reset posizione: invia RESET ad Alvik + azzera Digital Twin in Godot
	# Il bridge_v1_7 riceve "reset", invia b"RESET" ad Alvik via UDP,
	# e pubblica su /alvik/reset per azzerare l'odometria in ROS2.
	if event.keycode == KEY_R:
		_ws_send({"type": "reset"})
		align_offset = Vector3.ZERO
		align_rot_y  = 0.0
		if robot_node and robot_node.has_method("reset_pose_godot"):
			robot_node.reset_pose_godot()
		# Aggiorna la freccia posa iniziale se la griglia è visibile
		if _grid_visible:
			_draw_origin_arrow()
		_log("[color=yellow]Reset posizione inviato[/color]")
		return

	# G — Toggle griglia 1m×1m / 25cm×25cm + freccia posa iniziale
	if event.keycode == KEY_G:
		_grid_visible = !_grid_visible
		if _grid_visible:
			_draw_grid()
			_draw_origin_arrow()
			_log("[color=gray]Griglia attiva[/color]")
		else:
			_clear_grid()
			_log("[color=gray]Griglia nascosta[/color]")
		return

	# C — Cancella la mappa ostacoli dinamica nel Digital Twin
	# La mappa ostacoli viene costruita in tempo reale dai sensori ToF
	if event.keycode == KEY_C:
		if robot_node and robot_node.has_method("clear_obstacle_map"):
			robot_node.clear_obstacle_map()
		_log("[color=yellow]Mappa ostacoli pulita[/color]")
		return

	# SPACE — Invia i waypoint selezionati al bridge per il path following
	if event.keycode == KEY_SPACE:
		if waypoint_list.size() > 0:
			_send_waypoints()
		else:
			_log("[color=yellow]Nessun punto — premi S[/color]")
		return

# =============================================================================
# UNHANDLED INPUT
# =============================================================================
# Gestisce gli eventi mouse non catturati da altri nodi.
# Separato da _input() per gestire correttamente la priorità degli eventi.

## Gestisce il click mouse sul pavimento per la selezione waypoint e Nav2 goal.
## Usa raycast per trovare il punto 3D sul pavimento (layer 1) o su un ostacolo (layer 2).
## In modalità WAYPOINT: aggiunge il punto alla lista e spawna un marker.
## In modalità NAV2: converte le coordinate Godot→ROS2 e invia il goal al bridge.
func _unhandled_input(event: InputEvent) -> void:

	# Aggiorna le coordinate nel label vicino al cursore durante la selezione
	# oppure quando _mouse_coords_visible è attivo (tasto H)
	if event is InputEventMouseMotion:
		if current_mode == Mode.WAYPOINT or _mouse_coords_visible:
			_update_coord_label(event.position)
		elif not _mouse_coords_visible:
			coord_label.visible = false
		return

	# Zoom con rotellina — funziona in tutte le modalità
	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			_zoom_distance = clampf(_zoom_distance - ZOOM_SPEED, ZOOM_MIN, ZOOM_MAX)
			get_viewport().set_input_as_handled()
			return
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			_zoom_distance = clampf(_zoom_distance + ZOOM_SPEED, ZOOM_MIN, ZOOM_MAX)
			get_viewport().set_input_as_handled()
			return

	# Click sinistro in modalità WAYPOINT — aggiunge un punto alla lista
	if (current_mode == Mode.WAYPOINT
			and event is InputEventMouseButton
			and event.pressed
			and event.button_index == MOUSE_BUTTON_LEFT):

		var result = _raycast_full(event.position)
		if result.is_empty():
			return

		# Controlla se il punto è su un ostacolo della mappa dinamica
		# ObstacleMap contiene i StaticBody3D generati dai sensori ToF
		var collider = result.get("collider")
		if collider != null and collider is StaticBody3D \
				and collider.get_parent() != null \
				and collider.get_parent().name == "ObstacleMap":
			_log("[color=red]Posizione non raggiungibile — ostacolo![/color]")
			return

		var point = result.get("position")
		if point != null:
			waypoint_list.append(point)
			_spawn_marker(point, waypoint_list.size())
			_log("Punto %d: (%.1f, %.1f) cm" % [
				waypoint_list.size(), point.z / CM_TO_M, point.x / CM_TO_M])

	# Click sinistro in modalità NAV2 — invia goal di navigazione autonoma
	if (current_mode == Mode.NAV2
			and event is InputEventMouseButton
			and event.pressed
			and event.button_index == MOUSE_BUTTON_LEFT):

		var result = _raycast_full(event.position)
		if result.is_empty():
			return

		var point = result.get("position")
		if point != null:
			# Converti da coordinate Godot a coordinate ROS2
			# Sistema Godot: X=destra, Y=su, Z=avanti (ma in Godot Z è profondità)
			# Sistema ROS2:  X=avanti, Y=sinistra, Z=su
			# Con l'offset theta+π applicato in alvik_ros_bridge, serve la negazione
			# Usa le coordinate Alvik già calcolate (con allineamento)
			# e convertile in metri per ROS2
			# Conversione Godot → ROS2 senza allineamento
			var ros_x = -point.x
			var ros_y =  point.z
			var theta_raw = DDS.read("Theta")
			var theta_nav = (float(theta_raw) + PI) if theta_raw != null else PI
			while theta_nav >  PI: theta_nav -= 2 * PI
			while theta_nav < -PI: theta_nav += 2 * PI
			_ws_send({
				"type":  "nav2_goal",
				"x":     ros_x,
				"y":     ros_y,
				"theta": theta_nav
			})
			_log("[color=cyan]Nav2 goal: (%.2f, %.2f) m[/color]" % [ros_x, ros_y])

# =============================================================================
# RAYCAST
# =============================================================================

## Proietta un raggio dal punto del mouse nella scena 3D e restituisce
## un Dictionary con la posizione colpita. Collision mask 0b11 = layer 1 (pavimento)
## + layer 2 (ostacoli). Restituisce {} se nessuna collisione.
func _raycast_full(mouse_pos: Vector2) -> Dictionary:
	# Proietta un raggio dal punto del mouse nella scena 3D
	# collision_mask 0b11 = layer 1 + layer 2 (pavimento + ostacoli)
	var space  = get_world_3d().direct_space_state
	var origin = project_ray_origin(mouse_pos)
	var target = origin + project_ray_normal(mouse_pos) * RAY_LENGTH
	var query  = PhysicsRayQueryParameters3D.create(origin, target)
	query.collide_with_areas = true
	query.collision_mask     = 0b11
	return space.intersect_ray(query)

# =============================================================================
# MARKER WAYPOINT
# =============================================================================

## Istanzia un marker 3D (pallino rosso) con Label3D gialla sopra.
## Il Label3D mostra il numero del punto e le coordinate in cm.
## Il marker viene aggiunto al gruppo "markers" per poter essere rimosso con _clear_markers().
func _spawn_marker(point: Vector3, index: int) -> void:
	# Istanzia un pallino 3D con etichetta coordinate sovrapposta.
	# Il gruppo "markers" permette di trovare e rimuovere tutti i marker.
	var m = marker_scene.instantiate()
	get_tree().current_scene.add_child(m)
	m.add_to_group("markers")
	m.global_position = point

	# Etichetta 2D con coordinate e numero del punto
	# Usa Label3D per mostrare le coordinate sopra il marker
	var label = Label3D.new()
	label.text = "%d: (%.0f, %.0f) cm" % [
		index,
		point.z / CM_TO_M,   # asse avanti Alvik
		point.x / CM_TO_M    # asse laterale Alvik
	]
	label.font_size       = 8
	label.modulate        = Color(1.0, 1.0, 0.2, 1.0)  # giallo
	label.billboard       = BaseMaterial3D.BILLBOARD_ENABLED
	label.no_depth_test   = true
	label.position        = Vector3(-0.08, 0.05, -0.08)   # traslata lateralmente
	m.add_child(label)

## Rimuove tutti i marker e le etichette dalla scena.
## Chiamato all'uscita dalla modalità WAYPOINT (ESC) o al completamento del percorso.
func _clear_markers() -> void:
	# Rimuove tutti i marker e le loro etichette dalla scena
	for m in get_tree().get_nodes_in_group("markers"):
		m.queue_free()

# =============================================================================
# INVIO WAYPOINT AL BRIDGE
# =============================================================================

## Converte la lista waypoint in coordinate Alvik (cm) applicando l'allineamento
## e le invia al bridge tramite WebSocket come {type: "waypoints", points: [...]}.
## Il bridge eseguirà il path following proporzionale verso ogni punto.
func _send_waypoints() -> void:
	# Converte i waypoint da coordinate Godot a coordinate Alvik (cm)
	# applicando l'offset di allineamento e la rotazione.
	#
	# Sistema di riferimento:
	#   Godot: X=destra, Z=profondità (avanti del robot)
	#   Alvik: x=avanti, y=sinistra (sistema mobile, ruota con il robot)
	#
	# La trasformazione applica:
	#   1. Sottrae l'offset di allineamento (traslazione)
	#   2. Ruota del valore align_rot_y (correzione orientamento)
	#   3. Converte da metri a cm (divide per CM_TO_M)

	var payload = []
	var rot   = deg_to_rad(align_rot_y)
	var cos_r = cos(rot)
	var sin_r = sin(rot)

	for p in waypoint_list:
		# Sottrai offset allineamento
		var gx = p.x - align_offset.x
		var gz = p.z - align_offset.z

		# Scambia assi: in Alvik x=avanti(Z Godot), y=destra(X Godot)
		var base_ax = gz
		var base_ay = gx

		# Applica rotazione allineamento
		var ax = base_ax * cos_r + base_ay * sin_r
		var ay = -base_ax * sin_r + base_ay * cos_r

		payload.append({"x": ax / CM_TO_M, "y": ay / CM_TO_M})

	_ws_send({"type": "waypoints", "points": payload})
	_log("[color=green]Percorso inviato[/color] (%d punti)" % waypoint_list.size())
	# I marker rimangono visibili durante la navigazione
	# _clear_markers() viene chiamato al completamento del percorso
	coord_label.visible = false
	waypoint_list.clear()
	current_mode = Mode.NONE
	_update_info_label()

# =============================================================================
# PATH PREDEFINITI
# =============================================================================

## Genera e invia un path predefinito (quadrato, cerchio, triangolo).
## Il tipo è "quadrato"/"cerchio"/"triangolo", size_cm è la dimensione in cm.
## Legge theta iniziale da DDS e lo aggiunge all'ultimo waypoint per il ritorno
## alla posa iniziale con orientamento corretto.
func _send_path(tipo: String, size_cm: float) -> void:
	# Genera i waypoint per il percorso scelto e li invia al bridge.
	# I punti vengono inviati direttamente come coordinate Alvik (cm)
	# a partire dalla posizione corrente (origine = posizione attuale di Alvik).
	# L'ultimo waypoint è sempre (0,0,theta_iniziale) — ritorno alla posa iniziale.
	#
	# Il campo "theta" nell'ultimo waypoint indica al bridge l'orientamento
	# finale desiderato — Alvik tornerà con lo stesso angolo di partenza.

	# Legge il theta corrente come orientamento iniziale da ripristinare
	var theta_iniziale = DDS.read("Theta")
	if theta_iniziale == null:
		theta_iniziale = 0.0

	var points = []
	match tipo:
		"quadrato":
			# 4 angoli del quadrato, in senso antiorario
			# L'ultimo punto (0,0) riporta Alvik alla posa iniziale
			points = [
				{"x": size_cm, "y": 0.0},
				{"x": size_cm, "y": size_cm},
				{"x": 0.0,     "y": size_cm},
				{"x": 0.0,     "y": 0.0, "theta": theta_iniziale},
			]
		"cerchio":
			# 12 punti equidistanti sulla circonferenza (passo 30°)
			var n = 12
			for i in range(1, n + 1):
				var angle = 2.0 * PI * i / n
				points.append({
					"x": size_cm * cos(angle),
					"y": size_cm * sin(angle),
				})
			# Aggiunge ritorno esplicito all'origine con theta iniziale
			points.append({"x": 0.0, "y": 0.0, "theta": theta_iniziale})
		"triangolo":
			# Triangolo equilatero: altezza = lato * sqrt(3)/2
			# L'ultimo punto (0,0) riporta Alvik alla posa iniziale
			var h = size_cm * sqrt(3.0) / 2.0
			points = [
				{"x": size_cm,       "y": 0.0},
				{"x": size_cm / 2.0, "y": h},
				{"x": 0.0,           "y": 0.0, "theta": theta_iniziale},
			]

	_ws_send({"type": "waypoints", "points": points})
	_log("[color=cyan]Path %s inviato (%.0fcm) — ritorno all'origine[/color]" % [tipo, size_cm])

# =============================================================================
# COORDINATE LABEL
# =============================================================================

## Aggiorna la label coordinate (CoordLabel) con le coordinate Godot e Alvik
## del punto sotto il cursore. Attiva in modalità WAYPOINT o con tasto H.
func _update_coord_label(mouse_pos: Vector2) -> void:
	# Mostra le coordinate Godot e Alvik vicino al cursore.
	# Attivo in modalità WAYPOINT sempre, oppure in qualsiasi modalità con H.

	var active = (current_mode == Mode.WAYPOINT) or _mouse_coords_visible
	if not active:
		coord_label.visible = false
		return

	var result = _raycast_full(mouse_pos)
	if result.is_empty():
		coord_label.visible = false
		return

	var point = result.get("position")
	if point == null:
		coord_label.visible = false
		return

	var gx    = point.x
	var gz    = point.z
	var rot   = deg_to_rad(align_rot_y)
	var cos_r = cos(rot)
	var sin_r = sin(rot)
	var pgx   = gx - align_offset.x
	var pgz   = gz - align_offset.z
	var ax    =  pgz * cos_r + pgx * sin_r
	var ay    = -pgz * sin_r + pgx * cos_r

	coord_label.text     = "Godot: (%.2f, %.2f)\nAlvik: (%.1f, %.1f) cm" % [
		gx, gz, ax / CM_TO_M, ay / CM_TO_M]
	coord_label.position = mouse_pos + Vector2(15, -50)
	coord_label.visible  = true

# =============================================================================
# TELEOP — invio comandi
# =============================================================================

## Invia il comando teleop corrente al bridge in base ai tasti W/X/Q/E premuti.
## Chiamata a 20Hz dal timer in _process(). Invia {type:"cmd", vlin, vang}.
func _send_teleop() -> void:
	# Invia il comando di velocità corrente basato sui tasti premuti.
	# Chiamata ogni TELEOP_SEND_EVERY secondi da _process().
	# Se nessun tasto è premuto invia vlin=0, vang=0 (stop).
	var vlin = 0.0
	var vang = 0.0
	if _key_w: vlin =  TELEOP_VEL_LIN
	if _key_x: vlin = -TELEOP_VEL_LIN
	if _key_q: vang =  TELEOP_VEL_ANG
	if _key_e: vang = -TELEOP_VEL_ANG
	_ws_send({"type": "cmd", "vlin": vlin, "vang": vang})

# =============================================================================
# INVIO POSA
# =============================================================================

## Invia la posa corrente di Alvik al bridge ogni POSE_SEND_EVERY frame (~16Hz).
## Il bridge usa questa posa per il path following proporzionale.
func _send_pose() -> void:
	# Legge la posa corrente di Alvik dal DDS (autoload) e la invia al bridge.
	# Il bridge usa questa posa per il path following proporzionale:
	# calcola la direzione e la distanza dal prossimo waypoint.
	#
	# I valori X, Y, Theta vengono pubblicati da alvik_ros_bridge.py
	# leggendo la telemetria CSV da Alvik via UDP e distribuendola
	# tramite il protocollo DDS binario custom.

	if robot_node == null or _socket.get_ready_state() != WebSocketPeer.STATE_OPEN:
		return

	var x     = DDS.read("X")
	var y     = DDS.read("Y")
	var theta = DDS.read("Theta")

	# DDS.read() restituisce null se il valore non è ancora disponibile
	if x == null or y == null or theta == null:
		return

	_ws_send({"type": "pose", "x": x, "y": y, "theta": theta})

# =============================================================================
# MENU
# =============================================================================

## Mostra il menu principale con tutti i comandi disponibili (tasto M).
func _show_menu() -> void:
	menu_label.visible        = true
	menu_label.bbcode_enabled = true
	menu_label.text = """[center][b]ALVIK DIGITAL TWIN v1.7[/b][/center]

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[color=cyan][b]NAVIGAZIONE[/b][/color]
[left]
						[b]1[/b]  Vista dall'alto        [b]4[/b]  Laterale destra
						[b]2[/b]  Vista dietro            [b]5[/b]  Laterale sinistra
						[b]3[/b]  Prima persona      [b]Rotellina[/b]  Zoom
[/left]

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[color=green][b]MODALITA[/b][/color]
[left]
						[b]S[/b]  Waypoint manuale			(Esc per uscire)
						[b]T[/b]  Teleop W/X/Q/E			    (Esc o T per uscire)
						[b]P[/b]  Path predefiniti			    (Esc per uscire)
						[b]A[/b]  Allineamento				    (Esc o A per uscire)
						[b]E[/b]  Esploratore SLAM+PID	(Esc o E per uscire)
						[b]L[/b]  SLAM standalone			(Esc o L per uscire)
						[b]N[/b]  Nav2 autonomo				(Esc o N per uscire)
						[b]Spazio[/b]	Invia percorso waypoint
[/left]

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[color=yellow][b]SISTEMA[/b][/color]
[left]
						[b]R[/b]  Reset posizione
						[b]C[/b]  Cancella mappa ostacoli
						[b]G[/b]  Griglia + freccia posa iniziale
						[b]H[/b]  Coordinate mouse ON/OFF
						[b]M[/b]  Mostra questo menu
						[b]Shift[/b]  Nascondi/mostra UI
						[b]TAB[/b]  Nascondi/mostra Log Label
						[b]Esc[/b]  Ferma robot / Esci modalita
[/left]

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[center][color=green]Premi [b]ENTER[/b] per continuare[/color][/center]"""

## Mostra il sottomenù di selezione tipo path (Q=quadrato, C=cerchio, T=triangolo).
func _show_path_type_menu() -> void:
	menu_label.visible = true
	menu_label.text = """[center][b]PATH PREDEFINITI — Tipo[/b][/center]

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[b]4[/b]  Quadrato
[b]5[/b]  Cerchio
[b]6[/b]  Triangolo equilatero

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[center][color=yellow]Esc per annullare[/color][/center]"""

## Mostra il sottomenù di selezione dimensione path (1=piccolo ... 4=extra grande).
func _show_path_size_menu() -> void:
	menu_label.visible = true
	menu_label.text = """[center][b]PATH PREDEFINITI — Dimensione[/b][/center]
[center][color=cyan]%s[/color][/center]

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[b]1[/b]  25 cm
[b]2[/b]  50 cm
[b]3[/b]  75 cm
[b]4[/b]  100 cm

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[center][color=yellow]Esc per tornare indietro[/color][/center]""" % _path_type.to_upper()

## Mostra il dialogo di uscita dall'Esploratore con opzione salvataggio mappa.
func _show_explorer_exit_dialog() -> void:
	menu_label.visible = true
	menu_label.text = """[center][b]USCITA MODALITA ESPLORATORE[/b][/center]

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]

  Vuoi salvare la mappa costruita?

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[b]S[/b]  Salva mappa ed esci
[b]N[/b]  Esci senza salvare
[b]C[/b]  Annulla — rimani in Esploratore

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]"""

## Mostra il dialogo di uscita dallo SLAM con opzione salvataggio mappa.
func _show_slam_exit_dialog() -> void:
	# Dialogo identico all'Esploratore ma per la modalità SLAM standalone
	menu_label.visible = true
	menu_label.text = """[center][b]USCITA MODALITA SLAM[/b][/center]

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]

  Vuoi salvare la mappa costruita?

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]
[b]S[/b]  Salva mappa ed esci
[b]N[/b]  Esci senza salvare
[b]C[/b]  Annulla — rimani in SLAM

[color=gray]━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━[/color]"""

# =============================================================================
# WEBSOCKET — connessione e invio
# =============================================================================

## Avvia la connessione WebSocket verso il bridge (ws://127.0.0.1:8765).
func _ws_connect() -> void:
	_socket.connect_to_url(ws_url)

## Gestisce la macchina a stati WebSocket: polling, riconnessione automatica.
func _handle_ws(delta: float) -> void:
	# Gestisce lo stato della connessione con riconnessione automatica.
	# Se la connessione cade, aspetta RECONNECT_DELAY secondi e riprova.
	match _socket.get_ready_state():
		WebSocketPeer.STATE_OPEN:
			# Connessione attiva — azzera il timer di riconnessione
			_reconnect_timer = 0.0
		WebSocketPeer.STATE_CONNECTING:
			# Connessione in corso — aspetta
			pass
		WebSocketPeer.STATE_CLOSED:
			# Connessione chiusa — conta il tempo e riconnetti
			_reconnect_timer += delta
			if _reconnect_timer >= RECONNECT_DELAY:
				_reconnect_timer = 0.0
				_ws_connect()

## Serializza e invia un Dictionary come JSON al bridge via WebSocket.
func _ws_send(data: Dictionary) -> void:
	# Invia un messaggio JSON al bridge solo se la connessione è aperta
	if _socket.get_ready_state() == WebSocketPeer.STATE_OPEN:
		_socket.send_text(JSON.stringify(data))

# =============================================================================
# WEBSOCKET — ricezione pacchetti dal bridge
# =============================================================================

## Svuota la coda pacchetti WebSocket e processa i messaggi in arrivo dal bridge:
## log, waypoint_reached, explorer_started/stopped, slam_started/stopped,
## nav2_started/stopped, nav2_goal_reached/failed.
func _process_ws_packets() -> void:
	# Processa tutti i pacchetti in arrivo dal bridge in un singolo frame.
	# Il bridge invia messaggi JSON con "type" per identificare il contenuto.
	while _socket.get_available_packet_count() > 0:
		var raw  = _socket.get_packet().get_string_from_utf8()
		var data = JSON.parse_string(raw)
		if data == null:
			continue

		match data.get("type"):
			"log":
				# Messaggio di log dal bridge — visualizza nella console a schermo
				var lvl   = data.get("level", "info")
				var msg   = data.get("msg", "")
				var color = {"ok": "green", "warn": "yellow",
							 "error": "red"}.get(lvl, "gray")
				_log("[color=%s][Bridge] %s[/color]" % [color, msg])

			"waypoint_reached":
				# Il bridge ha raggiunto un waypoint — mostra progresso
				var idx   = int(data.get("index", 0)) + 1
				var total = int(data.get("total", 0))
				_log("[color=cyan]Waypoint %d/%d raggiunto[/color]" % [idx, total])
				if idx >= total:
					_clear_markers()   # rimuove marker al completamento

			"explorer_started":
				_log("[color=cyan]Esploratore attivo — SLAM + PID pronti[/color]")

			"explorer_stopped":
				_log("[color=yellow]Esploratore terminato[/color]")

			"slam_started":
				# SLAM standalone avviato — ora si può mappare con teleop
				_log("[color=cyan]SLAM attivo — usa T per teleop e mappare[/color]")

			"slam_stopped":
				_log("[color=yellow]SLAM terminato[/color]")

			"nav2_started":
				# Nav2 pronto — si può cliccare sulla mappa per il goal
				_log("[color=cyan]Nav2 pronto — clicca sulla mappa per il goal[/color]")

			"nav2_stopped":
				_log("[color=yellow]Nav2 fermato[/color]")

			"nav2_goal_reached":
				# Nav2 ha raggiunto il goal
				_log("[color=green]Nav2: goal raggiunto![/color]")

			"nav2_goal_failed":
				# Nav2 non è riuscito a raggiungere il goal
				_log("[color=red]Nav2: impossibile raggiungere il goal[/color]")

# =============================================================================
# INFO LABEL — stato modalità corrente
# =============================================================================

## Aggiorna l'InfoLabel (UI destra) con la modalità corrente e le istruzioni.
func _update_info_label() -> void:
	# Mostra nella label a destra la modalità corrente e i comandi disponibili.
	# Viene chiamata ogni volta che cambia la modalità.
	_log_history.clear()

	match current_mode:
		Mode.ALIGN:
			_log("[b][color=yellow]ALLINEAMENTO[/color][/b]", false)
			_log("Frecce: sposta/ruota  R: azzera  Esc/A: conferma", false)
		Mode.TELEOP:
			_log("[b][color=magenta]TELEOP[/color][/b]", false)
			_log("W/X avanza/indietro  Q/E ruota  Esc/T: esci", false)
		Mode.WAYPOINT:
			_log("[b][color=cyan]WAYPOINT[/color][/b]", false)
			_log("Clicca sul pavimento  Spazio: invia  Esc/S: esci", false)
		Mode.PATH:
			_log("[b][color=cyan]PATH PREDEFINITI[/color][/b]", false)
		Mode.EXPLORER:
			_log("[b][color=cyan]ESPLORATORE[/color][/b]", false)
			_log("SLAM + PID attivi  Esc/E: esci", false)
		Mode.SLAM:
			_log("[b][color=cyan]SLAM STANDALONE[/color][/b]", false)
			_log("Usa T per teleop  Esc/L: esci", false)
		Mode.NAV2:
			_log("[b][color=cyan]NAV2 AUTONOMO[/color][/b]", false)
			_log("Clicca sulla mappa per goal  Esc/N: esci", false)
		_:
			var view_name = ["Dall'alto", "Dietro", "Prima persona", "Laterale destra", "Laterale sinistra"][current_view]
			_log("[b]Vista:[/b] %s  M=menu  Esc=stop" % view_name, false)

	info_label.text = "\n".join(_log_history)

# =============================================================================
# GRIGLIA
# =============================================================================

## Disegna la griglia 3D sul pavimento con linee ogni 25cm (bianco) e 1m (grigio).
## Attivata con il tasto G insieme alla freccia di posa iniziale.
func _draw_grid() -> void:
	# Disegna la griglia con ImmediateMesh — linee sul piano Y=0.01 (leggermente
	# sopra il pavimento per evitare z-fighting).
	# Prima disegna le linee minori (25cm) poi le maggiori (1m) sopra.

	_clear_grid()

	var mesh     = ImmediateMesh.new()
	_grid_instance           = MeshInstance3D.new()
	_grid_instance.mesh      = mesh
	_grid_instance.name      = "GridOverlay"

	# Materiale con vertex_color_use_as_albedo per colorare le linee
	var mat = StandardMaterial3D.new()
	mat.shading_mode             = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.transparency             = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.vertex_color_use_as_albedo = true
	_grid_instance.material_override = mat

	var y = 0.01   # leggermente sopra il pavimento

	# ── Linee minori 25cm ────────────────────────────────────────────────────
	mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	var x_min = -GRID_SIZE
	var x_max =  GRID_SIZE
	var z_min = -GRID_SIZE
	var z_max =  GRID_SIZE

	var pos = x_min
	while pos <= x_max + 0.001:
		# Salta le linee che cadono su un multiplo di 1m (saranno disegnate dopo)
		if not _is_near_multiple(pos, GRID_MAJOR):
			mesh.surface_set_color(GRID_COLOR_MINOR)
			mesh.surface_add_vertex(Vector3(pos, y, z_min))
			mesh.surface_set_color(GRID_COLOR_MINOR)
			mesh.surface_add_vertex(Vector3(pos, y, z_max))
		pos += GRID_MINOR

	pos = z_min
	while pos <= z_max + 0.001:
		if not _is_near_multiple(pos, GRID_MAJOR):
			mesh.surface_set_color(GRID_COLOR_MINOR)
			mesh.surface_add_vertex(Vector3(x_min, y, pos))
			mesh.surface_set_color(GRID_COLOR_MINOR)
			mesh.surface_add_vertex(Vector3(x_max, y, pos))
		pos += GRID_MINOR
	mesh.surface_end()

	# ── Linee maggiori 1m ────────────────────────────────────────────────────
	mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	pos = x_min
	while pos <= x_max + 0.001:
		if _is_near_multiple(pos, GRID_MAJOR):
			mesh.surface_set_color(GRID_COLOR_MAJOR)
			mesh.surface_add_vertex(Vector3(pos, y, z_min))
			mesh.surface_set_color(GRID_COLOR_MAJOR)
			mesh.surface_add_vertex(Vector3(pos, y, z_max))
		pos += GRID_MINOR

	pos = z_min
	while pos <= z_max + 0.001:
		if _is_near_multiple(pos, GRID_MAJOR):
			mesh.surface_set_color(GRID_COLOR_MAJOR)
			mesh.surface_add_vertex(Vector3(x_min, y, pos))
			mesh.surface_set_color(GRID_COLOR_MAJOR)
			mesh.surface_add_vertex(Vector3(x_max, y, pos))
		pos += GRID_MINOR
	mesh.surface_end()

	get_tree().current_scene.add_child(_grid_instance)

## Restituisce true se val è vicino a un multiplo di step (±1%).
## Usato da _draw_grid() per distinguere linee principali (1m) da secondarie (25cm).
func _is_near_multiple(val: float, step: float) -> bool:
	# Restituisce true se val è vicino a un multiplo di step (tolleranza 1mm)
	var remainder = fmod(abs(val), step)
	return remainder < 0.001 or remainder > step - 0.001

## Rimuove la griglia 3D dalla scena.
func _clear_grid() -> void:
	if _grid_instance != null and is_instance_valid(_grid_instance):
		_grid_instance.queue_free()
		_grid_instance = null

# queue_free() in Godot non elimina il nodo immediatamente — lo marca per l'eliminazione 
# e Godot lo rimuove in modo sicuro alla fine del frame corrente, dopo che tutti i nodi 
# hanno terminato _process().
# Questo è importante perché eliminare un nodo durante _process() mentre altri nodi potrebbero 
# ancora riferirlo causerebbe un crash.
#
#Il doppio controllo serve per sicurezza:
# — _grid_instance != null — verifica che la variabile non sia null (mai inizializzata)
# — is_instance_valid(_grid_instance) — verifica che il nodo esista ancora nella scena,
#   perché potrebbe essere già stato eliminato da un queue_free() precedente — 
#   in quel caso la variabile non è null ma il nodo non è più valido

# =============================================================================
# FRECCIA POSA INIZIALE
# =============================================================================

## Disegna la freccia 3D che indica la posa iniziale (origine + orientamento).
## La freccia è composta da linee ImmediateMesh colorate.
func _draw_origin_arrow() -> void:
	# Freccia rossa piatta sul pavimento che indica la posa iniziale di Alvik.
	# Usa ImmediateMesh con PRIMITIVE_TRIANGLES — geometria 2D sul piano XZ
	# quindi non serve nessuna rotazione del mesh, solo quella del nodo parent.
	# La freccia punta verso -Z (avanti di Alvik in Godot).

	_clear_origin_arrow()

	_origin_pose = robot_node.global_position if robot_node else Vector3.ZERO
	_origin_rot  = robot_node.rotation.y if robot_node else 0.0

	_origin_arrow = Node3D.new()
	_origin_arrow.name = "OriginArrow"

	var mesh = ImmediateMesh.new()
	var inst = MeshInstance3D.new()
	inst.mesh = mesh

	var mat = StandardMaterial3D.new()
	mat.shading_mode  = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.transparency  = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.albedo_color  = Color(1.0, 0.1, 0.1, 0.9)
	mat.cull_mode     = BaseMaterial3D.CULL_DISABLED
	inst.material_override = mat

	var y = 0.02   # appena sopra il pavimento

	# Corpo: rettangolo stretto
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
	mesh.surface_add_vertex(Vector3(-0.015, y,  0.0))
	mesh.surface_add_vertex(Vector3( 0.015, y,  0.0))
	mesh.surface_add_vertex(Vector3( 0.015, y, -0.125))
	mesh.surface_add_vertex(Vector3(-0.015, y,  0.0))
	mesh.surface_add_vertex(Vector3( 0.015, y, -0.125))
	mesh.surface_add_vertex(Vector3(-0.015, y, -0.125))
	# Punta: triangolo largo verso -Z
	mesh.surface_add_vertex(Vector3(-0.045, y, -0.125))
	mesh.surface_add_vertex(Vector3( 0.045, y, -0.125))
	mesh.surface_add_vertex(Vector3( 0.0,   y, -0.225))
	mesh.surface_end()

	_origin_arrow.add_child(inst)
	get_tree().current_scene.add_child(_origin_arrow)
	_origin_arrow.global_position = Vector3(_origin_pose.x, 0.0, _origin_pose.z)
	_origin_arrow.rotation        = Vector3(0, _origin_rot + PI, 0)

## Rimuove la freccia di posa iniziale dalla scena.
func _clear_origin_arrow() -> void:
	if _origin_arrow != null and is_instance_valid(_origin_arrow):
		_origin_arrow.queue_free()
		_origin_arrow = null

# =============================================================================
# LOG — console a schermo
# =============================================================================

## Aggiunge un messaggio al log UI destra (InfoLabel) con timestamp opzionale.
## Supporta BBCode per colori e grassetto.
func _log(text: String, show_time: bool = true) -> void:
	# Aggiunge una riga al log a schermo con timestamp opzionale.
	# Le righe più vecchie vengono rimosse quando si supera max_lines.
	# push_front() mette il messaggio più recente in cima.
	var line = ""
	if show_time:
		var t = Time.get_time_string_from_system()
		line = "[color=gray][%s][/color] %s" % [t, text]
	else:
		line = text

	_log_history.push_front(line)
	if _log_history.size() > max_lines:
		_log_history.pop_back()

	info_label.text = "\n".join(_log_history)
