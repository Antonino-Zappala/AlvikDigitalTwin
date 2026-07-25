## digital_twin_v1_7.gd
## =====================
## Script collegato al nodo "Alvik" nella scena AlvikRobotEnv.
##
## Gestisce la logica di visualizzazione del Digital Twin:
##  - Aggiornamento posizione e rotazione del robot in base ai dati DDS
##  - Animazione delle ruote tramite valori encoder (WheelLeft, WheelRight)
##  - Visualizzazione dei 5 raggi ToF con colore in base alla distanza
##  - Generazione della mappa ostacoli dinamica (ObstacleMap) dai sensori ToF
##  - Aggiornamento della UI sinistra (DebugLabel) con posa e stato connessione
##  - Gestione della connessione WebSocket con il bridge (riconnessione automatica)
##
## Dipendenze: DDS (Autoload), bridge_v1_7.py (WebSocket :8765)

extends Node3D

# ── Configurazione esportata (modificabile dall'Inspector di Godot) ────────────

@export var socket_url         = "ws://127.0.0.1:8765"  # indirizzo bridge WebSocket
@export var max_lines          = 6                       # righe massime nel log UI
@export var is_debug_enabled   = true                    # abilita log UI sinistra
@export var reconnect_delay    = 3.0                     # secondi tra tentativi riconnessione
@export var connection_timeout = 5.0                     # timeout connessione WebSocket (s)

# ── Costanti ToF ──────────────────────────────────────────────────────────────

const TOF_MAX_CM  = 200.0   # distanza massima ToF (cm)
const TOF_WARN_CM = 30.0    # soglia avviso (raggio giallo)
const TOF_STOP_CM = 15.0    # soglia stop firmware (raggio rosso)
const TOF_SCALE   = 0.01    # fattore scala cm → metri Godot
const CM_TO_M     = 0.01    # conversione cm → metri

# ── Costanti mappa ostacoli ───────────────────────────────────────────────────

const GRID_SIZE_M  = 0.05    # risoluzione griglia ostacoli (5cm per cella)
const OBS_SPAWN_CM = 15.0    # distanza sotto cui si spawna un ostacolo (cm)
const OBS_BOX_W    = 0.04    # larghezza box ostacolo (m)
const OBS_BOX_H    = 0.10    # altezza box ostacolo (m)
const OBS_BOX_D    = 0.04    # profondità box ostacolo (m)
const OBS_FADE_TIME = 10.0   # secondi prima che un ostacolo inizi a sfumare
const OBS_MIN_ALPHA = 0.15   # opacità minima degli ostacoli più vecchi

# ── Colori raggi ToF ──────────────────────────────────────────────────────────

const COLOR_FREE = Color(0.0, 1.0, 0.0, 1.0)   # verde — distanza sicura
const COLOR_WARN = Color(1.0, 0.7, 0.0, 1.0)   # arancione — avviso
const COLOR_STOP = Color(1.0, 0.0, 0.0, 1.0)   # rosso — soglia stop firmware

# ── Riferimenti nodi della scena ──────────────────────────────────────────────

@onready var debug_label : RichTextLabel = \
	get_node("/root/AlvikRobotEnv/CanvasLayer/DebugLabel")   # UI sinistra

# Ruote — mesh separate per animazione indipendente
@onready var wheel_left  = $Wheel_Left_001
@onready var wheel_right = $Wheel_Right_001

# RayCast3D per i 5 sensori ToF
@onready var ray_L  : RayCast3D = $RayCast_L    # sensore sinistro (-45°)
@onready var ray_CL : RayCast3D = $RayCast_CL   # sensore centro-sinistra (-20°)
@onready var ray_C  : RayCast3D = $RayCast_C    # sensore centrale (0°)
@onready var ray_CR : RayCast3D = $RayCast_CR   # sensore centro-destra (+20°)
@onready var ray_R  : RayCast3D = $RayCast_R    # sensore destro (+45°)

# Contenitore ostacoli dinamici — nodo ObstacleMap nella scena
@onready var _obs_container : Node3D = \
	get_node("/root/AlvikRobotEnv/ObstacleMap")

# ── Variabili interne ─────────────────────────────────────────────────────────

var socket          = WebSocketPeer.new()   # client WebSocket verso il bridge
var message_history = []                    # storico messaggi UI (max max_lines)
var reconnect_timer = 0.0                   # timer riconnessione automatica
var timeout_timer   = 0.0                   # timer timeout connessione
var attempts        = 0                     # contatore tentativi di connessione

# Mesh ImmediateMesh per disegnare i raggi ToF come linee 3D
var _tof_mesh     : ImmediateMesh
var _tof_instance : MeshInstance3D

# Parametri di allineamento — aggiornati dalla modalità ALIGN
var align_offset : Vector3 = Vector3.ZERO   # offset traslazione allineamento
var align_rot_y  : float   = 0.0            # rotazione allineamento (gradi)

# Mappa ostacoli: Vector2i(gx, gz) → {"node": StaticBody3D, "ttl": float, "hits": int}
var _obs_map : Dictionary = {}

# Timer per limitare il log di posa a 1Hz (evita aggiornamento a 50Hz)
var _log_pose_timer : float = 0.0

# ── Setup ─────────────────────────────────────────────────────────────────────

## Inizializzazione: sottoscrizione variabili DDS, creazione mesh ToF,
## verifica container ostacoli, avvio connessione WebSocket.
func _ready() -> void:
	debug_label.bbcode_enabled = true

	# Sottoscrizione variabili DDS
	DDS.subscribe("X")
	DDS.subscribe("Y")
	DDS.subscribe("Theta")
	DDS.subscribe("WheelLeft")    # velocità ruota sinistra (rad/s)
	DDS.subscribe("WheelRight")   # velocità ruota destra (rad/s)
	DDS.subscribe("ToF_L")
	DDS.subscribe("ToF_CL")
	DDS.subscribe("ToF_C")
	DDS.subscribe("ToF_CR")
	DDS.subscribe("ToF_R")
	DDS.subscribe("TickId")       # -1 = stale, >0 = Alvik connesso

	# Crea ImmediateMesh per i raggi ToF — ridisegnata ogni frame
	_tof_mesh               = ImmediateMesh.new()
	_tof_instance           = MeshInstance3D.new()
	_tof_instance.mesh      = _tof_mesh
	_tof_instance.name      = "ToFLines"
	var mat                 = StandardMaterial3D.new()
	mat.shading_mode        = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	mat.transparency        = BaseMaterial3D.TRANSPARENCY_ALPHA
	_tof_instance.material_override = mat
	add_child.call_deferred(_tof_instance)

	# Verifica container ostacoli
	if _obs_container == null:
		push_error("[OBS] Nodo ObstacleMap non trovato in scena!")
	else:
		print("[OBS] Container pronto: ", _obs_container.get_path())

	_connect_to_bridge()
	log_debug("[color=cyan]Sistema pronto[/color]")
	
	

# ── Process ───────────────────────────────────────────────────────────────────

## Loop principale — eseguito ogni frame (~60Hz).
## Aggiorna: WebSocket, posa robot, animazione ruote, raggi ToF, mappa ostacoli, UI.
func _process(delta: float) -> void:
	# WebSocket
	socket.poll()
	var state = socket.get_ready_state()
	_handle_socket_logic(state, delta)

	# Tick DDS
	DDS.publish("tick", DDS.DDS_TYPE_FLOAT, delta)

	# Lettura dati dal DDS (aggiornati a 50Hz dal bridge)
	var x     = DDS.read("X")
	var y     = DDS.read("Y")
	var theta = DDS.read("Theta")
	var wl    = DDS.read("WheelLeft")
	var wr    = DDS.read("WheelRight")

	var tof_l  = DDS.read("ToF_L")
	var tof_cl = DDS.read("ToF_CL")
	var tof_c  = DDS.read("ToF_C")
	var tof_cr = DDS.read("ToF_CR")
	var tof_r  = DDS.read("ToF_R")

	# Aggiornamento posa — CM_TO_M converte cm in metri
	if x != null and y != null and theta != null:
		print("[MOVE] x=%.3f y=%.3f theta=%.4f align_rot_y=%.1f" % [x, y, theta, align_rot_y])
		# Posizione base in coordinate Godot (Alvik X→Godot Z, Alvik Y→Godot X)
		var base_x = y * CM_TO_M   # Godot X
		var base_z = x * CM_TO_M   # Godot Z

		# Applica rotazione di allineamento (modalità ALIGN)
		var rot   = deg_to_rad(align_rot_y)
		var cos_r = cos(rot)
		var sin_r = sin(rot)
		var rotated_x =  base_x * cos_r + base_z * sin_r
		var rotated_z = -base_x * sin_r + base_z * cos_r

		# Applica offset di traslazione
		position   = Vector3(rotated_x, 0.05, rotated_z) + align_offset
		rotation.y = theta + deg_to_rad(align_rot_y)

		# Animazione ruote — il segno negativo compensa l'orientamento Blender
		if wl != null and wr != null:
			# wheel_left.rotation.x  = wl
			# wheel_right.rotation.x = wr
			wheel_left.rotation.x  -= wl * delta
			wheel_right.rotation.x -= wr * delta

		# Log posa a 1Hz		
		_log_pose_timer += delta
		if _log_pose_timer >= 0: # _log_pose_timer >= 1: per avere i log aggiornati ogni secondo (1 Hz) 
			_log_pose_timer = 0.0
			if is_debug_enabled:
				log_debug("Posa: X:%.2f Y:%.2f T:%.3f" % [
					x * CM_TO_M, y * CM_TO_M, theta
				])
			if tof_c != null:
				log_debug("ToF [%.0f|%.0f|%.0f|%.0f|%.0f] cm" % [
					tof_l  if tof_l  != null else -1.0,
					tof_cl if tof_cl != null else -1.0,
					tof_c,
					tof_cr if tof_cr != null else -1.0,
					tof_r  if tof_r  != null else -1.0,
				])

	# Raggi ToF
	_update_tof_rays(tof_l, tof_cl, tof_c, tof_cr, tof_r)
	
	# Mappa ostacoli dinamica
	_update_obstacle_map([tof_l, tof_cl, tof_c, tof_cr, tof_r], delta)

	
	if is_debug_enabled:
		_update_debug_ui(state)

# ── Raggi ToF ─────────────────────────────────────────────────────────────────

## Aggiorna i 5 RayCast3D e disegna le linee ToF colorate.
## Verde = libero (>30cm), Arancione = avviso (15-30cm), Rosso = stop (<15cm).
func _update_tof_rays(tof_l, tof_cl, tof_c, tof_cr, tof_r) -> void:
	# [RayCast3D, valore ToF, angolo in radianti]
	# Angoli: positivo = destra, negativo = sinistra (sistema Godot)
	var rays = [
		[ray_L,  tof_l,  deg_to_rad(45.0)],
		[ray_CL, tof_cl, deg_to_rad(20.0)],
		[ray_C,  tof_c,  0.0              ],
		[ray_CR, tof_cr, deg_to_rad( -20.0)],
		[ray_R,  tof_r,  deg_to_rad( -45.0)],
	]

	_tof_mesh.clear_surfaces()
	_tof_mesh.surface_begin(Mesh.PRIMITIVE_LINES)

	for entry in rays:
		var ray   : RayCast3D = entry[0]
		var dist  : float     = clampf(
			float(entry[1]) if entry[1] != null else TOF_MAX_CM,
			1.0, TOF_MAX_CM
		)
		var angle : float = entry[2]

		# Direzione del raggio nel piano XZ locale del robot
		var dir = Vector3(sin(angle), 0.0, cos(angle))
		var endpoint = dir * dist * TOF_SCALE

		# Aggiorna RayCast3D
		ray.target_position = endpoint

		# Colore in base alla distanza
		var color : Color
		if dist < TOF_STOP_CM:
			color = COLOR_STOP
		elif dist < TOF_WARN_CM:
			color = COLOR_WARN
		else:
			color = COLOR_FREE

		# Disegna la linea: origine → endpoint
		_tof_mesh.surface_set_color(color)
		_tof_mesh.surface_add_vertex(Vector3.ZERO)
		_tof_mesh.surface_set_color(color)
		_tof_mesh.surface_add_vertex(endpoint)

	_tof_mesh.surface_end()

# ── Mappa ostacoli dinamica ───────────────────────────────────────────────────

## Aggiorna la mappa ostacoli in base alle letture ToF.
## Spawna StaticBody3D per ostacoli a <15cm. Ogni cella ha un sistema di
## confidenza basato su "hits" — più conferme = più opaco e più rosso.
## Gli ostacoli sfumano progressivamente dopo OBS_FADE_TIME secondi.
func _update_obstacle_map(tof_vals: Array, delta: float) -> void:
	if _obs_container == null:
		print("[OBS] _obs_container è null!")
		return
	if not _obs_container.is_inside_tree():
		print("[OBS] _obs_container non è ancora nell'albero")
		return
	# Sono due guardie di sicurezza all'inizio della funzione che prevengono crash:
	#
	# _obs_container == null — verifica che il nodo ObstacleMap sia stato trovato 
	# nella scena durante _ready(). Potrebbe essere null se il nodo è stato rinominato 
	# o rimosso dalla scena per errore.

	# _obs_container.is_inside_tree() — verifica che il nodo sia effettivamente 
	# presente nell'albero della scena in questo momento. Questo è necessario 
	# perché in Godot esiste una finestra temporale tra quando un nodo viene creato 
	# e quando viene effettivamente aggiunto all'albero. Se _process() viene chiamato 
	# prima che il nodo sia completamente inizializzato — cosa che può succedere nei 
	# primi frame — chiamare add_child() su un nodo non ancora nell'albero causerebbe 
	# un crash.
	#
	# In pratica la seconda guardia protegge da questo scenario:
	# Frame 1: _ready() eseguito, _obs_container non ancora nell'albero
	# Frame 1: _process() chiamato → senza la guardia → CRASH
	# Frame 2: _obs_container finalmente nell'albero → tutto OK


	var angles_deg = [45.0, 20.0, 0.0, -20.0, -45.0]

	# Aggiorna TTL (time to live) e sfuma ostacoli vecchi
	for key in _obs_map.keys():
		var cell = _obs_map[key]
		cell["ttl"] += delta
		if cell["ttl"] > OBS_FADE_TIME:
			var age   = cell["ttl"] - OBS_FADE_TIME
			var alpha = lerp(0.7, OBS_MIN_ALPHA, clampf(age / 8.0, 0.0, 1.0))
			var mi  : MeshInstance3D     = cell["node"].get_child(0)
			var mat : StandardMaterial3D = mi.material_override
			var c   = mat.albedo_color
			mat.albedo_color = Color(c.r, c.g, c.b, alpha)

	# Processa le 5 letture ToF
	for i in range(5):
		var dist : float = float(tof_vals[i]) if tof_vals[i] != null else 999.0
		# Il 999.0 è un valore sentinel — una distanza impossibile (molto maggiore di TOF_MAX_CM = 200.0)
		# che garantisce che la condizione if dist >= OBS_SPAWN_CM sia sempre vera, quindi nessun ostacolo 
		# viene spawnato quando il sensore non ha dati.
		
		if dist >= OBS_SPAWN_CM:
			continue

		# Calcola posizione mondo dell'ostacolo
		var angle_rad  = deg_to_rad(angles_deg[i])
		var global_ang = rotation.y + angle_rad
		var dir        = Vector3(sin(global_ang), 0.0, cos(global_ang))
		var world_pos  = position + dir * dist * TOF_SCALE
		world_pos.y    = OBS_BOX_H / 2.0

		# Snap alla griglia 5cm
		var gx  = roundi(world_pos.x / GRID_SIZE_M)
		var gz  = roundi(world_pos.z / GRID_SIZE_M)
		var key = Vector2i(gx, gz)

		if _obs_map.has(key):
			# Cella esistente — aumenta confidenza
			var cell     = _obs_map[key]
			cell["ttl"]  = 0.0
			cell["hits"] += 1

			# Più hits = più opaco e più rosso (conferma dell'ostacolo)
			var confidence = clampf(cell["hits"] / 5.0, 0.0, 1.0)
			var mi  : MeshInstance3D     = cell["node"].get_child(0)
			var mat : StandardMaterial3D = mi.material_override
			mat.albedo_color = Color(
				1.0,          # R sempre alto
				lerp(0.6, 0.0, confidence),          # G scende con confidence
				0.0,
				lerp(0.5, 0.9, confidence)           # più opaco con più hit
			)

		# Il colore viene costruito come Color(R, G, B, Alpha):

		# R = 1.0 — rosso fisso al massimo
		#
		# G = lerp(0.6, 0.0, confidence) — verde parte da 0.6 (arancione) 
		# e scende a 0 (rosso puro) man mano che la confidenza aumenta
		#
		# B = 0.0 — blu sempre zero
		#
		# Alpha = lerp(0.5, 0.9, confidence) — opacità parte da 0.5 (semitrasparente) 
		# e sale a 0.9 (quasi opaco) con più conferme

		# In pratica l'effetto visivo è:
		#	
		# Prima rilevazione → arancione semitrasparente (poca certezza)
		# Dopo 5+ conferme → rosso quasi opaco (alta certezza — ostacolo confermato)	
		
		
		#else:
			## Nuova cella — spawna parallelepipedo
			#var mi       = MeshInstance3D.new()
			#var box      = BoxMesh.new()
			#box.size     = Vector3(OBS_BOX_W, OBS_BOX_H, OBS_BOX_D)
			#mi.mesh      = box
			#mi.name      = "Obs_%d_%d" % [gx, gz]
			#
			## Orientato perpendicolare al raggio
			#mi.rotation.y = global_ang
			#
			#var mat = StandardMaterial3D.new()
			#mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
			#mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
			## Prima lettura: arancione semitrasparente
			#mat.albedo_color = Color(1.0, 0.6, 0.0, 0.5)
			#mi.material_override = mat
			#
			#get_tree().current_scene.add_child(mi)
			#mi.global_position = world_pos
			#
			#_obs_map[key] = {
				#"node": mi,
				#"ttl":  0.0,
				#"hits": 1,
			#}
		else:
			# Nuova cella — spawna StaticBody3D
			var node = _spawn_obstacle(world_pos, global_ang)
			_obs_map[key] = {
				"node": node,
				"ttl":  0.0,
				"hits": 1,
			}

## Spawna un ostacolo come StaticBody3D con mesh visiva e CollisionShape3D.
## collision_layer = 2 permette al raycast del click mouse di identificarlo
## come ostacolo (parent.name == "ObstacleMap") in selezione_waypoint_v1_7.gd.
func _spawn_obstacle(world_pos: Vector3, global_ang: float) -> Node3D:
	# Nodo radice ostacolo
	var root = StaticBody3D.new()
	root.name = "Obs_%d_%d" % [
		roundi(world_pos.x / GRID_SIZE_M),
		roundi(world_pos.z / GRID_SIZE_M)
	]

	# Mesh visiva
	var mi   = MeshInstance3D.new()
	var box  = BoxMesh.new()
	box.size = Vector3(OBS_BOX_W, OBS_BOX_H, OBS_BOX_D)
	mi.mesh  = box
	var mat  = StandardMaterial3D.new()
	mat.shading_mode     = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.transparency     = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.albedo_color     = Color(1.0, 0.6, 0.0, 0.5)
	mi.material_override = mat
	root.add_child(mi)
	
	# Collision shape — stessa dimensione del mesh
	var col       = CollisionShape3D.new()
	var box_shape = BoxShape3D.new()
	box_shape.size = Vector3(OBS_BOX_W, OBS_BOX_H, OBS_BOX_D)
	col.shape      = box_shape
	root.add_child(col)
	
	root.collision_layer = 2   # layer 2 = ostacoli
	root.collision_mask  = 0   # gli ostacoli non collidono tra loro
	
	# Aggiunto al contenitore — diventa figlio di ObstacleMap
	_obs_container.add_child(root)
	
	# Posizione e rotazione nel mondo
	root.global_position = world_pos
	root.rotation.y      = global_ang
	
	return root

## Rimuove tutti gli ostacoli dalla mappa e dalla scena.
func clear_obstacle_map() -> void:
	for key in _obs_map.keys():
		_obs_map[key]["node"].queue_free()
	_obs_map.clear()
	log_debug("[color=yellow]Mappa ostacoli pulita[/color]")

## Aggiorna i parametri di allineamento — chiamato da selezione_waypoint_v1_7.gd.
func set_alignment(offset: Vector3, rot_y: float) -> void:
	align_offset = offset
	align_rot_y  = rot_y

## Resetta l'allineamento all'origine.
func reset_pose_godot() -> void:
	align_offset = Vector3.ZERO
	align_rot_y  = 0.0
	log_debug("[color=yellow]Posa Godot resettata[/color]")
	
# ── WebSocket ─────────────────────────────────────────────────────────────────

## Avvia la connessione WebSocket verso il bridge.
func _connect_to_bridge() -> void:
	attempts      += 1
	timeout_timer  = 0.0
	var err = socket.connect_to_url(socket_url)
	if err != OK:
		log_debug("[color=red]WS errore connessione: %d[/color]" % err)

## Gestisce input globali: Tab (toggle UI sinistra), C (cancella ostacoli).
func _input(event: InputEvent) -> void:
	if event.is_action_pressed("ui_focus_next"):
		is_debug_enabled = !is_debug_enabled
		get_node("/root/AlvikRobotEnv/CanvasLayer").visible = is_debug_enabled

	# Pulisci mappa ostacoli
	if event is InputEventKey and event.keycode == KEY_C and event.pressed:
		clear_obstacle_map()
	
## Macchina a stati WebSocket: OPEN → processa, CONNECTING → timeout, CLOSED → riconnette.		
func _handle_socket_logic(state: int, delta: float) -> void:
	match state:
		WebSocketPeer.STATE_OPEN:
			attempts        = 0
			reconnect_timer = 0.0
			timeout_timer   = 0.0
			_process_packets()
		WebSocketPeer.STATE_CONNECTING:
			timeout_timer += delta
			if timeout_timer >= connection_timeout:
				socket.close()
		WebSocketPeer.STATE_CLOSED:
			reconnect_timer += delta
			if reconnect_timer >= reconnect_delay:
				reconnect_timer = 0.0
				_connect_to_bridge()

## Svuota la coda pacchetti WebSocket in arrivo dal bridge.
func _process_packets() -> void:
	while socket.get_available_packet_count() > 0:
		var _packet = socket.get_packet()

# ── UI ────────────────────────────────────────────────────────────────────────

## Aggiorna il DebugLabel con stato Bridge (WebSocket) e Alvik (DDS) + log.
func _update_debug_ui(state: int) -> void:
	# Bridge: stato della connessione WebSocket col middleware
	var bridge_label = ""
	match state:
		WebSocketPeer.STATE_OPEN:
			bridge_label = "[b]Bridge: [color=green]● CONNESSO[/color][/b]"
		WebSocketPeer.STATE_CONNECTING:
			bridge_label = "[b]Bridge: [color=yellow]◌ CONNESSIONE...[/color][/b]"
		_:
			bridge_label = "[b]Bridge: [color=red]✕ DISCONNESSO[/color][/b]"

	# Alvik connesso se ha inviato dati negli ultimi 2 secondi
	var alvik_label = ""
	if Time.get_ticks_msec() - DDS.last_receive_msec < 2000:
		alvik_label = "[b]Alvik:  [color=green]● CONNESSO[/color][/b]"
	else:
		alvik_label = "[b]Alvik:  [color=red]✕ NON CONNESSO[/color][/b]"

	debug_label.text = bridge_label + "\n" + alvik_label + "\n" + "\n".join(message_history)

## Aggiunge un messaggio al log UI con timestamp [HH:MM:SS.cs].
## Supporta BBCode per colori e grassetto.
func log_debug(new_text: String) -> void:
	var t = Time.get_time_string_from_system()
	# var ms = Time.get_ticks_msec() % 1000 / 100  # decimi di secondo 0-9
	# message_history.push_front("[color=gray][%s.%d][/color] %s" % [t, ms, new_text])
	var cs = Time.get_ticks_msec() % 1000 / 10  # centesimi di secondo 0-99
	message_history.push_front("[color=gray][%s.%d][/color] %s" % [t, cs, new_text])
	if message_history.size() > max_lines:
		message_history.pop_back()
