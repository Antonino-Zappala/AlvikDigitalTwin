# Alvik Digital Twin v1.7 — Development Log

**Data inizio:** Aprile 2026  
**Directory di lavoro:** `/home/neobot78/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/`

---

## Step 1 — Creazione struttura directory v1.7

```bash
mkdir -p ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7
```

Struttura finale:
```
AlvikDigitalTwin_v1_7/
├── alvik_start_v1_7.sh             ← script avvio completo ✅
├── alvik_test_odom.sh              ← test odometria standalone
├── alvik_test_nav2.sh              ← test Nav2
├── alvik_teleop_keyboard.py        ← teleop frecce tastiera
├── bridge_v1_7.py                  ← middleware unificato ✅
├── ros2/                           ← copia da progetto_alvik/ros2 ✅
│   ├── Dockerfile
│   ├── docker-compose.yml          ← path aggiornati ✅
│   └── src/alvik_bridge/
│       └── alvik_bridge/
│           ├── alvik_ros_bridge.py
│           ├── alvik_pid_controller.py
│           ├── alvik_launch.py
│           ├── nav2_params.yaml
│           └── slam_params.yaml
└── alvik_digital_twin_v1_7/        ← progetto Godot ✅
    ├── project.godot               ← autoload DDS, scena principale
    ├── alvik_robot.tscn            ← scena principale (uid://c3yvavrsvf5s5)
    ├── DDS_v1_5.gd                 ← da rinominare DDS_v1_7.gd
    ├── digital_twin_v1_5.gd        ← da rinominare digital_twin_v1_7.gd
    ├── selezione_waypoint_v1_6.gd  ← sostituire con selezione_waypoint_v1_7.gd ✅
    └── ... (altri file)
```

---

## Step 2 — Copia directory ros2

```bash
cp -r ~/Scrivania/progetto_alvik/ros2 \
      ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/ros2
```

---

## Step 3 — Aggiornamento path docker-compose.yml

```bash
sed -i 's|/home/neobot78/Scrivania/progetto_alvik/ros2|/home/neobot78/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/ros2|g' \
  ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/ros2/docker-compose.yml
```

---

## Step 4 — Fix odometria alvik_ros_bridge.py

Correzioni definitive al sistema di riferimento:

```python
self.theta = float(theta) + math.pi      # offset 180° sistema riferimento
self.vth   = -(wr - wl) / BASE_WIDTH_M * WHEEL_RADIUS_M  # rotazione invertita
odom.pose.pose.position.x = -self.x     # posizione negata
odom.pose.pose.position.y = -self.y
t.transform.translation.x = -self.x
t.transform.translation.y = -self.y
scan.header.frame_id = 'base_footprint'
WHEEL_RADIUS_M = 0.017
BASE_WIDTH_M   = 0.090
```

**Architettura sensori (annotazione tecnica permanente):**
- **ToF** → solo per rilevare ostacoli nella costmap Nav2 (obstacle layer)
  e per il blocco hardware a 15cm nel firmware Alvik
- **Odometria (encoder)** → unica sorgente per slam_toolbox
  (`use_scan_matching: false` — i ToF sono troppo pochi e corti)

---

## Step 5 — Configurazione slam_params.yaml

```yaml
use_scan_matching: false   # odometria pura, senza correzione ToF
```

---

## Step 6 — Configurazione nav2_params.yaml

```yaml
raytrace_max_range: 0.15   # 15cm — blocco fisico firmware
obstacle_max_range: 0.15
transform_tolerance: 2.0
laser_model_type: beam
max_beams: 5
```

---

## Step 7 — bridge_v1_7.py ✅ COMPLETATO

**File:** `AlvikDigitalTwin_v1_7/bridge_v1_7.py` (1082 righe)

Middleware unificato con commenti estesi. Sostituisce bridge_DDS_v1_5.py + bridge_websocket_v1_6.py.

**Architettura:**
```
Alvik UDP 50Hz CSV
  ↓
[Thread DDS] ──── asyncio.Queue ◄──── [asyncio WS handler]
  │                    (CMD)                │
  │  UDP DDS binario                        │  WebSocket JSON
  ▼                                         ▼
Godot :4444                              Godot :8765
ROS2  :4445
```

**Messaggi WebSocket gestiti:**
- `waypoints`, `stop`, `reset`, `cmd`, `pose`, `alignment`
- `explorer_start/stop` — SLAM + PID controller
- `slam_start/stop` — SLAM standalone (senza PID) ← NUOVO v1.7
- `nav2_start/stop` — navigazione autonoma AMCL + Nav2 ← NUOVO v1.7
- `nav2_goal` — pubblica goal su /goal_pose ← NUOVO v1.7

**Nuove funzioni v1.7:**
- `slam_start()` / `slam_stop()` — SLAM standalone
- `nav2_start()` / `nav2_stop()` — Nav2 con posa iniziale automatica
- `nav2_send_goal()` — pubblica goal con conversione quaternione
- Helper: `_ros2_exec()`, `_ros2_run()`, `_save_map()`, `_kill_processes()`, `_stop_process()`

**Pattern tecnici documentati:**
- `asyncio.Queue` — comunicazione thread-safe DDS ↔ WS
- `run_in_executor + functools.partial` — operazioni bloccanti non congelano asyncio
- `threading.Thread(daemon=True)` — thread DDS si chiude con il processo

---

## Step 8 — alvik_start_v1_7.sh ✅ COMPLETATO

**File:** `AlvikDigitalTwin_v1_7/alvik_start_v1_7.sh`

**Modalità:**
- `[ENTER]` → Applicazione principale (container + bridge + RViz2 + Godot)
- `[C]` → Calibrazione PID (container + bridge + PID + PlotJuggler)
- `[Ctrl+C]` → cleanup automatico via trap INT TERM

**Correzioni v1.7:**
- `ROS2_DIR="$PROJECT_DIR/ros2"` — path aggiornato alla copia v1.7
- `GODOT_PROJECT="$PROJECT_DIR/alvik_digital_twin_v1_7"` — directory v1.7
- Menu Godot aggiornato con tutte le modalità: S T P A E L N
- Parametri ottimali PID mostrati nel pannello calibrazione

---

## Step 9 — Script di test (da aggiornare)

### alvik_test_odom.sh
Test standalone odometria senza Godot/SLAM/Nav2.

### alvik_test_nav2.sh
Test Nav2 completo.

### alvik_teleop_keyboard.py
Controllo Alvik con frecce tastiera.

---

## Step 10 — alvik_launch.py

Modalità disponibili:
```bash
ros2 launch alvik_bridge alvik_launch.py                         # solo bridge
ros2 launch alvik_bridge alvik_launch.py slam:=true              # + slam_toolbox
ros2 launch alvik_bridge alvik_launch.py localization:=true      # + AMCL + map_server
```

Fix: `default_value='False'` (F maiuscola) per ROS2 Humble.

---

## Step 11 — Mappa salvata

**Directory:** `/ros2_ws/config/` (volume Docker persistente)  
**File:** `alvik_map.pgm` + `alvik_map.yaml`  
**Risoluzione:** 0.05 m/cella  
**Costruita con:** odometria pura, `use_scan_matching: false`

---

## Step 12 — selezione_waypoint_v1_7.gd ✅ COMPLETATO

**File:** `AlvikDigitalTwin_v1_7/alvik_digital_twin_v1_7/selezione_waypoint_v1_7.gd`

**Enum Mode completo:**
```gdscript
enum Mode {
    NONE, WAYPOINT, TELEOP, ALIGN, PATH,
    EXPLORER, EXPLORER_EXIT,    # v1.6
    SLAM, SLAM_EXIT, NAV2       # v1.7 NUOVO
}
```

**Tasti v1.7:**
- `L` → SLAM standalone (mappatura con teleop, senza PID)
- `N` → Nav2 navigazione autonoma (click su mappa = goal)

**Nuovi messaggi WebSocket inviati:**
- `slam_start` / `slam_stop` con `save_map`
- `nav2_start` / `nav2_stop`
- `nav2_goal` con x,y in metri (conversione Godot→ROS2: ros_x=-point.z, ros_y=-point.x)

**Nuovi messaggi WebSocket ricevuti:**
- `slam_started/stopped`, `nav2_started/stopped`, `nav2_goal_reached/failed`

**Commenti:** ogni variabile, funzione, scelta architetturale è documentata.

---

## Step 13 — Rinomina file Godot ✅ COMPLETATO

### Struttura finale alvik_digital_twin_v1_7/

**File attivi (root):**
```
Alvik.glb + .import                 ← modello 3D robot
Alvik_Image_0.png + .import         ← texture modello
Alvik_new.glb + .import             ← modello 3D aggiornato
Alvik_new_Image_0.png + .import     ← texture modello nuovo
alvik_robot.gdshader + .uid         ← shader robot
alvik_robot.tscn                    ← scena principale (uid://c3yvavrsvf5s5)
DDS_v1_7.gd + .uid                  ← autoload DDS ✅
digital_twin_v1_7.gd + .uid         ← script nodo robot 3D ✅
selezione_waypoint_v1_7.gd + .uid   ← script camera + modalità ✅
select_pointer.tscn                 ← pallino 3D waypoint
project.godot                       ← configurazione (autoload DDS → v1_7)
icon.svg + .import                  ← icona applicazione
export_presets.cfg                  ← configurazione export
```

**File archiviati in versioni_precedenti/:**
```
DDS_v01.gd + .uid
digital_twin_v01.gd + .uid
digital_twin_v02.gd + .uid
_selezione_waypoint.gd + .uid
_selezione_waypoint_v1_0.gd + .uid
selezione_waypoint_v1_5.gd + .uid
selezione_waypoint_v1_6_old.gd + .uid
robot.gd + .uid
robot.tscn
simRobot.gd + .uid
test_move0.gd + .uid
test_move1.gd + .uid
test_script.gd + .uid
Schermata da 2026-03-21 21-08-04.png + .import
```

### Operazioni eseguite
- ✅ `DDS_v1_5.gd` → `DDS_v1_7.gd`
- ✅ `digital_twin_v1_5.gd` → `digital_twin_v1_7.gd`
- ✅ `selezione_waypoint_v1_6.gd` → `selezione_waypoint_v1_7.gd`
- ✅ `project.godot` aggiornato: autoload DDS → `DDS_v1_7.gd`
- ✅ `alvik_robot.tscn` aggiornato: riferimenti script → v1_7
- ✅ File obsoleti spostati in `versioni_precedenti/`

---

## Parametri PID ottimali calibrati

```
kp_lin=1.5   ki_lin=0.8   kp_ang=25.0  ki_ang=0.0
acc_lin=3.0  dec_lin=2.0  vmax_lin=12.0 vmax_ang=60.0
angle_thresh=0.07  reach_thresh=1.0
```

---

## Test odometria eseguiti ✅

- **Traslazione 25cm**: x = -0.2505m → errore 0.2mm → ✅ preciso
- **Rotazione 90°**: coerente tra Alvik fisico e RViz2 → ✅ coerente
- **Nota**: `-x` in odometria corretto — effetto di theta+π offset

---

## Note tecniche importanti

- Container si chiude se si fa Ctrl+C sul nodo — usare terminale dedicato
- Mappa persiste in `/ros2_ws/config/` (volume Docker)
- AMCL richiede posa iniziale dopo "Managed nodes are active"
- `default_value='False'` con F maiuscola nei DeclareLaunchArgument (ROS2 Humble)
- `alvik_shell` in ~/.bashrc: `docker exec -it $(docker ps --filter "name=ros2-alvik_ros2" --format "{{.Names}}" | head -1) bash`
- VS Code warning ROS2 sono falsi positivi — pacchetti solo nel container

---

## TODO pendenti v1.7

### PROBLEMA CRITICO — Deviazione SLAM
slam_toolbox devia a destra durante movimento lineare.
- `use_scan_matching: false` nel sorgente ma il container carica ancora il file
  in `/ros2_ws/install/alvik_bridge/share/alvik_bridge/slam_params.yaml`
  che ha `use_scan_matching: true`
- Tentativo `cp` manuale e `colcon build --symlink-install` non hanno risolto
- Il container usato è `/home/neobot78/Scrivania/progetto_alvik/ros2/`
- Comando verifica: `cat /proc/$(pgrep -f slam_toolbox)/cmdline | tr '\0' ' '`
- **Soluzione da provare:** forzare il parametro a runtime con:
  `ros2 param set /slam_toolbox use_scan_matching false`
  oppure verificare se esiste un secondo `--params-file` in `/tmp/launch_params_*`
  che sovrascrive il primo

### Altri TODO
1. **Reset unificato** — alvik_ros_bridge deve azzerare x,y,theta su /alvik/reset
2. **Direzione Nav2** — robot va direzione opposta al goal (verificare theta+π)
3. **Testare Nav2** — tasto N non ancora verificato
4. **Archivio v1.7** — zip completo con relazione aggiornata

## Configurazioni file RViz
- `alvik.rviz`     → Fixed Frame=odom — odometria standalone
- `alvik_map.rviz` → Fixed Frame=map, topic /map Transient Local — SLAM
- `alvik_nav.rviz` → Fixed Frame=map, mappa caricata — Nav2

## Costanti bridge_v1_7.py (stato attuale)
```python
RVIZ_ODOM_CONFIG = "/ros2_ws/config/alvik.rviz"
RVIZ_SLAM_CONFIG = "/ros2_ws/config/alvik_map.rviz"
RVIZ_NAV_CONFIG  = "/ros2_ws/config/alvik_nav.rviz"
```

---

## Sessione 26 Aprile 2026 — Fix e miglioramenti

### Fix completati

#### 1. Label Bridge/Alvik separati ✅
**Problema:** Godot mostrava solo "CONNESSO" senza distinguere Bridge da Alvik.
**Causa:** `TickId` viene gestito separatamente in `DDS_v1_7.gd` e non va in `_local_vars` — quindi `DDS.read("TickId")` restituiva sempre `null`.
**Soluzione:**
- Aggiunta variabile `last_receive_msec` in `DDS_v1_7.gd` — aggiornata quando arriva un TickId valido
- In `digital_twin_v1_7.gd` la condizione diventa: `Time.get_ticks_msec() - DDS.last_receive_msec < 2000`
- Se Alvik si spegne, dopo 2 secondi appare "NON CONNESSO"
- `parse_bbcode()` → `text =` (parse_bbcode deprecato in Godot 4)

#### 2. Allineamento Godot non interferisce con ROS2 ✅
**Problema:** La rotazione fatta in modalità Allineamento veniva pubblicata su `/alvik/alignment` in ROS2, causando la freccia del Digital Twin sfasata di 90°.
**Causa:** L'allineamento serve solo per la visualizzazione Godot, non per ROS2 che ha il suo sistema di riferimento.
**Soluzione:** Rimosso `publish_alignment()` dal gestore `alignment` nel bridge — l'offset viene memorizzato solo internamente per il calcolo waypoint.

#### 3. Script sbagliato in Godot ✅
**Problema:** KEY_N non funzionava, print di debug non apparivano — niente funzionava nelle modifiche a `selezione_waypoint_v1_7.gd`.
**Causa:** `alvik_robot.tscn` puntava a `versioni_precedenti/selezione_waypoint_v1_6_old.gd` invece di `selezione_waypoint_v1_7.gd`.
**Soluzione:** `sed -i` sul .tscn per correggere il riferimento.
**Lezione:** Quando qualcosa non risponde in Godot, prima cosa da verificare è quale script è assegnato al nodo nel .tscn.

#### 4. use_scan_matching: false permanente ✅
**Problema:** Il container Docker ricreava `/ros2_ws/install/` ad ogni avvio con `use_scan_matching: true`.
**Causa:** `/ros2_ws/install/` non era montato come volume — i file venivano ricreati dall'immagine Docker.
**Soluzione:** `docker cp` del file aggiornato nel container dopo l'avvio. Aggiunto cleanup preventivo in `nav2_start()`.
**Nota tecnica:** Aggiungere il volume install al docker-compose ha causato problemi (permessi root) — la soluzione docker cp è più robusta.

#### 5. RViz si chiude all'uscita da SLAM/NAV2 ✅
**Problema:** RViz rimaneva aperto dopo ESC da SLAM o NAV2.
**Soluzione:** `_stop_rviz()` aggiornato per fare pkill di `rviz2` dentro il container oltre a terminare il Popen del gnome-terminal.

#### 6. PID Controller si chiude all'uscita da Esploratore ✅
**Problema:** `alvik_pid_controller` rimaneva attivo dopo ESC dall'Esploratore.
**Soluzione:** Aggiunto `pkill -f alvik_pid_controller` in `explorer_stop()`.

#### 7. RViz config corretta per ogni modalità ✅
**Configurazione finale:**
```python
RVIZ_ODOM_CONFIG = "/ros2_ws/config/alvik.rviz"      # odometria standalone
RVIZ_SLAM_CONFIG = "/ros2_ws/config/alvik_map.rviz"  # SLAM/Esploratore
RVIZ_NAV_CONFIG  = "/ros2_ws/config/alvik_nav.rviz"  # Nav2
```
- Esploratore e SLAM → `alvik_map.rviz` (Fixed Frame=map, topic /map Transient Local)
- Nav2 → `alvik_nav.rviz` (Fixed Frame=map, mappa caricata)

#### 8. Robot si ferma all'uscita da Nav2 ✅
**Problema:** Robot continuava a muoversi dopo ESC da Nav2.
**Soluzione:** In `nav2_stop()` i nodi Nav2 vengono fermati PRIMA di `send_stop_alvik()` — così Nav2 non manda più comandi su /cmd_vel quando si invia lo stop ad Alvik.

### Problemi noti / TODO

#### Nav2 — caricamento lento e goal fallisce
**Causa:** Nav2 avvia una seconda istanza dei nodi mentre la prima è ancora attiva — conflitti lifecycle.
**Errore:** `BtActionNode::Tick: invalid status value`
**Fix applicato:** Cleanup preventivo dei nodi residui all'inizio di `nav2_start()`
**Nota:** La mappa deve essere creata esplorando con SLAM+teleop prima di usare Nav2 — una mappa vuota non permette la pianificazione del percorso.

#### Coordinate goal Nav2 in scala
**Problema:** I goal in metri sono nell'ordine di 0.1-0.8m — troppo piccoli.
**Causa:** La mappa è stata creata senza muovere Alvik — l'area mappata è minima.
**Soluzione:** Creare una mappa esplorando almeno 1-2 metri in ogni direzione con SLAM.

### File modificati in questa sessione
- `DDS_v1_7.gd` — aggiunta `last_receive_msec`, fix TickId
- `digital_twin_v1_7.gd` — fix label Bridge/Alvik, `text =` invece di `parse_bbcode()`
- `selezione_waypoint_v1_7.gd` — fix KEY_N, fix return NAV2, max_lines=5
- `alvik_robot.tscn` — corretto riferimento script da v1_6_old a v1_7
- `bridge_v1_7.py` — rimosso publish_alignment, fix _stop_rviz, fix nav2_stop, cleanup nav2_start

---

## Sviluppi futuri

### Modalità Esploratore / SLAM
- **Scelta nome file mappa** — al momento del salvataggio (dialogo S/N) permettere all'utente di inserire un nome personalizzato per il file della mappa invece di sovrascrivere sempre `alvik_map`. Utile per avere più mappe di ambienti diversi.

### Modalità Nav2
- **Selezione mappa** — prima di avviare Nav2, mostrare un menu con le mappe disponibili in `/ros2_ws/config/` e permettere di scegliere quale caricare. Attualmente carica sempre `alvik_map.yaml`.

---

## Sessione 27 Aprile 2026 — Fix e problemi aperti

### Fix applicati

#### slam_params.yaml — max_laser_range: 0.15 ✅
```yaml
max_laser_range: 0.15   # era 2.0
```
Aggiornato sia nel sorgente che nel container con docker cp.

#### nav2_params.yaml — laser_max_range: 0.15 ✅
```yaml
laser_max_range: 0.15   # era 2.0
```

#### alvik_ros_bridge.py — scan.range_max: 0.15 ✅
```python
scan.range_max = 0.15   # era 2.0
```

#### alvik_ros_bridge.py — timestamp sincronizzato ✅
```python
now = self._get_synced_stamp(t_robot_ms)   # attivo
# now = self.get_clock().now().to_msg()     # commentato
```

#### Nav2 — posa iniziale dalla posizione corrente ✅
Nav2 ora pubblica la posa corrente di Alvik invece di sempre (0,0,π).

#### MAP_SAVE_PATH unificato con MAP_NAV_PATH ✅
```python
MAP_SAVE_PATH = "/ros2_ws/config/alvik_map"   # era /ros2_ws/maps/
MAP_NAV_PATH  = "/ros2_ws/config/alvik_map.yaml"
```
SLAM salva direttamente in /config/ dove Nav2 la cerca.

---

### TODO PRIORITARI prossima sessione

1. **Map SLAM non funzionante** — dopo pkill alvik_ros_bridge il topic /map
   diventa rosso. Probabilmente legato al cambio timestamp (_get_synced_stamp
   potrebbe causare problemi con slam_toolbox). Da investigare se tornare a
   get_clock().now() risolve.

2. **Raggi ToF non solidali con Alvik** — i raggi ToF ogni tanto "saltano"
   all'origine invece di seguire il robot. Causa probabile: disallineamento
   temporale TF. Il cambio a _get_synced_stamp dovrebbe aiutare ma da verificare.

3. **Punti neri a 0.15m** — con range_max=0.15 i punti devono apparire solo
   entro 15cm. Da verificare che il fix sia effettivo dopo riavvio completo.

4. **Nome fisso container Docker** — docker-compose run genera nome casuale
   ad ogni avvio (ros2-alvik_ros2-run-XXXXXXXX). Soluzione: usare
   docker-compose up invece di docker-compose run in alvik_start_v1_7.sh.
   Il docker-compose.yml ha già container_name: alvik_ros2 — basta usarlo.

### Procedura docker cp dopo ogni riavvio
Finché non si risolve il nome fisso del container, dopo ogni riavvio:
```bash
CONTAINER=$(docker ps --filter "name=ros2-alvik_ros2" --format "{{.Names}}" | head -1)
docker cp ~/Scrivania/progetto_alvik/ros2/src/alvik_bridge/alvik_bridge/alvik_ros_bridge.py \
  $CONTAINER:/ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/alvik_ros_bridge.py
docker cp ~/Scrivania/progetto_alvik/ros2/src/alvik_bridge/alvik_bridge/slam_params.yaml \
  $CONTAINER:/ros2_ws/install/alvik_bridge/share/alvik_bridge/slam_params.yaml
```

---

## Sessione 29 Aprile 2026 — Fix e miglioramenti

### Fix completati

#### Griglia + freccia posa iniziale (tasto G) ✅
- **G** toggle griglia + freccia posa iniziale
- Griglia 25cm×25cm (linee leggere) + 1m×1m (linee marcate) estensione ±20m
- Freccia rossa piatta sul pavimento — indica posizione e orientamento al reset
- La freccia si aggiorna dopo conferma allineamento (ESC o A in modalità ALIGN)
- La freccia si aggiorna anche ad ogni Reset (R)
- Dimensione freccia: ~22cm totale

#### Floor ampliato ✅
- PlaneMesh e CollisionShape portati da 2m×2m a 10m×10m

### TODO PRIORITARI prossima sessione

1. **Sincronismo Teleop Godot ↔ Alvik fisico** — durante la navigazione
   in modalità Teleop con Godot, dopo un po' si perde la corrispondenza
   tra Alvik fisico e il Digital Twin. Problema critico soprattutto
   per la modalità SLAM dove la mappa deve corrispondere alla realtà.
   Causa probabile: latenza WiFi variabile + nessun feedback di posizione
   in tempo reale durante il teleop.
   Soluzione da investigare: aggiornare il Digital Twin sempre dalla
   telemetria DDS (posizione reale) anche in modalità Teleop, invece
   di muoverlo in base ai comandi inviati.

2. **Nome fisso container Docker** — da risolvere

3. **Map SLAM** — verificare stabilità dopo le ultime modifiche

4. **Log su file** — da implementare

---

## TODO — Prossima sessione

### Perfezionamento Path Fissi
Il robot deve ritornare nella posizione e posa iniziale al termine
di qualsiasi percorso predefinito (quadrato, cerchio, triangolo).

Implementazione:
- Salvare la posa iniziale (x, y, theta) al momento dell'invio del path
- Aggiungere un waypoint finale con la posa iniziale alla lista dei punti
- Il bridge riceverà il waypoint finale e Alvik tornerà all'origine del percorso

---

## Sessione 30 Aprile 2026 — Fix e miglioramenti

### Fix completati

#### Path fissi — ritorno con theta iniziale ✅
- `_send_path()` legge `DDS.read("Theta")` al momento dell'invio
- Ultimo waypoint ha `{"x": 0.0, "y": 0.0, "theta": theta_iniziale}`
- Bridge `path_follow_loop()` ruota Alvik fino al theta iniziale (tolleranza 3°)
- Bug trovato: bridge scartava theta dal parsing waypoints — fix con `**{"theta": float(p["theta"])} if "theta" in p else {}`

#### Etichette coordinate sui marker waypoint ✅
- `Label3D` giallo sopra ogni marker con numero e coordinate `(x, y) cm`
- Font size 12, billboard, traslata lateralmente `(-0.08, 0.05, -0.08)`
- `_clear_markers()` rimuove anche le etichette
- Marker rimangono durante la navigazione — cancellati al completamento o ESC

#### Viste laterali 4 e 5 ✅
- **4** = laterale destra, **5** = laterale sinistra
- Guard `current_mode != Mode.PATH` per evitare conflitto con selezione path
- Fix array out of bounds `view_name` in `_update_info_label()`

#### Menu Godot aggiornato ✅
- Due colonne in NAVIGAZIONE: 1/2/3 e 4/5/Rotellina

#### Menu avvio shell aggiornato ✅
- ESP32+STM32, [*] RViz e PID, sezione NOTE, printf dimensione terminale

#### Calibrazione BASE_WIDTH_M ✅
- Testata ma non efficace — l'errore di rotazione è nel firmware STM32
- Il firmware calcola correttamente la rotazione relativa ma ~7-10° di errore su 360°
- BASE_WIDTH_M ripristinato a 0.090 — non influisce sul Teleop puro
- PDF guida calibrazione generato: `alvik_calibrazione.pdf`

#### max_laser_range: 0.15 → 0.30 per avvio SLAM ✅
- Con 0.15 la mappa non appariva finché Alvik non si muoveva (tutti .inf)
- Soluzione temporanea: 0.30 nel container — sorgente rimane a 0.15

### TODO prossima sessione

1. **Nav2 goal** — corrispondenza coordinate Godot↔ROS2
2. **Nome fisso container Docker**
3. **Log su file**
4. **Documentazione finale**

---

## Sessione 03-04 Maggio 2026 — Fix Nav2 e SLAM

### Fix completati

#### Nav2 — goal inviato con ros2 action send_goal ✅
**Problema:** il goal veniva pubblicato su `/goal_pose` tramite `ros2 topic pub` con timestamp zero — bt_navigator lo ignorava silenziosamente.
**Causa:** `ros2 topic pub --once` non imposta il timestamp correttamente e Nav2 scarta messaggi con stamp=0.
**Soluzione:** sostituito con `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose` — l'action client usa il clock interno di ROS2 e gestisce correttamente handshake e feedback.
**File modificato:** `bridge_v1_7.py` → funzione `nav2_send_goal()`
```python
# PRIMA (non funzionava):
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped ...

# DOPO (funziona):
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...
```

#### Nav2 — singolo launch invece di due separati ✅
**Problema:** il bridge avviava due launch file separati (localizzazione + navigation) causando conflitto del `behavior_server` avviato due volte.
**Soluzione:** sostituito con un singolo `bringup_launch.py` che avvia tutto in una volta.

#### SLAM — max_laser_range portato a 2.0 ✅
**Problema:** con 0.15 i ToF leggevano oltre il range e la mappa non si costruiva.
**Soluzione:** `max_laser_range: 2.0` coerente con `scan_buffer_maximum_scan_distance` e `scan.range_max`.

#### Nav2 — theta normalizzato per posa iniziale ✅
**Problema:** theta accumulato (-13.68 rad) causava posa iniziale sbagliata (-603°).
**Fix da applicare:** normalizzare theta tra -π e +π prima di calcolare il quaternione.

### TODO prossima sessione

1. **Coordinata mouse su Godot** — label vicino al cursore con coordinate del punto sotto il mouse (tasto da definire, es. H). Utile per selezionare punti precisi e verificare la corrispondenza Godot↔Nav2.
2. **Test corrispondenza coordinate** — verificare che un punto cliccato in Godot corrisponda alla posizione corretta in Nav2/RViz.
3. **Nome fisso container Docker** — usare `docker-compose up` con `container_name: alvik_ros2`.
4. **Log su file** — salvare i log del bridge su file con timestamp.
5. **Documentazione finale** — dev_log, relazione, archivio zip v1.7.

---

## Sessione 05 Maggio 2026 — Fix Nav2 coordinate e stabilità TF

### Fix completati

#### Coordinate Nav2 Godot→ROS2 ✅
**Formula definitiva** (verificata sperimentalmente):
```gdscript
var ros_x = -point.x   # X Godot → -X ROS2
var ros_y =  point.z   # Z Godot → +Y ROS2
```
**Corrispondenza sistemi di riferimento:**
```
Alvik fisico:  X=avanti, Y=sinistra, Theta=0 punta avanti
Godot:         X=destra, Z=avanti(profondità), Y=alto
ROS2 (odom):   X=-alvik_x (negato), Y=-alvik_y (negato), theta=alvik_theta+π

Movimento 25cm avanti in Alvik fisico:
  Alvik:  X +25cm
  Godot:  label X +0.26m (= point.z nel raycast)
  ROS2:   x -0.257m (negato)

Quindi per inviare goal Nav2 dal punto cliccato in Godot:
  ros_x = -point.x   (X Godot negato → X ROS2)
  ros_y =  point.z   (Z Godot → Y ROS2)
```
**Nota:** `point` in Godot è già in metri — NON dividere per CM_TO_M.
**Nota:** L'allineamento Godot NON va applicato — ROS2 ha frame map indipendente.

#### Theta goal Nav2 ✅
Il theta inviato con il goal Nav2 include +π per compensare l'offset in alvik_ros_bridge:
```gdscript
var theta_nav = (float(theta_raw) + PI) if theta_raw != null else PI
# normalizzato tra -π e +π
```

#### TF pubblicato a 100Hz continuo ✅
**Problema:** gap di 200ms nel TF durante latenza WiFi causava salti di base_footprint.
**Soluzione:** `alvik_ros_bridge.py` pubblica il TF a 100Hz anche senza nuovi dati WiFi:
```python
now = self.get_clock().now().to_msg()
if self.x is not None and self.y is not None and self.theta is not None:
    self._publish_tf(now)  # sempre a 100Hz
# Solo quando arriva nuovo TickId aggiorna odom e scan
```

#### transform_tolerance aggiornato ✅
```yaml
# nav2_params.yaml
transform_tolerance: 0.5   # era 0.1 nei costmap
```

#### Immagine Docker ricostruita ✅
Tutti i fix sono permanenti nell'immagine:
- max_laser_range: 2.0
- transform_tolerance: 0.5
- TF continuo a 100Hz
- BASE_WIDTH_M: 0.090

### TODO prossima sessione
1. **Label sinistra Godot** — valori ripetuti, aggiornare a 1Hz invece che ogni frame
2. **Nav2 rotazione finale** — Alvik ruota molto durante la navigazione per allinearsi al theta finale
3. **Nome fisso container Docker**
4. **Log su file bridge**
5. **Documentazione finale**

---

## Sessione 05 Maggio 2026 (pomeriggio) — Fix label e log

### Fix completati

#### Label sinistra Godot — aggiornamento a 1Hz ✅
Posa e ToF aggiornati ogni 1 secondo invece che ogni frame (~60fps).
Aggiunta variabile `_log_pose_timer` in `digital_twin_v1_7.gd`.

#### Timestamp con centesimi di secondo ✅
```gdscript
var cs = (Time.get_ticks_msec() % 1000) / 10
message_history.push_front("[color=gray][%s.%02d][/color] %s" % [t, cs, new_text])
```

#### Nav2 rotazione finale ✅
Funziona correttamente dopo la correzione delle coordinate Godot→ROS2.

### TODO prossima sessione

1. **Nome fisso container Docker** — usare `docker-compose up` con `container_name: alvik_ros2`

2. **Log su file bridge con timestamp** — implementare rotazione log:
   - Cartella `/log` nella directory del progetto
   - Size massimo 100MB per la cartella
   - Cancellare i log più vecchi quando si supera il limite
   - Formato: `bridge_YYYYMMDD_HHMMSS.log`

3. **Test generale completo** — test di tutte le modalità

4. **Documentazione finale** — dev_log, relazione, archivio zip v1.7

---

## Sessione 06 Maggio 2026 — Nome fisso container Docker

### Fix completati

#### Nome fisso container Docker ✅
**Problema:** `docker-compose run --rm` genera nomi casuali tipo `ros2-alvik_ros2-run-XXXXXXXX`
che cambiavano ad ogni avvio, richiedendo di ricercare il nome ogni volta.

**Soluzione:** sostituito con `docker-compose up -d` che usa il `container_name: alvik_ros2`
definito nel `docker-compose.yml`. Il container si chiama sempre `alvik_ros2`.

**File modificati:**
- `alvik_start_v1_7.sh` — usa `docker-compose down` + `docker-compose up -d` + `docker exec -it alvik_ros2`
- `alvik_stop_v1_7.sh` — usa `docker exec alvik_ros2 pkill -f alvik_ros_bridge` + `docker-compose down`
- `bridge_v1_7.py` — aggiunta costante `CONTAINER_NAME = "alvik_ros2"`, `get_container_name()` restituisce sempre `CONTAINER_NAME`

**Nota:** con `docker-compose up -d` il processo `alvik_ros_bridge` gira dentro il container
come daemon — non è più un processo figlio dell'host. Quindi `pkill` dall'host non funziona:
bisogna usare `docker exec alvik_ros2 bash -c "pkill -f alvik_ros_bridge"`.

### TODO prossima sessione
1. **Log su file bridge con rotazione** — cartella `/log`, max 100MB, cancella log più vecchi
2. **Test generale completo**
3. **Documentazione finale**

---

## Sessione 07-09 Maggio 2026 — Log su file e documentazione

### Fix completati

#### Log su file con rotazione automatica ✅
**Implementazione in `bridge_v1_7.py`:**
```python
class _TeeOutput:
    """Scrive su terminale E su file simultaneamente."""
    def write(self, msg):
        self._terminal.write(msg)
        self._file.write(msg)

# Formato: bridge_YYYYMMDD_HHMMSS.log
# Rotazione: cancella i più vecchi se >100MB
LOG_DIR    = "./log"
LOG_MAX_MB = 100
```
Alias aggiunti in `~/.bashrc`:
```bash
alias alvik_shell='docker exec -it alvik_ros2 bash'
alias alvik_log='tail -f ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/log/$(ls -t ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/log/ | head -1)'
```

#### Coordinata mouse in Godot (tasto H) ✅
Label `CoordLabel` che mostra le coordinate del punto sotto il cursore in
tutte le modalità. Attivata/disattivata con tasto H.
```
Godot: (0.50, 0.00)
Alvik: (50.3, -0.0) cm
```

---

## Sessione 16-19 Maggio 2026 — Test generale e documentazione

### Fix completati

#### Test generale completo ✅
Tutte le modalità operative testate:
- Teleop (T) — controllo manuale ✅
- Waypoint (S) — selezione punti con click ✅
- Path fissi (P) — quadrato, cerchio, triangolo con ritorno posa ✅
- Allineamento (A) — rotazione Digital Twin ✅
- Esploratore (E) — SLAM + PID controller ✅
- SLAM standalone (L) — costruzione mappa manuale ✅
- Nav2 (N) — navigazione autonoma ✅
- Calibrazione PID (C) — PlotJuggler + test comandi ✅

#### Problema modalità Esploratore — mappa non visibile in RViz2 ✅
**Problema:** la mappa SLAM non appariva in RViz2 anche se slam_toolbox era attivo.
**Causa:** in RViz2 la mappa veniva aggiunta con "Add by Display" invece di "Add by Topic".
**Soluzione:** usare "Add by Topic" → `/map` → configurazione salvata in `alvik_map.rviz`.
La configurazione viene richiamata automaticamente ad ogni avvio RViz2.

#### Documentazione completa ✅
Prodotti i seguenti documenti:
- `ARCHITETTURA_TECNICA.md` — architettura, evoluzione, problemi risolti
- `MANUALE_UTENTE.md` — guida operativa completa
- `GUIDA_DOCKER.md` — Docker con motivazione Ubuntu 20.04
- `GUIDA_ROS2.md` — nodi, topic, TF, comandi
- `GUIDA_SLAM_NAV2.md` — teoria, configurazione, problemi risolti
- `GUIDA_CALIBRAZIONE_PID.md` — architettura PID, PlotJuggler, parametri
- `RELAZIONE_FINALE.md` — descrizione progetto, Godot, script
- `EVOLUZIONE_ARCHITETTURALE.md` — tre fasi con diagrammi draw.io
- `RoadMap.md` — roadmap aggiornata v1.7

#### Commenti a tutti gli script ✅
- `DDS_v1_7.gd` — docstring a classi e funzioni, commenti inline
- `digital_twin_v1_7.gd` — docstring, pulizia codice debug
- `selezione_waypoint_v1_7.gd` — docstring a tutte le 20 funzioni
- `bridge_v1_7.py` — docstring a tutte le funzioni
- `alvik_ros_bridge.py` — docstring, spiegazione trasformazioni
- `alvik_pid_controller.py` — docstring a classi PID, State, controller
- `alvik_launch.py` — docstring con argomenti e motivazione singolo launch

#### Correzione documentazione — Problema 3 architettura v1.6 ✅
**Correzione:** il Problema 3 non era "sincronizzazione manuale tramite IPC/file"
ma "assenza di sincronizzazione diretta" — i due bridge erano completamente
indipendenti e la posizione di Alvik arrivava al bridge_websocket solo
tramite un giro indiretto via Godot (doppia latenza):
```
Alvik → bridge_DDS → UDP :4444 → Godot
                                    ↓ WebSocket :8765
                              bridge_websocket ← {"type": "pose", x, y, theta}
```
`current_pose` inizializzato a `{0,0,0}` — usato dal path following anche
senza dati reali se Godot non aveva ancora inviato la posa.
Corretti: `ARCHITETTURA_TECNICA.md` e il diagramma `architettura_problematiche.drawio`.

---

## Stato finale v1.7 — Maggio 2026

### Parametri di sistema definitivi

```python
# bridge_v1_7.py
ALVIK_IP          = "192.168.4.1"
ALVIK_PORT        = 5005
DDS_GODOT_PORT    = 4444
DDS_ROS_PORT      = 4445
WS_PORT           = 8765
CONTAINER_NAME    = "alvik_ros2"
WAYPOINT_REACH_CM = 3.0
ANGLE_THRESH_RAD  = 0.15
MAX_VLIN          = 10.0
MAX_VANG          = 60.0
LOG_DIR           = "./log"
LOG_MAX_MB        = 100

# alvik_ros_bridge.py
WHEEL_RADIUS_M = 0.017
BASE_WIDTH_M   = 0.090
TF_RATE_HZ     = 100

# slam_params.yaml
use_scan_matching: false
max_laser_range: 2.0
resolution: 0.05

# nav2_params.yaml
transform_tolerance: 0.5   # nei costmap
laser_max_range: 0.15

# alvik_pid_controller.py
kp_lin=1.5  ki_lin=0.8  kp_ang=25.0  ki_ang=0.0
acc=3.0 cm/s²  dec=2.0 cm/s²  vmax_lin=12.0 cm/s  vmax_ang=60.0 deg/s
```

### Sviluppi futuri identificati
- Scelta nome mappa al salvataggio SLAM
- Selezione mappa da caricare in Nav2
- Rimozione automatica ostacoli non più rilevati
- Sensor fusion IMU per correzione errore odometrico (~7-10°/giro)
- Miglioramento firmware STM32 per compensazione gioco meccanico
