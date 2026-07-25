# Architettura Tecnica — Alvik Digital Twin v1.7

**Versione:** 1.7  
**Data:** Maggio 2026  
**Destinatari:** Commissione universitaria

---

## Indice

1. [Introduzione e obiettivi](#1-introduzione-e-obiettivi)
2. [Evoluzione architetturale](#2-evoluzione-architetturale)
3. [Architettura generale v1.7](#3-architettura-generale-v17)
3. [Hardware — Arduino Alvik](#3-hardware--arduino-alvik)
4. [Protocollo di comunicazione DDS](#4-protocollo-di-comunicazione-dds)
5. [Middleware — bridge_v1_7.py](#5-middleware--bridge_v17py)
6. [Digital Twin — Godot Engine 4](#6-digital-twin--godot-engine-4)
7. [Stack ROS2](#7-stack-ros2)
8. [SLAM — Costruzione mappa](#8-slam--costruzione-mappa)
9. [Navigazione autonoma Nav2](#9-navigazione-autonoma-nav2)
10. [Controller PID](#10-controller-pid)
11. [Infrastruttura Docker](#11-infrastruttura-docker)
12. [Sistemi di riferimento e trasformazioni](#12-sistemi-di-riferimento-e-trasformazioni)
13. [Scelte progettuali e problemi risolti](#13-scelte-progettuali-e-problemi-risolti)
14. [Parametri di sistema](#14-parametri-di-sistema)

---

## 1. Introduzione e obiettivi

### Contesto

Il progetto nasce dall'esigenza di creare un sistema integrato per il controllo e la supervisione del robot Arduino Alvik che vada oltre il semplice controllo remoto, aggiungendo:

- Visualizzazione 3D in tempo reale della posizione e dello stato del robot
- Capacità di navigazione autonoma con costruzione di mappe
- Interfaccia intuitiva per la selezione di percorsi
- Strumenti per la calibrazione e l'ottimizzazione del comportamento

### Obiettivi tecnici

1. **Comunicazione real-time** a 50Hz tra robot fisico e Digital Twin
2. **Integrazione ROS2** per sfruttare l'ecosistema di algoritmi robotici
3. **SLAM** per la costruzione autonoma di mappe dell'ambiente
4. **Navigazione autonoma** con pianificazione del percorso
5. **Architettura modulare** facilmente estendibile

### Evoluzione dalla versione precedente

La v1.7 unifica i due bridge separati (DDS e WebSocket) delle versioni precedenti in un unico middleware Python (`bridge_v1_7.py`), aggiunge le modalità SLAM standalone e Nav2, e introduce il controller PID per il path following.

---

<div style="page-break-before: always;"></div>

## 2. Evoluzione architetturale

### 2.1 Fase 1 — Struttura iniziale

La struttura iniziale del sistema prevedeva due bridge Python separati e indipendenti:

- **bridge_DDS_v1_5.py** — gestiva la telemetria UDP a 50Hz da Alvik e la distribuzione dei dati via DDS binario verso Godot e ROS2
- **bridge_websocket_v1_6.py** — gestiva la comunicazione bidirezionale WebSocket con Godot per l'invio dei comandi

---

*[Diagramma: architettura_v16_bridge_separati.drawio]*

*Fase 1 — Due bridge Python separati con sincronizzazione manuale*

---

**Punti di forza iniziali:**
- Separazione delle responsabilità (Single Responsibility Principle)
- Indipendenza dei due moduli

**Limitazioni non anticipate:**
- Necessità di sincronizzazione manuale tra i due bridge (stato condiviso)
- Doppia connessione UDP verso Alvik (comandi da due sorgenti)
- Complessità di avvio e gestione dei processi

---

### 2.2 Fase 2 — Problematiche emerse

Durante lo sviluppo e i test sono emersi tre problemi strutturali.

---

*[Diagramma: architettura_problematiche.drawio]*

*Fase 2 — Problematiche emerse: conflitto alvik_ros_bridge, canale DDS:4445, sincronizzazione*

---

**Problema 1 — alvik_ros_bridge: nodo standalone vs nodo nel container**

`alvik_ros_bridge` era stato concepito come nodo ROS2 standalone sull'host. Per motivi di isolamento dell'ambiente ROS2 si è scelto di eseguirlo dentro Docker, creando un'ambiguità architetturale:

```
Host:
  bridge_DDS → UDP :4445 → localhost

Docker (container):
  alvik_ros_bridge ← porta :4445 mappata dall'host
  (ma il container ha il suo namespace di rete)
```

**Problema 2 — Ciclo bidirezionale canale UDP DDS :4445**

I comandi Nav2 (`/cmd_vel`) dovevano tornare dal container verso il bridge DDS per essere inviati ad Alvik, creando un ciclo difficile da gestire con due processi separati:

```
bridge_DDS  →  UDP :4445  →  container (alvik_ros_bridge)
                                      ↓ /cmd_vel Nav2
bridge_DDS  ←  UDP :5006  ←  container
```

**Problema 3 — Assenza di sincronizzazione diretta tra i due bridge**

I due bridge erano completamente indipendenti — non condividevano stato né comunicavano direttamente. `bridge_websocket_v1_6.py` aveva il proprio `current_pose` inizializzato a zero:

```python
current_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}  # inizializzato a zero
```

La posizione di Alvik arrivava al bridge WebSocket solo attraverso un giro indiretto via Godot:

```
Alvik → bridge_DDS → UDP :4444 → Godot
                                    ↓ (Godot legge la posizione dal DDS)
                                    ↓ WebSocket :8765
                              bridge_websocket ← {"type": "pose", x, y, theta}
```

Questo significava che:
- Il path following iniziava con `current_pose = {0,0,0}` se Godot non aveva ancora inviato la posa
- La latenza della posizione era doppia: Alvik → DDS → Godot → WebSocket → bridge_WS
- Se Godot non era aperto o non inviava il messaggio `pose`, il path following usava coordinate errate
- I comandi CMD venivano inviati da entrambi i bridge sullo stesso socket UDP verso Alvik — con possibili conflitti

---

### 2.3 Fase 3 — Struttura finale v1.7

La soluzione è stata l'**unificazione dei due bridge** in un unico processo Python `bridge_v1_7.py`.

---

*[Diagramma: architettura_v17_bridge_unificato.drawio]*

*Fase 3 — Bridge unificato con Thread DDS + asyncio e comunicazione thread-safe via asyncio.Queue*

---

```
bridge_v1_7.py (unico processo)
├── Thread DDS (RT 50Hz)
│   ├── Ricezione telemetria UDP :5005 ← Alvik
│   ├── Broadcast DDS binario :4444 → Godot
│   ├── Broadcast DDS binario :4445 → alvik_ros_bridge
│   ├── Invio CMD :5005 → Alvik
│   └── Lettura asyncio.Queue (non bloccante)
│
├── asyncio.Queue (comunicazione thread-safe)
│
└── asyncio event loop
    ├── WebSocket server :8765 ↔ Godot
    ├── path_follow_loop (20Hz)
    └── subprocess → Docker ROS2 (SLAM, Nav2, PID)
```

| Aspetto | Bridge separati (v1.6) | Bridge unificato (v1.7) |
|---|---|---|
| Stato condiviso | File/IPC — race condition | Variabili Python — thread-safe con Queue |
| Comandi Nav2 | Ciclo bidirezionale complesso | Loop unico nel thread DDS |
| Avvio/arresto | Due processi da gestire | Un solo processo |
| Debugging | Due log separati | Un unico log |

---

### 2.4 Recap versioni e problemi risolti

#### v1.0 — Prototipo iniziale
- Digital Twin 3D base con Godot Engine 4, comunicazione UDP, visualizzazione posizione

#### v1.1–v1.4 — Sviluppo incrementale
- Visualizzazione raggi ToF, Teleop, Waypoint, Path following, Path fissi

#### v1.5 — Bridge DDS
- `bridge_DDS_v1_5.py` — thread RT 50Hz, protocollo DDS binario custom, prima integrazione ROS2

#### v1.6 — Bridge WebSocket + SLAM
- `bridge_websocket_v1_6.py` — asyncio per comandi Godot
- Modalità Esploratore (SLAM + PID), allineamento Digital Twin

#### v1.7 — Bridge unificato + Nav2 completo

| # | Problema | Soluzione |
|---|---|---|
| 1 | Due bridge separati con sync manuale | Unificazione in `bridge_v1_7.py` con asyncio.Queue |
| 2 | `alvik_ros_bridge` standalone vs container | Eseguito sempre dentro Docker |
| 3 | Container con nome casuale ad ogni avvio | `docker-compose up -d` con `container_name: alvik_ros2` |
| 4 | `use_scan_matching` non persistente | docker-compose build ricostruzione immagine |
| 5 | TF gap 200ms durante latenza WiFi | TF pubblicato a 100Hz continuo |
| 6 | Goal Nav2 ignorato (timestamp=0) | `ros2 action send_goal` invece di `topic pub` |
| 7 | Doppio `behavior_server` Nav2 | Singolo `bringup_launch.py` |
| 8 | Theta accumulato (-603°) posa iniziale | Normalizzazione theta tra -π e +π |
| 9 | Coordinate Godot→ROS2 errate | Verificate sperimentalmente: `ros_x=-point.x, ros_y=point.z` |
| 10 | Label Godot aggiornata a 60fps | Timer 1Hz per posa e ToF |
| 11 | Label Bridge/Alvik indistinta | Label separati con `last_receive_msec` |
| 12 | Allineamento pubblicato su ROS2 | Rimosso `publish_alignment()` |
| 13 | Script Godot obsoleto in .tscn | Corretto riferimento da v1_6_old a v1_7 |
| 14 | Path fissi senza ritorno posa iniziale | Ultimo waypoint con campo `theta` |
| 15 | Marker waypoint senza coordinate | `Label3D` con numero e coordinate cm |
| 16 | KEY_4/5 conflitto in modalità PATH | Guard `current_mode != Mode.PATH` |
| 17 | `transform_tolerance` troppo bassa | Portata da 0.1 a 0.5 nei costmap Nav2 |
| 18 | `max_laser_range` troppo basso | Portato da 0.15 a 2.0 per SLAM efficace |
| 19 | Log bridge non persistente | Sistema log su file con rotazione (max 100MB) |
| 20 | Nessuna coordinata mouse in Godot | Tasto H toggle label coordinate |
## 3. Architettura generale

### Schema a blocchi

```
┌─────────────────────────────────────────────────────────────────┐
│                    ARDUINO ALVIK                                  │
│  ESP32 (WiFi) + STM32 (motori, encoder, ToF)                    │
│  UDP CSV 50Hz → 192.168.4.1:5005                                │
└──────────────────────┬──────────────────────────────────────────┘
                       │ UDP telemetria CSV
                       ▼
┌─────────────────────────────────────────────────────────────────┐
│                   bridge_v1_7.py                                 │
│  Thread DDS (50Hz) + asyncio WebSocket + asyncio path following  │
│                                                                   │
│  Uscite:                                                          │
│  • UDP :4444 → Godot (DDS binario)                              │
│  • UDP :4445 → alvik_ros_bridge (DDS binario)                   │
│  • WS  :8765 ↔ Godot (JSON comandi/notifiche)                  │
│  • UDP :5005 → Alvik (comandi CMD,vlin,vang)                    │
└───────┬──────────────────────────────────┬───────────────────────┘
        │ UDP DDS :4444                    │ WebSocket :8765
        ▼                                  ▼
┌───────────────┐                ┌──────────────────────────────────┐
│  Godot DT     │                │  Godot DT                        │
│  Visualiz. 3D │                │  Controllo modalità              │
│  (DDS_v1_7.gd)│                │  (selezione_waypoint_v1_7.gd)   │
└───────────────┘                └──────────────────────────────────┘

        │ UDP DDS :4445
        ▼
┌─────────────────────────────────────────────────────────────────┐
│              Docker Container "alvik_ros2"                       │
│              Ubuntu 20.04 + ROS2 Humble                         │
│                                                                   │
│  alvik_ros_bridge ──→ /odom, /tf (100Hz), /scan                 │
│                                                                   │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │  slam_toolbox     → /map                                │    │
│  │  nav2_bringup     → AMCL + controller + planner        │    │
│  │  alvik_pid_controller ← /pid/command                   │    │
│  └─────────────────────────────────────────────────────────┘    │
│                          │ /cmd_vel                              │
└──────────────────────────┼──────────────────────────────────────┘
                           │ UDP :5006
                           ▼
                    bridge_v1_7.py → Alvik
```

### Flusso dati principale

```
Alvik fisico
  │  50Hz UDP CSV: x,y,theta,wl,wr,tof_L,tof_CL,tof_C,tof_CR,tof_R,tickId,t_ms
  ▼
bridge_v1_7.py [Thread DDS]
  │  Decodifica CSV, aggiorna variabili condivise
  │  Pubblica DDS binario a 50Hz
  ├──→ Godot :4444  (visualizzazione Digital Twin)
  └──→ ROS2  :4445  (alvik_ros_bridge per SLAM/Nav2)
```

---

## 4. Hardware — Arduino Alvik

### Specifiche fisiche

| Parametro | Valore |
|---|---|
| Raggio ruota | 17mm |
| Distanza interasse | 90mm |
| Sensori ToF | 5 (L, CL, C, CR, R) |
| Range ToF | 0-200cm (limite pratico ~150cm) |
| Blocco firmware ToF | 15cm (arresto automatico) |
| Comunicazione | UDP WiFi 50Hz |
| Indirizzo IP | 192.168.4.1:5005 |

### Firmware

Il firmware Arduino Alvik è composto da due MCU:
- **ESP32** — gestione WiFi, comunicazione UDP, parsing comandi
- **STM32** — controllo motori, lettura encoder e sensori ToF, calcolo odometria

Il firmware calcola l'odometria internamente con un loop a ~1kHz, integrando i tick degli encoder. Il valore di theta è cumulativo (non normalizzato) e può superare i ±360°.

### Limitazione nota: errore odometrico

Durante i test è stato riscontrato un errore sistematico nella rotazione di circa 7-10° per giro completo (360°). L'errore è **non cumulativo** (reversibile): tornando nella posizione iniziale con rotazione inversa, il theta torna a 0.

La causa è attribuibile principalmente al gioco meccanico dell'albero motore piuttosto che allo slittamento delle ruote sul pavimento.

Un possibile miglioramento sarebbe intervenire sul firmware di Alvik per affinare la compensazione del gioco meccanico, oppure sfruttare la sensor fusion con l'IMU integrata nell'STM32 — che in teoria già contribuisce al calcolo dell'odometria — per correggere la deriva angolare in modo più accurato.

---

## 5. Protocollo di comunicazione DDS

### Formato telemetria UDP (Alvik → bridge)

Ogni pacchetto è una stringa CSV a 12 campi, inviata a 50Hz (ogni 20ms):

```
x_cm, y_cm, theta_rad, wl_rpm, wr_rpm, tof_L, tof_CL, tof_C, tof_CR, tof_R, tickId, t_robot_ms
```

Esempio:
```
25.5, -0.2, 0.12, 45.3, 44.8, 122, 75, 76, 75, 60, 1250, 24680
```

### Formato DDS binario (bridge → Godot/ROS2)

Il bridge converte i dati CSV in un protocollo binario custom (DDS-like) per la pubblicazione locale:

```python
# Struttura pacchetto DDS
# Header: 4 byte magic + 2 byte num_vars
# Per ogni variabile: 1 byte len_name + N byte name + 4 byte float32
```

Le variabili pubblicate sono: `X`, `Y`, `Theta`, `WheelLeft`, `WheelRight`, `ToF_L`, `ToF_CL`, `ToF_C`, `ToF_CR`, `ToF_R`, `TickId`, `T_robot_ms`.

### Formato comandi (bridge → Alvik)

```
CMD,<vlin_cm_s>,<vang_deg_s>   # comando velocità
RESET                           # reset odometria
```

---

## 6. Middleware — bridge_v1_7.py

### Pattern architetturali

Il bridge utilizza due pattern fondamentali per gestire la concorrenza:

**1. Thread DDS + asyncio event loop**

```python
# Thread DDS — sincrono, real-time 50Hz
def dds_thread():
    while True:
        data = receive_udp()           # bloccante, bassa latenza
        parse_csv(data)
        publish_dds_binary()           # a Godot :4444 e ROS2 :4445
        cmd = cmd_queue.get_nowait()   # non bloccante
        if cmd: send_to_alvik(cmd)

# asyncio event loop — per WebSocket e path following
async def main():
    await asyncio.gather(
        websocket_server(),    # gestisce comandi da Godot
        path_follow_loop(),    # path following a 20Hz
    )
```

**2. asyncio.Queue per comunicazione thread-safe**

```python
cmd_queue = asyncio.Queue()

# Il WS handler mette i comandi nella coda (asyncio)
async def ws_handler(msg):
    await cmd_queue.put({"type": "cmd", "vlin": 10.0, "vang": 0.0})

# Il thread DDS legge dalla coda (thread)
def dds_thread():
    try:
        cmd = cmd_queue.get_nowait()
    except asyncio.QueueEmpty:
        pass
```

**3. run_in_executor per operazioni bloccanti**

Le funzioni che lanciano processi Docker (SLAM, Nav2) sono bloccanti. Vengono eseguite in un thread pool per non congelare l'event loop asyncio:

```python
loop = asyncio.get_event_loop()
await loop.run_in_executor(None, partial(nav2_start))
```

### Path following

Il bridge implementa un controller proporzionale per il path following dei waypoint Godot:

```python
def compute_command(pose, target):
    dx = target["x"] - pose["x"]
    dy = target["y"] - pose["y"]
    dist = math.sqrt(dx**2 + dy**2)
    
    target_angle = math.atan2(dy, dx)
    angle_err = normalize_angle(target_angle - pose["theta"])
    
    if abs(angle_err) > ANGLE_THRESH_RAD:
        # Ruota prima di avanzare
        vang = clamp(angle_err * KP_ANGLE, -MAX_VANG, MAX_VANG)
        return 0.0, vang, dist
    else:
        vlin = min(dist * KP_LIN, MAX_VLIN)
        vang = angle_err * KP_ANGLE
        return vlin, vang, dist
```

### Gestione theta finale nei path fissi

L'ultimo waypoint dei path fissi include il campo `theta` per il ritorno alla posa iniziale:

```python
# Godot invia
{"x": 0.0, "y": 0.0, "theta": theta_iniziale}

# Il bridge gestisce
if is_last and theta_target is not None:
    angle_err = normalize_angle(theta_target - current_pose["theta"])
    if abs(angle_err) > 0.05:  # 3° tolleranza
        vang = clamp(angle_err * KP_ANGLE, -MAX_VANG, MAX_VANG)
        continue  # non avanza al waypoint successivo
```

### Log su file

Il bridge redirige stdout su un file di log con rotazione automatica:

```python
class _TeeOutput:
    """Scrive su terminale E su file simultaneamente."""
    def write(self, msg):
        self._terminal.write(msg)
        self._file.write(msg)

# Rotazione: se la cartella /log supera 100MB, elimina i log più vecchi
def _cleanup_old_logs():
    files = sorted(glob.glob(LOG_DIR + "/bridge_*.log"))
    total = sum(os.path.getsize(f) for f in files)
    while total > LOG_MAX_BYTES and files:
        os.remove(files.pop(0))
```

---

## 7. Digital Twin — Godot Engine 4

### Struttura script

Il progetto Godot è composto da tre script principali:

| Script | Ruolo |
|---|---|
| `DDS_v1_7.gd` | Autoload — ricezione DDS UDP, variabili condivise |
| `digital_twin_v1_7.gd` | Aggiornamento posizione/rotazione robot, visualizzazione ToF |
| `selezione_waypoint_v1_7.gd` | Gestione modalità operative, WebSocket, UI |

### DDS_v1_7.gd — Autoload

Riceve i pacchetti DDS binari dal bridge a 50Hz e li decodifica in variabili GDScript accessibili globalmente:

```gdscript
# Lettura variabili ovunque nel progetto
var x = DDS.read("X")        # posizione X in cm
var theta = DDS.read("Theta") # orientamento in radianti
```

### Sincronizzazione con TickId

Il Digital Twin aggiorna la posizione solo quando riceve un nuovo TickId, evitando aggiornamenti duplicati durante i picchi di latenza WiFi:

```gdscript
var tick = DDS.read("TickId")
if tick == null or tick == _last_tick:
    return
_last_tick = tick
# aggiorna posizione...
```

### Sistema di allineamento

L'allineamento permette di ruotare il Digital Twin per far corrispondere l'orientamento visivo con quello fisico di Alvik. La rotazione viene salvata come `align_rot_y` e applicata alle coordinate dei waypoint:

```gdscript
# Applica allineamento alle coordinate waypoint
var rot = deg_to_rad(align_rot_y)
var ax = gz * cos(rot) + gx * sin(rot)
var ay = -gz * sin(rot) + gx * cos(rot)
```

> **Importante:** L'allineamento Godot non viene comunicato a ROS2. I due sistemi hanno frame di riferimento indipendenti.

### Raggi ToF

I 5 sensori ToF vengono visualizzati come raggi 3D che partono dal robot:

```gdscript
# 5 sensori: L=-60°, CL=-30°, C=0°, CR=+30°, R=+60°
var angles = [-60, -30, 0, 30, 60]
for i in range(5):
    var angle_rad = deg_to_rad(angles[i])
    var direction = Vector3(sin(angle_rad), 0, -cos(angle_rad))
    ray.look_at(global_position + direction * tof_values[i])
```

---

## 8. Stack ROS2

### Nodi attivi

```
/alvik_ros_bridge          ← riceve DDS :4445, pubblica /odom /tf /scan
/slam_toolbox              ← sottoscrive /scan /tf, pubblica /map
/amcl                      ← localizzazione su mappa salvata
/bt_navigator              ← Behavior Tree per navigazione
/controller_server         ← RegulatedPurePursuitController
/planner_server            ← NavfnPlanner (Dijkstra)
/alvik_pid_controller      ← path following locale
```

### Topics principali

| Topic | Tipo | Descrizione |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Odometria da encoder |
| `/tf` | `tf2_msgs/TFMessage` | Trasformazioni (100Hz) |
| `/scan` | `sensor_msgs/LaserScan` | Laser scan da 5 ToF |
| `/map` | `nav_msgs/OccupancyGrid` | Mappa SLAM |
| `/cmd_vel` | `geometry_msgs/Twist` | Comandi velocità Nav2 |
| `/pid/command` | `std_msgs/String` | Comandi PID controller |
| `/pid_debug` | `std_msgs/Float32MultiArray` | Dati debug PID |

### alvik_ros_bridge.py

Converte la telemetria DDS in messaggi ROS2 standard:

```python
# Odometria
odom.pose.pose.position.x = -self.x   # negato per sistema di riferimento
odom.pose.pose.position.y = -self.y
odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

# TF continuo a 100Hz
def update(self):
    now = self.get_clock().now().to_msg()
    if self.x is not None and self.y is not None and self.theta is not None:
        self._publish_tf(now)   # sempre, anche senza nuovi dati
    # aggiorna odom e scan solo con nuovo TickId
    if tick_id == self.last_tick_id:
        return
```

Il TF viene pubblicato continuamente a 100Hz (indipendentemente dalla frequenza dei dati WiFi) per evitare gap che causerebbero errori di trasformazione in Nav2.

### Laser scan da ToF

I 5 sensori ToF vengono convertiti in un laser scan ROS2 con 5 raggi:

```python
scan = LaserScan()
scan.angle_min = -math.radians(60)
scan.angle_max =  math.radians(60)
scan.angle_increment = math.radians(30)
scan.range_max = 2.0
scan.ranges = [tof_L, tof_CL, tof_C, tof_CR, tof_R]  # in metri
```

---

## 9. SLAM — Costruzione mappa

### Configurazione slam_toolbox

```yaml
solver_plugin: solver_plugins::CeresSolver
use_scan_matching: false     # odometria pura — i 5 ToF non bastano
max_laser_range: 2.0         # range massimo scan considerato
minimum_travel_distance: 0.05  # aggiorna mappa ogni 5cm
minimum_travel_heading: 0.1    # o ogni ~6°
resolution: 0.05               # 5cm per cella
map_update_interval: 5.0       # pubblica /map ogni 5s
```

### Scelta di use_scan_matching: false

La decisione di disabilitare lo scan matching è motivata dalle caratteristiche dei sensori:
- Solo 5 raggi ToF (contro i 360 di un LiDAR tipico)
- Range fisico fino a ~150cm ma letture instabili in ambienti aperti
- Distribuzione angolare limitata a ±60°

Con scan matching abilitato, slam_toolbox tentava di correggere l'odometria basandosi su 5 soli punti con letture rumorose, peggiorando la qualità della mappa. Con odometria pura la mappa risulta più coerente. La qualità della mappa migliora esplorando più volte gli stessi percorsi, in modo che le letture ToF si stabilizzino per accumulo.

### Salvataggio mappa

```bash
# Da bridge (via WebSocket da Godot)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  '{name: {data: "/ros2_ws/config/alvik_map"}}'

# La mappa viene salvata come:
# /ros2_ws/config/alvik_map.pgm  (immagine occupancy grid)
# /ros2_ws/config/alvik_map.yaml (metadati: risoluzione, origine)
```

---

## 10. Navigazione autonoma Nav2

### Architettura Nav2

La navigazione autonoma usa il stack Nav2 completo con un singolo `bringup_launch.py`:

```
nav2_bringup
    ├── map_server          ← carica alvik_map.yaml
    ├── amcl                ← localizzazione particellare
    ├── controller_server   ← RegulatedPurePursuitController
    ├── planner_server      ← NavfnPlanner (Dijkstra globale)
    ├── bt_navigator        ← Behavior Tree
    └── lifecycle_manager   ← gestione ciclo di vita nodi
```

### Posa iniziale AMCL

All'avvio di Nav2, il bridge pubblica automaticamente la posa iniziale di Alvik su `/initialpose`:

```python
# Normalizza theta tra -π e +π
raw_theta = current_pose["theta"]
while raw_theta >  math.pi: raw_theta -= 2 * math.pi
while raw_theta < -math.pi: raw_theta += 2 * math.pi
theta = raw_theta + math.pi   # offset +π per sistema di riferimento

qz = math.sin(theta / 2.0)
qw = math.cos(theta / 2.0)
```

### Invio goal

I goal vengono inviati tramite l'action ROS2 (non tramite topic pub):

```bash
# PRIMA (non funzionava — timestamp zero ignorato da bt_navigator):
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped ...

# DOPO (funziona — action gestisce handshake e timestamp):
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...
```

### Conversione coordinate Godot → ROS2

La formula definitiva per la conversione è stata determinata sperimentalmente:

```gdscript
# point è già in metri in Godot (unità Godot = 1m)
var ros_x = -point.x   # X Godot → -X ROS2
var ros_y =  point.z   # Z Godot → +Y ROS2

# Theta con offset +π per compensare alvik_ros_bridge
var theta_nav = (float(theta_raw) + PI) if theta_raw != null else PI
# normalizzato tra -π e +π
```

**Corrispondenza verificata sperimentalmente:**

| Movimento | Alvik | Godot (label) | ROS2 |
|---|---|---|---|
| 25cm avanti | X +25cm | X +0.26m | x -0.257m |
| Sistema | X=avanti, Y=sinistra | X=destra, Z=profondità | X=-alvik_x, Y=-alvik_y |

---

## 11. Controller PID

### Architettura a cascata

Il sistema di controllo è a due anelli:

```
Anello esterno (ROS2, ~20Hz):
  Errore posizione → PID → velocità desiderata

Anello interno (STM32, ~1kHz):
  Errore velocità ruota → PID → PWM motori
```

### Implementazione

```python
class AlvikPIDController(Node):
    def update(self):
        # Calcola errore posizione
        dx = self.target["x"] - self.x
        dy = self.target["y"] - self.y
        dist = math.sqrt(dx**2 + dy**2)
        
        # Errore angolare
        target_angle = math.atan2(dy, dx)
        angle_err = normalize(target_angle - self.theta)
        
        # Controllo in cascata
        if abs(angle_err) > ANGLE_THRESH:
            vang = pid_angular.update(angle_err)
            vlin = 0.0
        else:
            vlin = pid_linear.update(dist) * math.cos(angle_err)
            vang = pid_angular.update(angle_err)
        
        # Profilo trapezioidale (rampa acc/dec)
        vlin = apply_trapezoid_profile(vlin, self.vlin_current)
        
        # Pubblica su /cmd_vel
        self.cmd_vel_pub.publish(Twist(linear=vlin, angular=vang))
```

### Parametri ottimali

| Parametro | Valore | Descrizione |
|---|---|---|
| `kp_lin` | 1.5 | Guadagno proporzionale lineare |
| `ki_lin` | 0.8 | Guadagno integrale lineare |
| `kp_ang` | 25.0 | Guadagno proporzionale angolare |
| `ki_ang` | 0.0 | Guadagno integrale angolare |
| `acc` | 3.0 cm/s² | Accelerazione lineare |
| `dec` | 2.0 cm/s² | Decelerazione lineare |
| `vmax_lin` | 12.0 cm/s | Velocità lineare massima |
| `vmax_ang` | 60.0 deg/s | Velocità angolare massima |
| `angle_thresh` | 0.07 rad | Soglia angolare (≈4°) |
| `reach_thresh` | 1.0 cm | Soglia waypoint raggiunto |

---

## 12. Infrastruttura Docker

### Container

Il container `alvik_ros2` (nome fisso) usa l'immagine `ros:humble-ros-base` con i pacchetti ROS2 necessari:

```dockerfile
FROM ros:humble-ros-base
RUN apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-plotjuggler-ros
```

### Persistenza dati

Le mappe vengono salvate nel volume Docker `/ros2_ws/config/` che persiste tra riavvii del container:

```yaml
# docker-compose.yml
volumes:
  - ./ros2_ws/config:/ros2_ws/config
```

### Nome fisso container

Dalla v1.7 il container usa `docker-compose up -d` invece di `docker-compose run`, garantendo sempre il nome `alvik_ros2`:

```yaml
# docker-compose.yml
services:
  alvik_ros2:
    container_name: alvik_ros2
```

Questo elimina la necessità di ricercare dinamicamente il nome del container ad ogni avvio.

---

## 13. Sistemi di riferimento e trasformazioni

### Sistemi di riferimento

```
Alvik fisico:
  X = avanti (direzione marcia)
  Y = sinistra
  Z = su
  Theta = 0 quando punta avanti, cresce antiorario

Godot Engine:
  X = destra
  Y = su (altezza)
  Z = profondità (avanti = -Z)
  Il robot punta verso -Z quando theta=0

ROS2 (frame odom/map):
  X = -alvik_x (negato)
  Y = -alvik_y (negato)
  Theta = alvik_theta + π
```

### Trasformazione TF

Il TF tree di ROS2 è:

```
map → odom → base_footprint → base_link → laser_frame
                                        → left_wheel
                                        → right_wheel
                                        → tof_link
```

### Conversioni implementate

**Alvik fisico → ROS2:**
```python
ros2_x = -alvik_x / 100.0   # cm → m, negato
ros2_y = -alvik_y / 100.0
ros2_theta = alvik_theta + math.pi
```

**Godot → Alvik fisico (senza allineamento):**
```gdscript
alvik_x_cm = point.z / CM_TO_M   # Z Godot = X Alvik
alvik_y_cm = point.x / CM_TO_M   # X Godot = Y Alvik
```

**Godot → ROS2 (goal Nav2, senza allineamento):**
```gdscript
ros_x = -point.x   # già in metri
ros_y =  point.z
```

---

## 14. Scelte progettuali e problemi risolti

### 1. Unificazione middleware

**Problema:** v1.6 usava due bridge separati (DDS e WebSocket) che richiedevano sincronizzazione manuale.

**Soluzione:** `bridge_v1_7.py` unifica tutto in un unico processo con Thread DDS + asyncio event loop, comunicando via `asyncio.Queue` thread-safe.

### 3. TF continuo a 100Hz

**Problema:** gap di 200ms nel TF durante picchi di latenza WiFi causavano errori di trasformazione in Nav2 (`Timed out waiting for transform`).

**Soluzione:** `alvik_ros_bridge` pubblica il TF a 100Hz continuo usando l'ultima posizione nota, anche senza nuovi dati UDP.

### 4. Goal Nav2 via action invece di topic

**Problema:** `ros2 topic pub --once /goal_pose` inviava messaggi con timestamp=0 che bt_navigator ignorava silenziosamente.

**Soluzione:** `ros2 action send_goal /navigate_to_pose` usa il clock interno di ROS2 e gestisce handshake e feedback correttamente.

### 5. Singolo launch Nav2

**Problema:** avviare `localization_launch.py` + `navigation_launch.py` separatamente causava il conflitto del `behavior_server` avviato due volte.

**Soluzione:** un singolo `bringup_launch.py` avvia tutto in una volta, eliminando il conflitto.

### 6. Nome fisso container Docker

**Problema:** `docker-compose run --rm` generava nomi casuali `ros2-alvik_ros2-run-XXXXXXXX` che cambiavano ad ogni avvio.

**Soluzione:** `docker-compose up -d` con `container_name: alvik_ros2` in `docker-compose.yml`.

### 7. Theta cumulativo per posa iniziale Nav2

**Problema:** dopo lunghe sessioni il theta di Alvik era accumulato (-13.68 rad = -603°), causando una posa iniziale errata per AMCL.

**Soluzione:** normalizzazione del theta tra -π e +π prima di calcolare il quaternione per la posa iniziale.

### 8. Coordinate Godot → ROS2

**Problema:** la conversione delle coordinate tra Godot e ROS2 richiedeva la comprensione di tre sistemi di riferimento diversi con negazioni e offset.

**Soluzione:** determinata sperimentalmente misurando la posizione dopo un movimento di 25cm avanti:
- Alvik X +25cm → Godot label X +0.26m → ROS2 x -0.257m
- Formula: `ros_x = -point.x`, `ros_y = point.z`

---

## 15. Parametri di sistema

### bridge_v1_7.py

```python
# Rete
ALVIK_IP          = "192.168.4.1"
ALVIK_PORT        = 5005
DDS_GODOT_PORT    = 4444
DDS_ROS_PORT      = 4445
WS_PORT           = 8765

# Path following
WAYPOINT_REACH_CM = 3.0     # cm — waypoint raggiunto
ANGLE_THRESH_RAD  = 0.15    # rad — soglia angolare (~8.6°)
MAX_VLIN          = 10.0    # cm/s
MAX_VANG          = 60.0    # deg/s
KP_ANGLE          = 80.0    # guadagno angolare proporzionale
CMD_HZ            = 20      # Hz comandi path following

# Log
LOG_DIR           = "./log"
LOG_MAX_MB        = 100
```

### alvik_ros_bridge.py

```python
WHEEL_RADIUS_M = 0.017
BASE_WIDTH_M   = 0.090
TF_RATE_HZ     = 100    # frequenza TF continuo
```

### slam_params.yaml

```yaml
use_scan_matching: false
max_laser_range: 2.0
resolution: 0.05
minimum_travel_distance: 0.05
minimum_travel_heading: 0.1
map_update_interval: 5.0
tf_buffer_duration: 30.0
transform_timeout: 0.2
```

### nav2_params.yaml

```yaml
# Global
transform_tolerance: 2.0

# Costmap
laser_max_range: 0.15
raytrace_max_range: 0.15
obstacle_max_range: 0.15
transform_tolerance: 0.5   # nei costmap

# Controller
controller_frequency: 10.0
min_vel_x: 0.0
max_vel_x: 0.12   # m/s
```

---

*Fine Architettura Tecnica — Alvik Digital Twin v1.7*
