<div align="center" style="margin-top: 200px;">

# Alvik Digital Twin v1.7

### SLAM & NAV2 — BREVE GUIDA

<br/>

### Arduino Alvik · Godot Engine 4 · ROS2 Humble · Docker

<br/><br/>

---

<br/>

**Progetto Sistemi Robotici — A.A. 2025/2026**

<br/><br/>

**Antonino Zappalà**  
Matricola: 1000081971

<br/><br/>

*Maggio 2026*

</div>

<div style="page-break-after: always;"></div>

<div style="page-break-after: always;"></div>

# Guida SLAM e Nav2 — Alvik Digital Twin v1.7

**Versione:** 1.7  
**Data:** Maggio 2026

---

## Indice

1. [Introduzione a SLAM e Nav2](#1-introduzione-a-slam-e-nav2)
2. [SLAM nel progetto](#2-slam-nel-progetto)
3. [Nav2 nel progetto](#3-nav2-nel-progetto)
4. [Problemi riscontrati e soluzioni](#4-problemi-riscontrati-e-soluzioni)
5. [Workflow operativo](#5-workflow-operativo)

---

## 1. Introduzione a SLAM e Nav2

### SLAM — Simultaneous Localization and Mapping

SLAM è un algoritmo che permette a un robot di costruire una mappa dell'ambiente mentre stima simultaneamente la propria posizione all'interno di quella mappa. Risolve il problema del "chi è nato prima tra l'uovo e la gallina": per costruire una mappa accurata serve conoscere la posizione, ma per conoscere la posizione serve una mappa.

Nel progetto viene usato **slam_toolbox** in modalità *mapping* — il robot esplora l'ambiente e costruisce progressivamente una mappa occupancy grid (griglia di occupazione) dove ogni cella può essere libera, occupata o sconosciuta.

### Nav2 — Navigation 2

Nav2 è lo stack di navigazione autonoma di ROS2. Data una mappa precostruita e una posa iniziale, Nav2 è in grado di:
- **Localizzarsi** sulla mappa tramite AMCL (Adaptive Monte Carlo Localization)
- **Pianificare** un percorso globale verso il goal (NavfnPlanner)
- **Seguire** il percorso evitando ostacoli dinamici (RegulatedPurePursuitController)
- **Gestire** situazioni di stallo tramite behavior tree (spin, backup, wait)

---

## 2. SLAM nel progetto

### Nodi coinvolti

```
alvik_ros_bridge → /scan (5 raggi ToF) → slam_toolbox → /map
alvik_ros_bridge → /odom, /tf          → slam_toolbox
```
<div style="page-break-after: always;"></div>

### Modalità disponibili

| Modalità Godot | Descrizione |
|---|---|
| `E` — Esploratore | SLAM + PID controller (esplorazione autonoma guidata) |
| `L` — SLAM standalone | SLAM con controllo manuale Teleop |

### Configurazione slam_params.yaml

```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    mode: mapping
    use_scan_matching: false     # scelta progettuale — vedi sotto
    max_laser_range: 2.0
    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.1
    resolution: 0.05
    map_update_interval: 5.0
    tf_buffer_duration: 30.0
    transform_timeout: 0.2
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 2.0
```

### Scelta progettuale: use_scan_matching: false

**Motivazione:** slam_toolbox funziona in due modalità:

- **Con scan matching** — usa i raggi ToF per correggere la stima di posizione confrontando il scan corrente con la mappa costruita
- **Senza scan matching** (odometria pura) — usa solo l'odometria degli encoder per stimare la posizione; i ToF servono solo per marcare le celle occupate

Con i 5 sensori ToF di Alvik (±60°, range ~150cm, letture instabili) lo scan matching **peggiorava** la qualità della mappa causando correzioni errate. Con odometria pura la mappa risulta più coerente e stabile.

**Come funziona lo SLAM nel progetto:**

```
1. Alvik si muove di ≥5cm o ruota di ≥0.1 rad
2. slam_toolbox legge la nuova posizione da /odom (encoder)
3. I 5 raggi ToF da /scan marcano le celle occupate nella griglia
4. La mappa viene aggiornata e pubblicata su /map ogni 5s
```

La qualità della mappa migliora esplorando più volte gli stessi percorsi.
<br>

### Laser scan da 5 ToF

I 5 sensori ToF vengono convertiti in un laser scan ROS2 con 5 raggi:

```python
scan.angle_min       = -math.radians(60)   # -60°
scan.angle_max       =  math.radians(60)   # +60°
scan.angle_increment =  math.radians(30)   # passo 30°
scan.range_max       = 2.0                 # 2m
scan.ranges          = [tof_L, tof_CL, tof_C, tof_CR, tof_R]  # metri
```

Distribuzione angolare: L=-60°, CL=-30°, C=0°, CR=+30°, R=+60°.

<br>

### Salvataggio mappa

Dalla modalità Esploratore o SLAM, in Godot, premere `ESC` per uscire dalla modaità ed in seguito `S` salvare:

```bash
# Il bridge esegue internamente:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  '{name: {data: "/ros2_ws/config/alvik_map"}}'
```

Vengono salvati:
- `/ros2_ws/config/alvik_map.pgm` — immagine della mappa (occupancy grid)
- `/ros2_ws/config/alvik_map.yaml` — metadati (risoluzione, origine, soglie)

Esempio `alvik_map.yaml`:
```yaml
resolution: 0.05
origin: [-1.76, -1.2, 0]
free_thresh: 0.25
occupied_thresh: 0.65
```

---

<div style="page-break-after: always;"></div>

## 3. Nav2 nel progetto

### Nodi coinvolti

```
map_server          ← carica alvik_map.yaml
amcl                ← localizzazione particellare su mappa
controller_server   ← RegulatedPurePursuitController
planner_server      ← NavfnPlanner (Dijkstra globale)
bt_navigator        ← Behavior Tree NavigateToPose
behavior_server     ← spin, backup, wait, drive_on_heading
smoother_server     ← SimpleSmoother
waypoint_follower   ← WaitAtWaypoint
velocity_smoother   ← VelocitySmoother
lifecycle_manager   ← gestione ciclo di vita (×2)
```
<br>

### Avvio con singolo launch

**Scelta progettuale:** dalla v1.7 Nav2 viene avviato con un **singolo** `bringup_launch.py` invece di due launch separati (localizzazione + navigation):

```python
# PRIMA (causava conflitto behavior_server):
ros2 launch alvik_bridge alvik_launch.py localization:=true
ros2 launch nav2_bringup navigation_launch.py ...

# DOPO (singolo launch, nessun conflitto):
ros2 launch nav2_bringup bringup_launch.py \
  map:=/ros2_ws/config/alvik_map.yaml \
  params_file:=/ros2_ws/install/alvik_bridge/share/alvik_bridge/nav2_params.yaml
```
<br>

### Posa iniziale AMCL

All'avvio di Nav2, il bridge pubblica automaticamente la posa corrente di Alvik su `/initialpose`. Il theta viene normalizzato tra -π e +π per evitare problemi con valori accumulati:

```python
raw_theta = current_pose["theta"]
while raw_theta >  math.pi: raw_theta -= 2 * math.pi
while raw_theta < -math.pi: raw_theta += 2 * math.pi
theta = raw_theta + math.pi   # offset +π per sistema di riferimento ROS2
```
<div style="page-break-after: always;"></div>

### Invio goal da Godot

Il goal viene inviato tramite `ros2 action send_goal` (non `topic pub`):

```bash
# Il bridge esegue internamente:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: ros_x, y: ros_y, z: 0.0}, orientation: {...}}}}'
```
<br>

### Conversione coordinate Godot → ROS2

Formula verificata sperimentalmente (25cm avanti in Alvik = X+0.26 in Godot = x-0.257 in ROS2):

```gdscript
# point è già in metri in Godot
var ros_x = -point.x   # X Godot → -X ROS2
var ros_y =  point.z   # Z Godot → +Y ROS2

# Theta con offset +π
var theta_nav = (float(theta_raw) + PI) if theta_raw != null else PI
while theta_nav >  PI: theta_nav -= 2 * PI
while theta_nav < -PI: theta_nav += 2 * PI
```
<br>

### Configurazione nav2_params.yaml (scelte progettuali)

```yaml
# transform_tolerance: portata da 0.1 a 0.5
# Motivazione: gap fino a 200ms nel TF durante latenza WiFi
# causavano errori "Timed out waiting for transform"
transform_tolerance: 0.5   # nei costmap

# laser_max_range: 0.15m
# Corrisponde al range di blocco automatico firmware Alvik
# I ToF si fermano a 15cm per sicurezza
laser_max_range: 0.15
raytrace_max_range: 0.15
obstacle_max_range: 0.15

# Controller PurePursuit
max_vel_x: 0.12        # m/s — velocità massima navigazione
min_vel_x: 0.0
lookahead_distance: 0.4
```

---
<div style="page-break-after: always;"></div>

## 4. Problemi riscontrati e soluzioni

### 4.1 Mappa non si costruisce all'avvio

**Sintomo:** aprendo RViz2 la mappa è vuota anche dopo alcuni secondi.

**Causa:** con `max_laser_range: 0.15` (troppo basso), tutti i raggi ToF che leggono >15cm vengono scartati da slam_toolbox → nessun dato utile.

**Soluzione:** portare `max_laser_range: 2.0` in `slam_params.yaml`. La mappa inizia a costruirsi non appena Alvik si muove di 5cm.

---

### 4.2 TF gap — errori di trasformazione

**Sintomo:**
```
Timed out waiting for transform from base_link to map to become available
```

**Causa:** il TF veniva pubblicato solo alla ricezione di un nuovo TickId da Alvik. Con WiFi instabile, gap fino a 200ms causavano l'assenza del TF.

**Soluzione:** TF pubblicato a 100Hz continuo in `alvik_ros_bridge.py`:
```python
def update(self):
    now = self.get_clock().now().to_msg()
    if self.x is not None:
        self._publish_tf(now)   # sempre, anche senza nuovi dati
```

Inoltre `transform_tolerance` nei costmap portata da 0.1 a 0.5s.

---
<br>

### 4.3 Goal Nav2 ignorato silenziosamente

**Sintomo:** il goal viene inviato ma Alvik non si muove. Nessun errore nei log.

**Causa:** `ros2 topic pub --once /goal_pose` inviava messaggi con `stamp.sec=0`. Il `bt_navigator` ignorava i messaggi con timestamp zero.

**Soluzione:** usare `ros2 action send_goal /navigate_to_pose` che gestisce automaticamente il timestamp dal clock ROS2.

---
<div style="page-break-after: always;"></div>

### 4.4 Conflitto behavior_server

**Sintomo:**
```
[behavior_server]: Unable to start transition 1 from current state active
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

**Causa:** avviare `localization_launch.py` (che include `bringup_launch.py`) seguito da `navigation_launch.py` causava il `behavior_server` avviato due volte.

**Soluzione:** singolo `bringup_launch.py` che avvia tutto in una volta.

---
<br>

### 4.5 Theta accumulato — posa iniziale errata

**Sintomo:** dopo sessioni con molte rotazioni, AMCL posizionava il robot in modo errato sulla mappa. Il log mostrava angoli come -603°.

**Causa:** il firmware Alvik accumula il theta senza normalizzarlo. Dopo molte rotazioni il valore era `-13.68 rad = -603°`.

**Soluzione:** normalizzazione del theta prima di calcolare il quaternione per `/initialpose`.

---
<br>

### 4.6 Coordinate goal sbagliate da Godot

**Sintomo:** cliccando un punto in Godot, Alvik si muoveva in direzione errata o perpendicolare.

**Causa:** i tre sistemi di riferimento (Alvik fisico, Godot, ROS2) hanno assi e segni diversi. La formula iniziale era errata.

**Soluzione:** formula determinata sperimentalmente misurando la posizione dopo un movimento noto di 25cm:

```
Alvik: +25cm in X → Godot: label X=+0.26m → ROS2: odom x=-0.257m
Formula: ros_x = -point.x, ros_y = point.z
```

---
<div style="page-break-after: always;"></div>

## 5. Workflow operativo

### Costruire una mappa

1. Avviare l'applicazione: `./alvik_start_v1_7.sh`
2. Premere `A` in Godot per allineare il Digital Twin
3. Premere `E` (Esploratore) o `L` (SLAM standalone)
4. Muovere Alvik con Teleop (`T`) esplorando l'ambiente
5. Verificare la mappa in RViz2 — esplorare più volte le stesse zone per migliorare la qualità
6. Premere `S` per salvare la mappa
7. Premere `Esc` per uscire

### Navigare con Nav2

1. Costruire e salvare una mappa (workflow precedente)
2. Posizionare Alvik nella **stessa posizione** in cui era quando ha avviato SLAM
3. Premere `R` in Godot per resettare l'odometria
4. Premere `N` — avvia Nav2 con la mappa salvata
5. Attendere `Nav2 pronto — clicca sulla mappa per il goal`
6. Premere `H` per abilitare le coordinate mouse
7. Cliccare un punto nella zona libera (grigio chiaro) della mappa
8. Alvik si muove autonomamente verso il goal

### Test goal da terminale

```bash
alvik_shell

# Goal a 25cm avanti
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: -0.25, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
```

---

*Fine Guida SLAM e Nav2 — Alvik Digital Twin v1.7*
