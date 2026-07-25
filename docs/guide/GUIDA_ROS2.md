<div align="center" style="margin-top: 200px;">

# Alvik Digital Twin v1.7

### ROS2 — BREVE GUIDA

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

# Guida ROS2 — Alvik Digital Twin v1.7

**Versione:** 1.7  
**Data:** Maggio 2026

---

## Indice

1. [Introduzione](#1-introduzione)
2. [Architettura ROS2 nel progetto](#2-architettura-ros2-nel-progetto)
3. [Nodi](#3-nodi)
4. [Topic e messaggi](#4-topic-e-messaggi)
5. [TF Tree](#5-tf-tree)
6. [Comandi utili](#6-comandi-utili)
7. [RViz2](#7-rviz2)
8. [Parametri di configurazione](#8-parametri-di-configurazione)

---

## 1. Introduzione

ROS2 Humble è il middleware robotico che fornisce l'infrastruttura per SLAM, navigazione autonoma e PID controller nel progetto. Gira interamente dentro il container Docker `alvik_ros2` e comunica con il bridge tramite il protocollo DDS binario custom su UDP :4445.

### Integrazione nel sistema

```
bridge_v1_7.py
    │  UDP DDS binario :4445
    ▼
alvik_ros_bridge.py  (dentro Docker)
    │
    ├── /odom      → slam_toolbox, Nav2, PID controller
    ├── /tf        → tutti i nodi (100Hz continuo)
    └── /scan      → slam_toolbox, Nav2 costmap
              │
              ├── slam_toolbox  → /map
              └── Nav2          → /cmd_vel → bridge → Alvik
```

---
<br>

## 2. Architettura ROS2 nel progetto

### alvik_ros_bridge.py

Converte la telemetria DDS in messaggi ROS2 standard. È il nodo centrale — sempre attivo quando il container è in esecuzione.

<div style="page-break-after: always;"></div>

**Trasformazioni applicate:**
```python
# Posizione: negata rispetto al frame Alvik
odom.pose.pose.position.x = -self.x   # cm → m, negato
odom.pose.pose.position.y = -self.y

# Orientamento: offset +π
self.theta = float(theta) + math.pi

# Velocità angolare: segno invertito
self.vth = -(wr_rads - wl_rads) / BASE_WIDTH_M * WHEEL_RADIUS_M
```

**TF continuo a 100Hz:**

Dalla v1.7 il TF viene pubblicato a 100Hz anche senza nuovi dati da Alvik, per evitare gap durante la latenza WiFi:

```python
def update(self):
    now = self.get_clock().now().to_msg()
    if self.x is not None and self.y is not None and self.theta is not None:
        self._publish_tf(now) # SEMPRE a 100Hz
    if tick_id == self.last_tick_id:
        return                # aggiorna odom e scan solo con nuovo TickId
```

### alvik_pid_controller.py

Controller PID a cascata per il path following. Legge `/odom` e pubblica `/cmd_vel`. Attivo solo in modalità Esploratore e Calibrazione PID.

### slam_toolbox

Costruisce la mappa 2D dell'ambiente. Attivo in modalità Esploratore (E) e SLAM standalone (L).

### nav2_bringup

Stack completo di navigazione autonoma. Attivo in modalità Nav2 (N).

---

## 3. Nodi

### Sempre attivi

```bash
/alvik_ros_bridge
```
<div style="page-break-after: always;"></div>

### Attivi in modalità Esploratore (E)

```bash
/alvik_ros_bridge
/async_slam_toolbox_node
/alvik_pid_controller
/robot_state_publisher
/joint_state_publisher
```

### Attivi in modalità SLAM standalone (L)

```bash
/alvik_ros_bridge
/async_slam_toolbox_node
/robot_state_publisher
/joint_state_publisher
```

### Attivi in modalità Nav2 (N)

```bash
/alvik_ros_bridge
/map_server
/amcl
/bt_navigator
/bt_navigator_navigate_to_pose_rclcpp_node
/bt_navigator_navigate_through_poses_rclcpp_node
/controller_server
/planner_server
/behavior_server
/smoother_server
/waypoint_follower
/velocity_smoother
/lifecycle_manager_localization
/lifecycle_manager_navigation
/robot_state_publisher
/joint_state_publisher
```

### Verifica nodi attivi

```bash
alvik_shell
ros2 node list
ros2 node info /alvik_ros_bridge
```

---

<div style="page-break-after: always;"></div>

## 4. Topic e messaggi

### Topic principali

| Topic | Tipo | Frequenza | Descrizione |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 50Hz | Posizione e velocità da odometria encoder |
| `/tf` | `tf2_msgs/TFMessage` | 100Hz | Trasformazioni frame (continuo) |
| `/scan` | `sensor_msgs/LaserScan` | 50Hz | Laser scan da 5 sensori ToF |
| `/map` | `nav_msgs/OccupancyGrid` | ~0.2Hz | Mappa SLAM (aggiornata ogni 5s) |
| `/cmd_vel` | `geometry_msgs/Twist` | 10Hz | Comandi velocità da Nav2/PID |
| `/pid/command` | `std_msgs/String` | on demand | Comandi al PID controller |
| `/pid_debug` | `std_msgs/Float32MultiArray` | 20Hz | Dati debug PID per PlotJuggler |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | on demand | Posa iniziale per AMCL (*Adaptive Monte Carlo Localization*)|
| `/goal_pose` | `geometry_msgs/PoseStamped` | on demand | Goal navigazione |

<br>
<br>

### Scelte progettuali sulle frequenze

**`/odom` e `/scan` a 50Hz** — frequenza vincolata dalla telemetria UDP di Alvik che invia dati a 50Hz (ogni 20ms). Non è possibile aumentarla senza modificare il firmware.

**`/tf` a 100Hz** — pubblicato a frequenza doppia rispetto all'odometria per garantire continuità anche durante i gap WiFi (fino a 200ms). Nav2 e slam_toolbox richiedono il TF disponibile ad ogni ciclo — un gap causa errori di trasformazione che interrompono la navigazione.

<div style="page-break-after: always;"></div>

**`/map` a ~0.2Hz (ogni 5s)** — valore di default di slam_toolbox, mantenuto per scelta progettuale. Pubblicare la mappa è un'operazione costosa (griglia 58×47 celle serializzata su ROS2). Con il sistema già impegnato a gestire TF a 100Hz, odom a 50Hz e WebSocket, un aggiornamento più frequente avrebbe appesantito inutilmente il sistema. Per la costruzione della mappa durante l'esplorazione e per Nav2, 5 secondi sono più che sufficienti.

**`/cmd_vel` a 10Hz** — frequenza del controller Nav2 (`controller_frequency: 10.0` in `nav2_params.yaml`). Sufficiente per Alvik che ha velocità massima di 12 cm/s.

**`/pid_debug` a 20Hz** — frequenza del loop PID controller, adeguata per visualizzare la risposta del sistema in PlotJuggler senza sovraccaricare il bus ROS2.  

<br>
<br>

### Verifica topic

```bash
alvik_shell

# Lista tutti i topic
ros2 topic list

# Frequenza di pubblicazione
ros2 topic hz /tf
ros2 topic hz /odom

# Contenuto
ros2 topic echo /odom --once | grep -A5 position
ros2 topic echo /scan --once | grep ranges
```

### Inviare comandi manuali

```bash
# Velocità diretta
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Comando PID
ros2 topic pub --once /pid/command std_msgs/msg/String \
  '{data: "goto_rel:25.0,0.0"}'

# Goal Nav2 (via action)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: -0.25, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'
```

---

<div style="page-break-after: always;"></div>

## 5. TF Tree

### Struttura

```
map
 └── odom
      └── base_footprint
           └── base_link
                ├── laser_frame
                ├── left_wheel
                ├── right_wheel
                └── tof_link
```
<br>

### Fixed Frame per modalità

| Modalità | Fixed Frame RViz2 | File .rviz |
|---|---|---|
| Teleop / Debug | `odom` | `alvik.rviz` |
| SLAM / Esploratore | `map` | `alvik_map.rviz` |
| Nav2 | `map` | `alvik_nav.rviz` |

<br>

### Verifica TF

```bash
alvik_shell
ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | head -5
```

<br>

Output atteso:
```
- Translation: [x, y, 0.000]
- Rotation: in RPY (radian) [0.000, -0.000, theta]
```

---

<div style="page-break-after: always;"></div>

## 6. Comandi utili

### Diagnostica

```bash
alvik_shell

# Verifica frequenza TF (atteso ~100Hz)
ros2 topic hz /tf

# Posizione corrente
ros2 topic echo /odom --once | grep -A3 position

# Stato navigazione
ros2 topic echo /navigate_to_pose/_action/status --once | grep status

# Log nodo
ros2 node info /alvik_ros_bridge
```
<br>

### Salvataggio mappa

```bash
alvik_shell
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  '{name: {data: "/ros2_ws/config/alvik_map"}}'
```

<br>

### Posa iniziale AMCL manuale

```bash
alvik_shell
ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}}}}'
```

<br>

### Verifica mappa caricata

```bash
ros2 topic echo /map --once | head -10
```

---
<br>

## 7. RViz2

<br>

### Configurazioni disponibili

| File | Uso |
|---|---|
| `alvik.rviz` | Fixed Frame=odom, per Teleop e debug |
| `alvik_map.rviz` | Fixed Frame=map, per SLAM ed Esploratore |
| `alvik_nav.rviz` | Fixed Frame=map, per Nav2 con mappa caricata |

<br>

### Avvio manuale RViz2

```bash
DISPLAY=:1 docker exec -it alvik_ros2 bash -c \
  "source /opt/ros/humble/setup.bash && rviz2 -d /ros2_ws/config/alvik_nav.rviz"
```

<br>

### Display utili in RViz2

| Display | Topic | Uso |
|---|---|---|
| `Odometry` | `/odom` | Posizione e orientamento |
| `LaserScan` | `/scan` | Raggi ToF (Size: 0.05) |
| `TF` | — | Frame di riferimento |
| `Map` | `/map` | Mappa SLAM |
| `Path` | `/plan` | Percorso pianificato Nav2 |

---

<div style="page-break-after: always;"></div>

## 8. Parametri di configurazione

### alvik_ros_bridge.py

```python
WHEEL_RADIUS_M = 0.017   # raggio ruota (m)
BASE_WIDTH_M   = 0.090   # distanza interasse (m)
```
<br>

### slam_params.yaml

```yaml
use_scan_matching: false     # odometria pura (5 ToF insufficienti per scan matching)
max_laser_range: 2.0         # range massimo considerato da slam_toolbox
minimum_travel_distance: 0.05
minimum_travel_heading: 0.1
resolution: 0.05             # 5cm per cella
map_update_interval: 5.0     # pubblica /map ogni 5s
tf_buffer_duration: 30.0
transform_timeout: 0.2
```
<br>

### nav2_params.yaml (estratto)

```yaml
# Tolleranza TF — compensazione gap WiFi
transform_tolerance: 2.0      # globale
transform_tolerance: 0.5      # nei costmap (era 0.1, causa errori con WiFi instabile)

# Range sensori ToF nella costmap
laser_max_range: 0.15
raytrace_max_range: 0.15
obstacle_max_range: 0.15

# Controller
controller_frequency: 10.0
max_vel_x: 0.12               # m/s
```

---

*Fine Guida ROS2 — Alvik Digital Twin v1.7*
