# Alvik Digital Twin v1.7

**Arduino Alvik · Godot Engine 4 · ROS2 Humble · Docker**

> Digital Twin in tempo reale del robot Arduino Alvik con navigazione autonoma SLAM e Nav2, controllo waypoint, path following e calibrazione PID.

---

## Panoramica

Alvik Digital Twin v1.7 è un sistema integrato che crea un gemello digitale 3D in tempo reale del robot Arduino Alvik. Il sistema permette di visualizzare, controllare e programmare il robot fisico attraverso un'interfaccia 3D interattiva costruita con Godot Engine, con supporto completo per la navigazione autonoma tramite ROS2 Nav2 e la costruzione di mappe con SLAM Toolbox.

Per l'architettura completa, le decisioni progettuali e il percorso di sviluppo vedi la cartella [`docs/`](docs/).

### Caratteristiche principali

- **Digital Twin 3D** in tempo reale con Godot Engine 4
- **Telemetria a 50Hz** via UDP WiFi (posizione, orientamento, sensori ToF)
- **6 modalità operative** selezionabili da tastiera
- **SLAM** con slam_toolbox (odometria pura, senza scan matching)
- **Navigazione autonoma Nav2** con AMCL su mappa precostruita
- **Path following** con PID controller a cascata
- **Log su file** con rotazione automatica (max 100MB)
- **Container Docker** con nome fisso per semplicità operativa

---

## Architettura del sistema

```
Arduino Alvik (192.168.4.1)
        │  UDP CSV 50Hz (x, y, theta, RPM, ToF, TickId)
        ▼
  bridge/bridge_v1_7.py  ─────────────────────────────────────┐
        │                                                    │
        │  UDP DDS binario :4444                             │  WebSocket JSON :8765
        ▼                                                    ▼
  Godot Digital Twin                                    Godot Digital Twin
  (visualizzazione 3D)                                  (comandi modalità)
        │
        │  UDP DDS binario :4445
        ▼
  alvik_ros_bridge (Docker)
        │
        ├── /odom      (odometria)
        ├── /tf        (trasformazioni, 100Hz continuo)
        └── /scan      (laser scan da ToF, 5 raggi)
              │
              ├── slam_toolbox  → /map
              └── Nav2 stack    → /cmd_vel → bridge → Alvik
```

### Componenti

| Componente | Tecnologia | Ruolo |
|---|---|---|
| `bridge/bridge_v1_7.py` | Python asyncio | Middleware unificato DDS + WebSocket |
| `ros2/src/alvik_bridge/alvik_bridge/alvik_ros_bridge.py` | ROS2 Python node | Bridge ROS2 ↔ DDS |
| `ros2/src/alvik_bridge/alvik_bridge/alvik_pid_controller.py` | ROS2 Python node | Path following con PID |
| `godot_project/` | Godot Engine 4 GDScript | Visualizzazione 3D e controllo |
| SLAM | slam_toolbox ROS2 | Costruzione mappa 2D |
| Nav2 | nav2_bringup ROS2 | Navigazione autonoma |
| Docker | Ubuntu 22.04 + ROS2 Humble | Isolamento ambiente ROS2 |

---

## Requisiti

### Hardware
- **Arduino Alvik** con firmware aggiornato (ESP32 + STM32)
- **PC Linux** (testato su Ubuntu 22.04)
- **Rete WiFi** `Alvik_Robot_WiFi` (creata da Alvik)

### Software
- Docker + docker-compose
- Godot Engine 4.x
- Python 3.10+
- `python3-websockets`

---

## Installazione

```bash
# 1. Clona il repository
git clone <url-repo>
cd AlvikDigitalTwin

# 2. Costruisci l'immagine Docker
cd ros2
docker-compose build --no-cache

# 3. Rendi eseguibili gli script
cd ..
chmod +x scripts/alvik_start_v1_7.sh scripts/alvik_stop_v1_7.sh
```

---

## Avvio rapido

```bash
# Connetti il PC alla rete WiFi di Alvik
# Accendi Arduino Alvik

# Avvia l'applicazione
./scripts/alvik_start_v1_7.sh

# Premi ENTER per l'applicazione principale
# Premi C per la calibrazione PID
```

Per fermare tutto:
```bash
./scripts/alvik_stop_v1_7.sh
# oppure Ctrl+C nella shell di avvio
```

---

## Struttura directory

```
AlvikDigitalTwin/
├── README.md
├── CHANGELOG.md
├── docs/                         # Manuali, architettura, guide, immagini
│   ├── MANUALE_UTENTE.md
│   ├── ARCHITETTURA_TECNICA.md
│   ├── EVOLUZIONE_ARCHITETTURALE.md
│   ├── RELAZIONE_FINALE.md
│   ├── RoadMap_finale.md
│   ├── guide/                    # Docker, ROS2, SLAM/Nav2, calibrazione PID
│   └── img/                      # Screenshot e diagrammi
├── scripts/
│   ├── alvik_start_v1_7.sh       # Script di avvio
│   └── alvik_stop_v1_7.sh        # Script di arresto
├── bridge/
│   └── bridge_v1_7.py            # Middleware unificato (commentato)
├── ros2/                         # Workspace ROS2
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── config/
│   └── src/alvik_bridge/
│       └── alvik_bridge/
│           ├── alvik_ros_bridge.py
│           ├── alvik_pid_controller.py
│           ├── alvik_launch.py
│           ├── nav2_params.yaml
│           └── slam_params.yaml
└── godot_project/                # Progetto Godot
    ├── project.godot
    ├── alvik_robot.tscn
    ├── DDS_v1_7.gd
    ├── digital_twin_v1_7.gd
    └── selezione_waypoint_v1_7.gd
```

> Nota: i log applicativi (`log/`) e le cartelle di build/cache (`.godot/`, `ros2/install/`, `__pycache__/`) non sono versionati — vedi `.gitignore`.

---

## Modalità operative

| Tasto Godot | Modalità | Descrizione |
|---|---|---|
| `T` | Teleop | Controllo manuale W/X/Q/E |
| `S` | Waypoint | Selezione punti con click, invio con SPAZIO |
| `P` | Path fissi | Quadrato / Cerchio / Triangolo con ritorno posa iniziale |
| `A` | Allineamento | Rotazione Digital Twin per allineamento con Alvik fisico |
| `E` | Esploratore | SLAM + PID controller per esplorazione autonoma |
| `L` | SLAM standalone | Costruzione mappa con teleop |
| `N` | Nav2 autonomo | Navigazione su mappa precostruita con click |

---

## Comandi shell utili

```bash
# Apri shell nel container ROS2
alvik_shell

# Visualizza log bridge in tempo reale
tail -f log/$(ls -t log/ | head -1)

# Verifica container attivo
docker ps --filter "name=alvik_ros2" --format "{{.Names}}\t{{.Status}}"

# Test PID da alvik_shell
ros2 topic pub --once /pid/command std_msgs/msg/String '{data: "goto_rel:25.0,0.0"}'
```

---

## Parametri chiave

### Odometria (alvik_ros_bridge.py)
```python
WHEEL_RADIUS_M = 0.017   # raggio ruota (m)
BASE_WIDTH_M   = 0.090   # distanza tra ruote (m)
```

### SLAM (slam_params.yaml)
```yaml
use_scan_matching: false   # odometria pura
max_laser_range: 2.0       # range ToF (m)
resolution: 0.05           # 5cm per cella
```

### Nav2 (nav2_params.yaml)
```yaml
transform_tolerance: 0.5   # tolleranza TF (s)
laser_max_range: 0.15      # range ostacoli costmap (m)
```

### PID Controller
```
kp_lin=1.5  ki_lin=0.8  kp_ang=25.0  ki_ang=0.0
acc=3.0 cm/s²  dec=2.0 cm/s²  vmax_lin=12.0 cm/s  vmax_ang=60.0 deg/s
```

---

## Note tecniche

- Il TF viene pubblicato a **100Hz continuo** anche senza nuovi dati WiFi, per evitare gap durante la latenza di rete
- La conversione coordinate **Godot → ROS2** per i goal Nav2 è: `ros_x = -point.x`, `ros_y = point.z`
- L'allineamento Godot **non viene applicato** ai goal Nav2 — ROS2 ha un frame di riferimento indipendente
- Il theta del goal Nav2 include **+π** per compensare l'offset in `alvik_ros_bridge.py`

---

## Licenza

Questo progetto è distribuito sotto **GNU General Public License v3.0 (GPLv3)** — vedi il file [`LICENSE`](LICENSE).

In breve: chiunque può usare, studiare e modificare il codice, ma qualsiasi versione derivata o distribuita deve rimanere open source sotto la stessa licenza.

```
Alvik Digital Twin v1.7
Copyright (C) 2026 Antonino Zappalà

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.
```
