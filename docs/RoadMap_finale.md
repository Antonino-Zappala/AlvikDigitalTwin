# Alvik Digital Twin — Project Roadmap

Roadmap di sviluppo per la realizzazione di un **Digital Twin 3D del robot Arduino Alvik** con comunicazione realtime tramite **UDP + DDS + Python + Godot 4**.

**Versione corrente:** 1.7  
**Data ultima modifica:** Maggio 2026

---

## Architettura del sistema

```
Arduino Alvik (ESP32 + STM32)
│
│ UDP CSV 50Hz
▼
bridge_v1_7.py  (Thread DDS + asyncio WebSocket)
│
├── UDP DDS :4444 ──→ Godot Digital Twin
├── UDP DDS :4445 ──→ alvik_ros_bridge (Docker)
└── WebSocket :8765 ↔ Godot Digital Twin
```

---

## FASE 0 – Setup ambiente ✅

### Tools
- [x] Installare **VSCode**
- [x] Installare **Robot Developer Extensions for ROS2**
- [x] Installare **Docker**
- [x] Installare **Pymakr**
- [x] Installare **Godot 4**

### Ambiente Python
- [x] Dipendenze Python installate (asyncio, websockets, subprocess)

### ROS2
- [x] Container **ROS2 Humble** configurato (Docker — Ubuntu 20.04 non supporta ROS2 Humble nativamente)
- [x] ROS2 tools verificati

---

## FASE 1 – Comunicazione DDS ✅

### Implementazione
- [x] Creare `DDS_v1_7.gd` (Autoload Godot)
- [x] Implementare server **UDP DDS** su porta :4444
- [x] Implementare: publish, subscribe, keep alive

### Test comunicazione
- [x] Inviare pose robot a Godot
- [x] Verificare stabilità comunicazione

---

## FASE 2 – Digital Twin base ✅

### Godot Project
- [x] Creare progetto `alvik_digital_twin_v1_7`
- [x] Creare scena robot con mesh fotogrammetrica (scansione smartphone + Blender)
- [x] Implementare `digital_twin_v1_7.gd`

### Robot 3D
- [x] Mesh fotogrammetrica di Alvik (scansione smartphone + pulizia Blender)
- [x] Ruote separate (modellate in Blender) con animazione indipendente tramite encoder
- [x] 5 RayCast3D per sensori ToF (L, CL, C, CR, R)

### Collegamento DDS
- [x] Subscribe variabili: X, Y, Theta, WheelLeft, WheelRight, ToF×5, TickId
- [x] Aggiornamento posizione robot a 50Hz
- [x] Sincronizzazione con TickId (evita aggiornamenti duplicati)

---

## FASE 3 – Simulazione robot ✅

### Modello
- [x] Odometria calcolata dal firmware STM32 di Alvik
- [x] Pubblicazione posizione su DDS e ROS2

---

## FASE 4 – Integrazione robot reale ✅

### Firmware ESP32
- [x] Firmware base Arduino Alvik
- [x] Lettura encoder ruote + odometria STM32
- [x] Invio dati via UDP (CSV 12 campi a 50Hz)

### Comunicazione WiFi
- [x] WiFi Direct (Alvik come Access Point — `Alvik_Robot_WiFi`)
- [x] Test ping PC ↔ Alvik (`192.168.4.1`)
- [x] UDP streaming stabile a 50Hz

---

## FASE 5 – Odometria realtime ✅

### Pipeline
```
Alvik STM32 (encoder) → UDP CSV → bridge_v1_7.py → DDS :4444 → Godot
                                                  → DDS :4445 → alvik_ros_bridge → /odom
```

### Implementazione
- [x] Ricezione telemetria Python (Thread DDS 50Hz)
- [x] Parsing CSV 12 campi: x, y, theta, wl, wr, ToF×5, TickId, t_ms
- [x] Pubblicazione su DDS (Godot) e ROS2 (/odom, /tf, /scan)

---

## FASE 6 – Debug e stabilità ✅

### Logging
- [x] `log_debug()` in Godot con BBCode e timestamp centesimale
- [x] Log su file in `bridge_v1_7.py` con rotazione automatica (max 100MB)
- [x] Label Bridge/Alvik separati con `last_receive_msec`

### Gestione connessione
- [x] Keep alive DDS
- [x] Timeout connessione WebSocket con riconnessione automatica
- [x] TF continuo a 100Hz per evitare gap durante latenza WiFi

---

## FASE 7 – Visualizzazione avanzata ✅

### Digital Twin
- [x] 5 raggi ToF colorati (verde/arancione/rosso) con ImmediateMesh
- [x] Mappa ostacoli dinamica (StaticBody3D) con sistema confidenza e TTL fade
- [x] 5 viste camera: TOP, THIRD_PERSON, FIRST_PERSON, RIGHT, LEFT
- [x] Griglia 3D e freccia posa iniziale (tasto G)
- [x] Label coordinate mouse (tasto H)
- [x] Marker waypoint con Label3D (numero + coordinate cm)

### UI
- [x] CanvasLayer sinistra — stato connessione, posa, ToF (1Hz)
- [x] CanvasLayer centrale — modalità corrente, istruzioni contestuali
- [x] CanvasLayer menu — menu comandi completo (tasto M)

---

## FASE 8 – Sensori robot ✅

### Sensori implementati
- [x] 5 sensori ToF (±60°, range ~150cm, blocco firmware a 15cm)
- [x] Encoder ruote (odometria STM32)

### Integrazione
- [x] ToF → mappa ostacoli Godot (ObstacleMap)
- [x] ToF → LaserScan ROS2 (/scan) con filtro media mobile
- [x] Encoder → animazione ruote Godot
- [x] Encoder → odometria ROS2 (/odom)

---

## FASE 9 – Autonomia robot ✅

### Controllo
- [x] PID controller a cascata (`alvik_pid_controller.py`) — ROTATING→MOVING→ADJUSTING
- [x] Path following proporzionale nel bridge (waypoint e path fissi)
- [x] Teleop da tastiera (W/X/Q/E)

### Navigazione
- [x] Path fissi: quadrato, cerchio, triangolo con ritorno alla posa iniziale
- [x] Waypoint manuali con click mouse
- [x] SLAM — costruzione mappa con slam_toolbox (odometria pura)
- [x] Nav2 — navigazione autonoma con AMCL e NavfnPlanner

---

## FASE 10 – Infrastruttura e DevOps ✅

- [x] Container Docker con nome fisso `alvik_ros2` (`docker-compose up -d`)
- [x] Script avvio/arresto (`alvik_start_v1_7.sh`, `alvik_stop_v1_7.sh`)
- [x] Menu interattivo (ENTER = applicazione, C = calibrazione PID)
- [x] PlotJuggler per calibrazione PID con `/pid_debug`
- [x] RViz2 con configurazioni salvate per SLAM e Nav2

---

## Changelog versioni

### v1.0 — Aprile 2026
- Digital Twin 3D base, comunicazione UDP, visualizzazione posizione

### v1.1–v1.4
- Visualizzazione raggi ToF, Teleop, Waypoint, Path following, Path fissi
- Mappa ostacoli dinamica, allineamento Digital Twin

### v1.5
- `bridge_DDS_v1_5.py` — thread RT 50Hz, protocollo DDS binario
- Prima integrazione ROS2 tramite `alvik_ros_bridge`

### v1.6
- `bridge_websocket_v1_6.py` — asyncio per comandi Godot
- Modalità Esploratore (SLAM + PID controller)
- Primo tentativo integrazione Nav2

### v1.7 — Aprile/Maggio 2026
- **Unificazione bridge** — `bridge_v1_7.py` (Thread DDS + asyncio + asyncio.Queue)
- TF continuo a 100Hz — risolto gap WiFi
- Goal Nav2 via `ros2 action send_goal` — risolto timestamp=0
- Singolo `bringup_launch.py` — risolto conflitto `behavior_server`
- Normalizzazione theta — risolta posa iniziale errata AMCL
- Coordinate Godot→ROS2 verificate sperimentalmente
- Container nome fisso `alvik_ros2`
- Log su file con rotazione automatica
- Marker waypoint con Label3D e coordinate
- Path fissi con ritorno theta iniziale
- Label coordinate mouse (tasto H)
- Viste laterali camera (tasti 4/5)

---

## Problemi risolti (v1.7)

| # | Problema | Soluzione |
|---|---|---|
| 1 | Due bridge separati con sync manuale | Unificazione in `bridge_v1_7.py` |
| 2 | `alvik_ros_bridge` standalone vs container | Sempre dentro Docker |
| 3 | Container nome casuale ad ogni avvio | `container_name: alvik_ros2` |
| 4 | `use_scan_matching` non persistente | Rebuild immagine Docker |
| 5 | TF gap 200ms latenza WiFi | TF continuo a 100Hz |
| 6 | Goal Nav2 ignorato (timestamp=0) | `ros2 action send_goal` |
| 7 | Doppio `behavior_server` Nav2 | Singolo `bringup_launch.py` |
| 8 | Theta accumulato posa iniziale | Normalizzazione theta -π/+π |
| 9 | Coordinate Godot→ROS2 errate | Verificate sperimentalmente |
| 10 | `max_laser_range` troppo basso | Portato a 2.0 |
| 11 | `transform_tolerance` troppo bassa | Portata da 0.1 a 0.5 |
| 12 | Modalità Esploratore — mappa non visibile in RViz2 | Usare Add by Topic (non Add by Display) — configurazione salvata |

---

## Note tecniche

> **Autoload DDS:** `DDS_v1_7.gd` è registrato come Autoload in Godot:
> `Impostazioni Progetto → Globali → seleziona file → Aggiungi → Attiva flag Globale`
> Questo lo rende un singleton accessibile globalmente con `DDS.read("X")`.

> **Docker — motivazione:** Ubuntu 20.04 non supporta ROS2 Humble nativamente
> (richiede Ubuntu 22.04 minimo). Docker esegue Ubuntu 22.04 + ROS2 Humble
> in un container isolato sull'host Ubuntu 20.04.

> **Upload firmware Alvik:** premere **due volte** il pulsante RESET per entrare
> in modalità bootloader. Il LED si illumina in modalità fading.

> **Test ricezione pacchetti DDS:**
> ```python
> python -c "
> import socket, struct
> s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
> s.bind(('0.0.0.0', 4444))
> while True:
>     data, _ = s.recvfrom(256)
>     nlen = data[2]
>     name = data[3:3+nlen].decode()
>     val  = struct.unpack_from('<f', data, 3+nlen)[0]
>     print(f'{name:12s} = {val:.3f}')
> "
> ```

> **Errore DFU upload firmware:**
> ```bash
> sudo usermod -a -G dialout neobot78
> echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", MODE:="0666"' | \
>   sudo tee /etc/udev/rules.d/99-arduino-alvik.rules && \
>   sudo udevadm control --reload-rules && sudo udevadm trigger
> ```

---

## Stato progetto

| Modulo | Stato |
|---|---|
| Setup ambiente | ✅ Completato |
| Comunicazione DDS | ✅ Completato |
| Digital Twin 3D | ✅ Completato |
| Integrazione robot reale | ✅ Completato |
| Teleop | ✅ Completato |
| Waypoint manuale | ✅ Completato |
| Path fissi | ✅ Completato |
| SLAM | ✅ Completato |
| Nav2 navigazione autonoma | ✅ Completato |
| PID controller | ✅ Completato |
| Calibrazione PID | ✅ Completato |
| Infrastruttura Docker | ✅ Completato |
| Documentazione completa | ✅ Completato |
| Commenti script | ✅ Completato |

---

## Next Step (sviluppi futuri)

- [ ] Chiedere nome mappa al salvataggio SLAM
- [ ] Chiedere quale mappa caricare all'avvio Nav2
- [ ] Rimozione automatica ostacoli non più rilevati durante la navigazione
- [ ] Sensor fusion IMU per correzione errore odometrico rotazione (~7-10°/giro)
- [ ] Miglioramento firmware STM32 per compensazione gioco meccanico albero motore
