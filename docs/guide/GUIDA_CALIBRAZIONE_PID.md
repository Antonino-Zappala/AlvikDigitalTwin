<div align="center" style="margin-top: 200px;">

# Alvik Digital Twin v1.7

### CALIBRAZIONE PID — GUIDA 

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

# Guida Calibrazione PID — Alvik Digital Twin v1.7

**Versione:** 1.7  
**Data:** Maggio 2026

---

## Indice

1. [Introduzione](#1-introduzione)
2. [Architettura del controller PID](#2-architettura-del-controller-pid)
3. [Avvio modalità calibrazione](#3-avvio-modalità-calibrazione)
4. [PlotJuggler — configurazione e legenda](#4-plotjuggler--configurazione-e-legenda)
5. [Test e comandi](#5-test-e-comandi)
6. [Parametri e calibrazione](#6-parametri-e-calibrazione)
7. [Salvataggio parametri ottimali](#7-salvataggio-parametri-ottimali)
8. [Risultati di calibrazione v1.7](#8-risultati-di-calibrazione-v17)

---
<br>

## 1. Introduzione

Il controller PID di Alvik Digital Twin gestisce il **path following** — la capacità del robot di raggiungere una posizione target con precisione. È implementato come nodo ROS2 (`alvik_pid_controller.py`) e usa un'architettura a cascata con due anelli di controllo.

### Quando è attivo

| Modalità | PID attivo |
|---|---|
| Teleop (T) | No — controllo diretto |
| Waypoint (S) | No — PID del bridge |
| Path fissi (P) | No — PID del bridge |
| Esploratore (E) | **Sì** |
| SLAM standalone (L) | No |
| Nav2 (N) | No — Nav2 usa il proprio controller |
| Calibrazione (C) | **Sì** |

<br>

> **Nota:** in modalità Waypoint e Path fissi il path following è gestito dal controller proporzionale nel `bridge_v1_7.py`, non dal nodo ROS2. I parametri sono diversi.

---

<div style="page-break-after: always;"></div>

## 2. Architettura del controller PID

### Cascata a due anelli

```
Anello esterno — controllo POSIZIONE (~20Hz)
  Input:  posizione corrente da /odom (x, y, theta)
  Output: velocità desiderata (vlin, vang) → /cmd_vel

Anello interno — controllo VELOCITÀ (~1kHz, firmware STM32)
  Input:  velocità ruote da encoder
  Output: PWM motori
```

Il nodo ROS2 gestisce solo l'anello esterno. L'anello interno è trasparente, gestito dal firmware STM32 di Alvik.

<br>

### Macchina a stati

```
IDLE → ROTATING → MOVING → ADJUSTING → IDLE
           ↓ (già allineato)
         MOVING
```

- **ROTATING** — ruota sul posto verso la direzione del target (fino a `angle_thresh`)
- **MOVING** — avanza con profilo trapezoidale (rampa acc → velocità costante → rampa dec)
- **ADJUSTING** — corregge il theta finale al raggiungimento della posizione

### Profilo trapezoidale

Il profilo di velocità usa una rampa di accelerazione e decelerazione per evitare movimenti bruschi:

```
velocità
  ^
  |    ___________
  |   /           \
  |  /             \
  | /               \
  |/                 \___
  +-----------------------------> tempo
    acc  costante   dec
```

---

<div style="page-break-after: always;"></div>

## 3. Avvio modalità calibrazione

```bash
cd ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7
./alvik_start_v1_7.sh
# Al menu premere C
```

Vengono avviati automaticamente:
- Container Docker `alvik_ros2`
- `alvik_ros_bridge` — bridge DDS → ROS2
- `alvik_pid_controller` — controller PID
- **PlotJuggler** — visualizzazione dati in tempo reale

---
<br>
<br>

## 4. PlotJuggler — configurazione e legenda

### Configurazione iniziale

1. In PlotJuggler: **Streaming → ROS2 Topic Subscriber → Start**
2. Selezionare `/pid_debug` → Buffer 30sec → **OK**
3. Trascinare i campi desiderati nel pannello grafico

<br>

### Legenda /pid_debug

Il topic `/pid_debug` pubblica un `Float32MultiArray` con 8 campi:

| Campo | Descrizione | Unità |
|---|---|---|
| `data[0]` | x_target — posizione X obiettivo | cm |
| `data[1]` | x_real — posizione X attuale | cm |
| `data[2]` | y_target — posizione Y obiettivo | cm |
| `data[3]` | y_real — posizione Y attuale | cm |
| `data[4]` | theta_target — orientamento obiettivo | gradi |
| `data[5]` | theta_real — orientamento attuale | gradi |
| `data[6]` | vlin — riservato per sviluppi futuri | — |
| `data[7]` | vang — riservato per sviluppi futuri | — |

<div style="page-break-after: always;"></div>

### Grafici utili per la calibrazione

| Grafico | Cosa mostra |
|---|---|
| `data[0]` vs `data[1]` | Errore posizione X — risposta al gradino lineare |
| `data[2]` vs `data[3]` | Errore posizione Y |
| `data[4]` vs `data[5]` | Errore angolare — risposta al gradino rotazione |

---

## 5. Test e comandi

### Prerequisiti

Alvik deve essere connesso e `alvik_ros_bridge` in esecuzione. Verificare che `/odom` pubblichi dati:

```bash
alvik_shell
ros2 topic echo /odom --once | grep -A3 position
```

### Comandi da alvik_shell

```bash
alvik_shell

# Test lineare — vai avanti 25cm
ros2 topic pub --once /pid/command std_msgs/msg/String \
  '{data: "goto_rel:25.0,0.0"}'

# Test angolare — ruota 90° antiorario
ros2 topic pub --once /pid/command std_msgs/msg/String \
  '{data: "goto_rel:0.0,1.5708"}'

# Test angolare — ruota 90° orario
ros2 topic pub --once /pid/command std_msgs/msg/String \
  '{data: "goto_rel:0.0,-1.5708"}'

# Test combinato — vai a (25cm, 25cm) con theta=0
ros2 topic pub --once /pid/command std_msgs/msg/String \
  '{data: "goto:25.0,25.0,0.0"}'

# Sequenza waypoint
ros2 topic pub --once /pid/command std_msgs/msg/String \
  '{data: "path:25.0,0.0,0.0;25.0,25.0,1.5708;0.0,25.0,3.1416;0.0,0.0,0.0"}'

# Stop immediato
ros2 topic pub --once /pid/command std_msgs/msg/String \
  '{data: "stop"}'
```

### Formato comandi

| Comando | Sintassi | Descrizione |
|---|---|---|
| `goto` | `goto:x,y,theta` | Posizione assoluta (cm, cm, rad) |
| `goto_rel` | `goto_rel:dist,angle` | Movimento relativo (cm, rad) |
| `path` | `path:x1,y1,t1;x2,y2,t2;...` | Sequenza waypoint |
| `stop` | `stop` | Ferma il robot |

---

## 6. Parametri e calibrazione

### Parametri disponibili

I parametri si trovano in `alvik_pid_controller.py`:

```python
# Guadagni PID
kp_lin       = 1.5    # proporzionale lineare
ki_lin       = 0.8    # integrale lineare
kp_ang       = 25.0   # proporzionale angolare
ki_ang       = 0.0    # integrale angolare

# Profilo trapezoidale
acc_lin      = 3.0    # accelerazione (cm/s²)
dec_lin      = 2.0    # decelerazione (cm/s²)
vmax_lin     = 12.0   # velocità massima lineare (cm/s)
vmax_ang     = 60.0   # velocità massima angolare (deg/s)

# Soglie
angle_thresh = 0.07   # rad (~4°) — sotto questa soglia avanza
reach_thresh = 1.0    # cm — waypoint considerato raggiunto
```

### Effetto dei parametri

**kp_lin** — aumentarlo rende la risposta più veloce ma può causare oscillazioni. Diminuirlo rallenta la risposta.

**ki_lin** — elimina l'errore a regime. Troppo alto causa oscillazioni.

**kp_ang** — controlla la velocità di rotazione. Valori alti rendono le rotazioni più precise ma brusche.

**acc_lin / dec_lin** — controllano la fluidità del movimento. Valori bassi = movimenti più fluidi.

**vmax_lin** — velocità massima. Limitata dalla precisione dell'odometria (a velocità elevate l'odometria è meno precisa).

**angle_thresh** — soglia per passare da ROTATING a MOVING. Valore troppo alto = robot avanza prima di essere allineato.

### Procedura di calibrazione

**1. Calibrazione angolare (kp_ang):**
```bash
# Test rotazione 90°
ros2 topic pub --once /pid/command std_msgs/msg/String '{data: "goto_rel:0.0,1.5708"}'
```
- Osservare `data[4]` vs `data[5]` in PlotJuggler
- Aumentare `kp_ang` se la rotazione è troppo lenta
- Diminuire se oscilla

**2. Calibrazione lineare (kp_lin, ki_lin):**
```bash
# Test movimento 25cm
ros2 topic pub --once /pid/command std_msgs/msg/String '{data: "goto_rel:25.0,0.0"}'
```
- Osservare `data[0]` vs `data[1]` in PlotJuggler
- Regolare `kp_lin` per la velocità di risposta
- Regolare `ki_lin` per eliminare l'errore a regime

**3. Test combinato:**
```bash
ros2 topic pub --once /pid/command std_msgs/msg/String '{data: "goto:25.0,25.0,0.0"}'
```

---

## 7. Salvataggio parametri ottimali

Dopo la calibrazione, salvare i parametri nel sorgente e ricostruire l'immagine Docker:

### 1. Modificare il file sorgente

```bash
nano ~/Scrivania/progetto_alvik/ros2/src/alvik_bridge/alvik_bridge/alvik_pid_controller.py
```

Trovare la sezione parametri e aggiornare i valori.

### 2. Copiare nel container per test immediato

```bash
docker cp ~/Scrivania/progetto_alvik/ros2/src/alvik_bridge/alvik_bridge/alvik_pid_controller.py \
  alvik_ros2:/ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/alvik_pid_controller.py

# Riavviare il nodo
docker exec alvik_ros2 bash -c "pkill -f alvik_pid_controller"
```

### 3. Rendere permanenti con rebuild

```bash
cd ~/Scrivania/progetto_alvik/ros2
docker-compose build --no-cache
```

### 4. Verificare parametri nel container

```bash
docker exec alvik_ros2 bash -c \
  "grep -A5 'kp_lin\|ki_lin\|kp_ang' \
  /ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/alvik_pid_controller.py | head -15"
```

---

## 8. Risultati di calibrazione v1.7

### Parametri ottimali calibrati

```python
kp_lin       = 1.5
ki_lin       = 0.8
kp_ang       = 25.0
ki_ang       = 0.0
acc_lin      = 3.0    # cm/s²
dec_lin      = 2.0    # cm/s²
vmax_lin     = 12.0   # cm/s
vmax_ang     = 60.0   # deg/s
angle_thresh = 0.07   # rad (~4°)
reach_thresh = 1.0    # cm
```

### Precisione misurata

| Test | Target | Risultato | Errore |
|---|---|---|---|
| Movimento lineare | 25cm | ~24cm | ~4% |
| Rotazione | 90° | ~90.5° | ~0.5% |

### Note sui limiti

- L'errore lineare (~4%) è dovuto principalmente all'odometria del firmware STM32
- L'errore angolare (~7-10° per giro completo) è causato dal gioco meccanico dell'albero motore — non correggibile via software senza IMU
- I parametri sono ottimizzati per superfici lisce (pavimento); su superfici diverse potrebbero servire aggiustamenti

### Parametri fisici Alvik

```python
WHEEL_RADIUS_M = 0.017   # raggio ruota misurato (17mm)
BASE_WIDTH_M   = 0.090   # distanza interasse misurata (90mm)
```
<br>

> **Nota:** `BASE_WIDTH_M` influisce sull'odometria ROS2 ma non sulla precisione fisica di Alvik. Tentativi di calibrarlo via software non hanno prodotto miglioramenti significativi — il problema è nel gioco meccanico del firmware STM32.

---

*Fine Guida Calibrazione PID — Alvik Digital Twin v1.7*
