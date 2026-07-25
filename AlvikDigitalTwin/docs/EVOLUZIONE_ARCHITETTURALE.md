# Evoluzione Architetturale — Alvik Digital Twin

**Versione:** 1.7  
**Data:** Maggio 2026

---

## Indice

1. [Fase 1 — Struttura iniziale](#1-fase-1--struttura-iniziale)
2. [Fase 2 — Problematiche emerse](#2-fase-2--problematiche-emerse)
3. [Fase 3 — Struttura finale v1.7](#3-fase-3--struttura-finale-v17)
4. [Recap versioni e problemi risolti](#4-recap-versioni-e-problemi-risolti)
5. [Annotazioni tecniche](#5-annotazioni-tecniche)

---

<div style="page-break-before: always;"></div>

## 1. Fase 1 — Struttura iniziale

### Descrizione

La struttura iniziale del sistema prevedeva due bridge Python separati e indipendenti, ciascuno con una responsabilità distinta:

- **bridge_DDS_v1_5.py** — gestiva esclusivamente la telemetria UDP a 50Hz da Alvik e la distribuzione dei dati via DDS binario verso Godot e ROS2
- **bridge_websocket_v1_6.py** — gestiva la comunicazione bidirezionale WebSocket con Godot per l'invio dei comandi (teleop, waypoint, path, allineamento)

Questa separazione sembrava pulita architetturalmente: ogni bridge aveva una sola responsabilità. Il container Docker veniva avviato separatamente e `alvik_ros_bridge` girava come nodo ROS2 standalone al suo interno.

---

*[Diagramma: architettura_v16_bridge_separati.drawio]*

*Fase 1 — Due bridge Python separati con sincronizzazione manuale*

---

### Punti di forza iniziali

- Separazione delle responsabilità (Single Responsibility Principle)
- Indipendenza dei due moduli

### Limitazioni non anticipate

- Necessità di sincronizzazione manuale tra i due bridge (stato condiviso)
- Doppia connessione UDP verso Alvik (comandi da due sorgenti)
- Complessità di avvio e gestione dei processi

---

<div style="page-break-before: always;"></div>

## 2. Fase 2 — Problematiche emerse

### Descrizione

Durante lo sviluppo e i test sono emersi tre problemi strutturali che hanno richiesto una revisione architetturale.

---

*[Diagramma: architettura_problematiche.drawio]*

*Fase 2 — Problematiche emerse: conflitto alvik_ros_bridge, canale DDS:4445, sincronizzazione*

---

### Problema 1 — alvik_ros_bridge: nodo standalone vs nodo nel container

`alvik_ros_bridge` era stato concepito come nodo ROS2 standalone eseguito direttamente sull'host. Tuttavia, per motivi di isolamento dell'ambiente ROS2, si è scelto di eseguirlo dentro il container Docker.

Questo ha creato una ambiguità architetturale:
- Il bridge DDS pubblicava su UDP :4445 verso `localhost`
- `alvik_ros_bridge` era dentro il container e doveva ricevere su una porta mappata
- La mappatura delle porte Docker aggiungeva latenza e complessità di configurazione

```
Host:
  bridge_DDS → UDP :4445 → localhost

Docker (container):
  alvik_ros_bridge ← porta :4445 mappata dall'host
  (ma il container ha il suo namespace di rete)
```

### Problema 2 — Canale UDP DDS :4445

Il canale UDP DDS verso ROS2 (porta :4445) era gestito dal bridge DDS, ma i comandi Nav2 (`/cmd_vel`) dovevano tornare dal container verso il bridge DDS per essere inviati ad Alvik. Questo creava un ciclo bidirezionale difficile da gestire con due processi separati:

```
bridge_DDS  →  UDP :4445  →  container (alvik_ros_bridge)
                                      ↓ /cmd_vel Nav2
bridge_DDS  ←  UDP :5006  ←  container
```

Il bridge WS non aveva accesso diretto ai comandi provenienti da Nav2, rendendo necessaria una sincronizzazione IPC tra i due bridge.

### Problema 3 — Sincronizzazione tra i due bridge

Lo stato condiviso tra i due bridge (posizione corrente di Alvik, modalità attiva, flag di navigazione) richiedeva una sincronizzazione manuale tramite file condivisi o IPC. Questo era fonte di bug difficili da riprodurre e debuggare.

Esempio problematico:
```python
# bridge_DDS aggiorna la posizione
current_pose = {"x": 25.0, "y": 0.0, "theta": 0.12}

# bridge_WS legge la stessa posizione per il path following
# ma potrebbe leggere un valore intermedio (race condition)
pose = read_shared_state()  # non thread-safe
```

---

<div style="page-break-before: always;"></div>

## 3. Fase 3 — Struttura finale v1.7

### Descrizione

La soluzione adottata in v1.7 è stata l'**unificazione dei due bridge** in un unico processo Python `bridge_v1_7.py`, utilizzando i pattern di concorrenza corretti per gestire i due contesti di esecuzione (thread sincrono RT e event loop asyncio).

---

*[Diagramma: architettura_v17_bridge_unificato.drawio]*

*Fase 3 — Bridge unificato con Thread DDS + asyncio e comunicazione thread-safe via asyncio.Queue*

---

### Soluzione architetturale

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
│   └── Thread DDS ← CMD ← asyncio WS handler
│
└── asyncio event loop
    ├── WebSocket server :8765 ↔ Godot
    ├── path_follow_loop (20Hz)
    └── subprocess → Docker ROS2 (SLAM, Nav2, PID)
```

### Vantaggi della soluzione unificata

| Aspetto | Bridge separati (v1.6) | Bridge unificato (v1.7) |
|---|---|---|
| Stato condiviso | File/IPC — race condition | Variabili Python — thread-safe con Queue |
| Comandi Nav2 | Ciclo bidirezionale complesso | Loop unico nel thread DDS |
| Avvio/arresto | Due processi da gestire | Un solo processo |
| Debugging | Due log separati | Un unico log |
| Latenza comandi | Overhead IPC | asyncio.Queue in-process |

### Pattern tecnici chiave

**asyncio.Queue** — l'unico meccanismo thread-safe per passare i comandi dal WebSocket handler (asyncio) al thread DDS (threading):

```python
# asyncio WS handler — mette il comando nella coda
async def ws_handler(websocket):
    data = json.loads(await websocket.recv())
    await cmd_queue.put({"type": "cmd", "vlin": 10.0, "vang": 0.0})

# Thread DDS — legge dalla coda senza bloccare
def dds_thread():
    while True:
        try:
            cmd = cmd_queue.get_nowait()  # non bloccante
            send_to_alvik(cmd)
        except asyncio.QueueEmpty:
            pass
```

**run_in_executor** — per eseguire funzioni bloccanti (subprocess Docker) senza congelare l'event loop:

```python
loop = asyncio.get_event_loop()
await loop.run_in_executor(None, partial(nav2_start))
```

---

<div style="page-break-before: always;"></div>

## 4. Recap versioni e problemi risolti

### v1.0 — Prototipo iniziale
- Digital Twin 3D base con Godot Engine 4
- Comunicazione UDP semplice con Alvik
- Visualizzazione posizione e orientamento

### v1.1 — v1.4 — Sviluppo incrementale
- Aggiunta visualizzazione raggi ToF
- Implementazione modalità Teleop
- Modalità Waypoint con click sul pavimento
- Path following proporzionale
- Modalità Path fissi (quadrato, cerchio, triangolo)

### v1.5 — Bridge DDS
- `bridge_DDS_v1_5.py` — thread RT 50Hz per telemetria
- Protocollo DDS binario custom per la trasmissione locale
- Prima integrazione con ROS2 tramite `alvik_ros_bridge`

### v1.6 — Bridge WebSocket + SLAM
- `bridge_websocket_v1_6.py` — asyncio per comandi Godot
- Modalità Esploratore (SLAM + PID controller)
- Allineamento Digital Twin con orientamento fisico
- Primo tentativo di integrazione Nav2

### v1.7 — Bridge unificato + Nav2 completo

#### Problemi risolti in v1.7

| # | Problema | Soluzione |
|---|---|---|
| 1 | Due bridge separati con sync manuale | Unificazione in `bridge_v1_7.py` con asyncio.Queue |
| 2 | `alvik_ros_bridge` standalone vs container | Eseguito sempre dentro Docker, DDS :4445 verso container |
| 3 | Container con nome casuale ad ogni avvio | `docker-compose up -d` con `container_name: alvik_ros2` |
| 4 | `use_scan_matching` non persistente | docker-compose build ricostruzione immagine |
| 5 | TF gap 200ms durante latenza WiFi | TF pubblicato a 100Hz continuo anche senza nuovi dati |
| 6 | Goal Nav2 ignorato (timestamp=0) | `ros2 action send_goal` invece di `topic pub` |
| 7 | Doppio `behavior_server` Nav2 | Singolo `bringup_launch.py` invece di due launch separati |
| 8 | Theta accumulato (-603°) posa iniziale | Normalizzazione theta tra -π e +π prima del quaternione |
| 9 | Coordinate Godot→ROS2 errate | Formula verificata sperimentalmente: `ros_x=-point.x, ros_y=point.z` |
| 10 | Label Godot aggiornata a 60fps | Timer 1Hz per posa e ToF |
| 11 | `alvik_ros_bridge` label indistinta | Label Bridge/Alvik separati con `last_receive_msec` |
| 12 | Allineamento pubblicato su ROS2 | Rimosso `publish_alignment()` — solo Godot usa l'offset |
| 13 | Script Godot obsoleto in .tscn | Corretto riferimento da v1_6_old a v1_7 |
| 14 | Path fissi senza ritorno posa iniziale | Ultimo waypoint con campo `theta` — bridge ruota al theta iniziale |
| 15 | Marker waypoint senza coordinate | `Label3D` con numero e coordinate cm su ogni marker |
| 16 | KEY_4/5 conflitto in modalità PATH | Guard `current_mode != Mode.PATH` |
| 17 | `transform_tolerance` troppo bassa | Portata da 0.1 a 0.5 nei costmap Nav2 |
| 18 | `max_laser_range` troppo basso | Portato da 0.15 a 2.0 per SLAM efficace |
| 19 | Log bridge non persistente | Sistema log su file con rotazione automatica (max 100MB) |
| 20 | Nessuna coordinata mouse in Godot | Tasto H toggle label coordinate in tutte le modalità |

---

<div style="page-break-before: always;"></div>

## 5. Annotazioni tecniche

### 5.1 Sistema di riferimento — conversione coordinate

La conversione tra i tre sistemi di riferimento è stata la sfida tecnica più complessa del progetto, risolta sperimentalmente.

**Sistemi di riferimento:**

```
Alvik fisico:
  X = avanti (direzione marcia)
  Y = sinistra
  Theta = 0 quando punta avanti, cresce antiorario

Godot Engine:
  X = destra
  Y = su (altezza)  
  Z = profondità (avanti = -Z)

ROS2 (frame odom/map):
  X = -alvik_x  (negato in alvik_ros_bridge)
  Y = -alvik_y  (negato in alvik_ros_bridge)
  Theta = alvik_theta + π
```

**Trasformazione in alvik_ros_bridge.py:**
```python
self.theta = float(theta) + math.pi      # offset +π
odom.pose.pose.position.x = -self.x     # negato
odom.pose.pose.position.y = -self.y     # negato
```

**Formula conversione Godot → ROS2 (goal Nav2):**
```gdscript
# Verificata sperimentalmente: 25cm avanti in Alvik
# → Godot label X:+0.26 → ROS2 x:-0.257
var ros_x = -point.x   # X Godot → -X ROS2
var ros_y =  point.z   # Z Godot → +Y ROS2
# point è già in metri in Godot — NON dividere per CM_TO_M
```

**Theta goal Nav2:**
```gdscript
# +π per compensare l'offset in alvik_ros_bridge
var theta_nav = (float(theta_raw) + PI) if theta_raw != null else PI
while theta_nav >  PI: theta_nav -= 2 * PI
while theta_nav < -PI: theta_nav += 2 * PI
```

---

### 5.2 TF continuo a 100Hz

**Problema:** gap fino a 200ms nel TF durante picchi di latenza WiFi causavano errori Nav2:
```
Timed out waiting for transform from base_link to map
```

**Causa:** `alvik_ros_bridge` pubblicava il TF solo alla ricezione di un nuovo TickId da Alvik.
Con WiFi instabile, i pacchetti arrivavano con ritardo variabile.

**Soluzione:** pubblicazione TF a 100Hz continuo indipendente dai dati:
```python
def update(self):
    now = self.get_clock().now().to_msg()
    # Pubblica TF SEMPRE a 100Hz con l'ultima posizione nota
    if self.x is not None and self.y is not None and self.theta is not None:
        self._publish_tf(now)
    # Aggiorna odom e scan SOLO con nuovo TickId
    if tick_id == self.last_tick_id:
        return
    # ... aggiorna dati
```

---

### 5.3 SLAM con odometria pura

**Scelta:** `use_scan_matching: false` in slam_params.yaml.

**Motivazione:** I 5 sensori ToF di Alvik hanno letture instabili in ambienti aperti.
Con `use_scan_matching: true`, slam_toolbox tentava di correggere l'odometria
usando 5 soli punti con alta varianza, peggiorando la qualità della mappa.

**Come funziona lo SLAM senza scan matching:**
1. **Posizione** — stimata esclusivamente da odometria encoder (`/odom`)
2. **Ostacoli** — i 5 ToF marcano le celle occupate nella griglia
3. **Aggiornamento** — ogni 5cm di spostamento o 0.1 rad di rotazione

La qualità della mappa dipende dalla precisione dell'odometria e migliora
esplorando più volte gli stessi percorsi.

---

### 5.4 Goal Nav2 via action invece di topic

**Problema:** `ros2 topic pub --once /goal_pose` inviava messaggi con `stamp.sec=0`
che `bt_navigator` ignorava silenziosamente senza errori evidenti.

**Causa:** `ros2 topic pub` non imposta il timestamp ROS2 correttamente.
Nav2 richiede un timestamp valido per accettare il goal.

**Soluzione:**
```bash
# PRIMA — non funzionava (timestamp zero):
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {...}}'

# DOPO — funziona (timestamp automatico dal clock ROS2):
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {...}}}'
```

---

### 5.5 Errore odometrico nelle rotazioni

**Osservazione:** errore sistematico di ~7-10° per giro completo (360°).

**Caratteristica importante:** l'errore è **non cumulativo** (reversibile).
Tornando nella posizione iniziale con rotazione inversa, il theta torna a 0.

**Causa:** gioco meccanico dell'albero motore, non slittamento delle ruote.

**Implicazioni:**
- `BASE_WIDTH_M` non è calibrabile via software — il problema è hardware
- Il valore `0.090` è stato mantenuto come default
- Un miglioramento futuro potrebbe intervenire sul firmware STM32
  o sfruttare la sensor fusion con l'IMU integrata per correggere la deriva angolare

---

### 5.6 Container Docker nome fisso

**Problema:** `docker-compose run --rm` genera nomi casuali
(`ros2-alvik_ros2-run-XXXXXXXX`) ad ogni avvio.

**Causa:** `docker-compose run` crea un container temporaneo con nome generato.

**Soluzione:** `docker-compose up -d` usa il `container_name` definito in
`docker-compose.yml`:
```yaml
services:
  alvik_ros2:
    container_name: alvik_ros2   # nome fisso
```

**Effetto collaterale:** con `up -d` il processo `alvik_ros_bridge` gira
come daemon dentro il container, non come processo figlio dell'host.
Il `pkill` dall'host non funziona — bisogna usare:
```bash
docker exec alvik_ros2 bash -c "pkill -f alvik_ros_bridge"
```

---

### 5.7 Theta accumulativo e posa iniziale Nav2

**Problema:** dopo sessioni con molte rotazioni, il theta di Alvik era accumulato
(es. -13.68 rad = -603°). AMCL riceveva una posa iniziale con angolo errato.

**Causa:** il firmware Alvik accumula il theta senza normalizzarlo.

**Soluzione:** normalizzazione prima di calcolare il quaternione:
```python
raw_theta = current_pose["theta"]
while raw_theta >  math.pi: raw_theta -= 2 * math.pi
while raw_theta < -math.pi: raw_theta += 2 * math.pi
theta = raw_theta + math.pi   # offset +π
qz = math.sin(theta / 2.0)
qw = math.cos(theta / 2.0)
```

---

*Fine documento — Evoluzione Architetturale Alvik Digital Twin v1.7*
