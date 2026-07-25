<div align="center" style="margin-top: 200px;">

# Alvik Digital Twin v1.7

### RELAZIONE FINALE

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

# Relazione Finale — Alvik Digital Twin v1.7

**Versione:** 1.7  
**Data:** Maggio 2026  
**Corso:** Sistemi Robotici — A.A. 2025/2026

---

## Indice

1. [Descrizione del progetto](#1-descrizione-del-progetto)
2. [Documentazione prodotta](#2-documentazione-prodotta)
3. [Digital Twin in Godot Engine 4](#3-digital-twin-in-godot-engine-4)
4. [Script del progetto](#4-script-del-progetto)

---

## 1. Descrizione del progetto

Alvik Digital Twin v1.7 è un framework applicativo per la prototipazione, il controllo e la supervisione del robot **Arduino Alvik** (ESP32 + STM32). Il sistema integra visualizzazione 3D in tempo reale, navigazione autonoma SLAM e Nav2, path following con PID controller e strumenti di calibrazione in un'architettura modulare ed estendibile.

<br>

### Obiettivi raggiunti

- **Digital Twin 3D** — replica fedele del robot fisico con aggiornamento in tempo reale a 50Hz
  
- **Telemetria UDP** — ricezione di posizione, orientamento e 5 sensori ToF via WiFi
  
- **6 modalità operative** — Teleop, Waypoint, Path fissi, Allineamento, Esploratore SLAM+PID, SLAM standalone, Nav2 autonomo
  
- **SLAM** — costruzione di mappe 2D dell'ambiente con slam_toolbox
  
- **Navigazione autonoma** — pianificazione e esecuzione di percorsi con Nav2
  
- **Path following** — controllo PID a cascata per il raggiungimento di waypoint
  
- **Calibrazione PID** — strumenti di analisi e ottimizzazione dei parametri del controller
  
- **Infrastruttura robusta** — container Docker con nome fisso, log su file con rotazione, TF continuo a 100Hz
<div style="page-break-before: always;"></div>

### Stack tecnologico

| Componente | Tecnologia |
|---|---|
| Robot fisico | Arduino Alvik (ESP32 + STM32) |
| Digital Twin | Godot Engine 4 — GDScript |
| Modello 3D | Scansione fotogrammetrica + Blender |
| Middleware | Python asyncio + threading |
| Stack robotico | ROS2 Humble (Docker) |
| SLAM | slam_toolbox |
| Navigazione | Nav2 |
| Visualizzazione ROS2 | RViz2 |
| Calibrazione | PlotJuggler |

---
<br>
<br>
<br>

<div align="center">
<img src="AlvikFisicoStart.png" height="290px"/>   <img src="AlvikFisico.jpeg" height="290px"/>

*Alvik Digital Twin e Alvik fisico a confronto*
</div>

---

<div style="page-break-before: always;"></div>

## 2. Documentazione prodotta

La documentazione del progetto è articolata in sei documenti distinti, ciascuno rivolto a un aspetto specifico del sistema.

---
<br>

### 2.1 Architettura Tecnica

**File:** `ARCHITETTURA_TECNICA.md`

Documento principale che descrive in dettaglio l'intera architettura del sistema, con particolare attenzione all'evoluzione progettuale che ha portato dalla struttura iniziale con due bridge separati alla soluzione finale con bridge unificato.

Contenuto:
- Evoluzione architetturale in tre fasi con diagrammi draw.io
- Recap delle versioni da v1.0 a v1.7 con tabella dei 20 problemi risolti
- Architettura generale con schema a blocchi
- Dettaglio tecnico di ogni componente (bridge, Godot, ROS2, SLAM, Nav2, PID, Docker)
- Sistemi di riferimento e trasformazioni coordinate
- Scelte progettuali e annotazioni tecniche
- Parametri di sistema completi

---
<br>

### 2.2 Manuale Utente

**File:** `MANUALE_UTENTE.md`

Guida operativa per l'utilizzo del sistema. Descrive tutte le modalità operative con procedure passo-passo, screenshot e note tecniche. Include la sezione di risoluzione problemi per i casi più comuni.

Contenuto:
- Requisiti e preparazione dell'ambiente
- Avvio e arresto del sistema
- Interfaccia Godot — layout, tasti di sistema, label coordinate mouse
- Tutte le 7 modalità operative con procedure dettagliate
- Calibrazione PID — PlotJuggler e test da terminale
- Comandi di sistema e monitoraggio
- Risoluzione problemi

---

<div style="page-break-before: always;"></div>

### 2.3 Calibrazione PID — Guida

**File:** `GUIDA_CALIBRAZIONE_PID.md`

Guida specifica per la calibrazione del controller PID. Descrive l'architettura a cascata, la macchina a stati del controller, le procedure di test e i parametri ottimali calibrati per Alvik v1.7.

Contenuto:
- Architettura PID a cascata (anello esterno ROS2 + anello interno STM32)
- Macchina a stati: IDLE → ROTATING → MOVING → ADJUSTING
- Avvio modalità calibrazione con PlotJuggler
- Legenda completa `/pid_debug` (8 campi)
- Comandi di test da `alvik_shell`
- Procedura di calibrazione parametro per parametro
- Salvataggio parametri e rebuild Docker
- Risultati di calibrazione v1.7 con precisione misurata

---
<br>

### 2.4 Docker — Breve Guida

**File:** `GUIDA_DOCKER.md`

Guida all'uso del container Docker che isola l'ambiente ROS2 Humble dall'host Ubuntu 20.04. Docker è stato adottato come soluzione necessaria poiché **ROS2 Humble richiede Ubuntu 22.04** come versione minima supportata, mentre il sistema host è Ubuntu 20.04.

Contenuto:
- Motivazione tecnica dell'uso di Docker
- Installazione e build dell'immagine
- Struttura del container e percorsi importanti
- Avvio automatico e manuale
- Operazioni comuni (shell, copia file, aggiornamento nodi)
- Procedura di aggiornamento con e senza rebuild
- Risoluzione problemi

---

<div style="page-break-before: always;"></div>

### 2.5 ROS2 — Breve Guida

**File:** `GUIDA_ROS2.md`

Guida di riferimento per l'uso di ROS2 nel contesto del progetto. Descrive i nodi attivi per ogni modalità, i topic con frequenze e motivazioni delle scelte, il TF tree e i comandi utili da `alvik_shell`.

Contenuto:
- Architettura ROS2 nel progetto con schema di integrazione
- Nodi attivi per ogni modalità (base, Esploratore, SLAM, Nav2)
- Topic principali con frequenze e giustificazione delle scelte
- TF tree e fixed frame per modalità
- Comandi utili da `alvik_shell`
- Configurazioni RViz2
- Parametri di configurazione

---

<br>

### 2.6 SLAM & Nav2 — Breve Guida

**File:** `GUIDA_SLAM_NAV2.md`

Guida che introduce i concetti di SLAM e navigazione autonoma e descrive la loro implementazione specifica nel progetto, con dettaglio sulle scelte progettuali, i problemi riscontrati e le soluzioni adottate.

Contenuto:
- Introduzione teorica a SLAM e Nav2
- Configurazione slam_toolbox con motivazione di `use_scan_matching: false`
- Laser scan da 5 ToF — distribuzione angolare e conversione
- Architettura Nav2 con nodi e parametri
- Conversione coordinate Godot → ROS2 (verificata sperimentalmente)
- 6 problemi riscontrati con cause e soluzioni dettagliate
- Workflow operativo completo per mappa e navigazione

---

<div style="page-break-before: always;"></div>

## 3. Digital Twin in Godot Engine 4

Godot Engine 4 è il motore di gioco open source usato per realizzare il Digital Twin 3D di Alvik. La scelta è motivata dalla facilità di creazione di scene 3D interattive, dal linguaggio GDScript (simile a Python) e dal supporto nativo a UDP e WebSocket.

<br>

### 3.1 Struttura della scena

La scena principale `AlvikRobotEnv` è organizzata nel seguente albero di nodi:

```
AlvikRobotEnv
├── WorldEnvironment         ← illuminazione e ambiente
├── DirectionalLight3D       ← luce direzionale
├── Camera3D                 ← camera principale (5 viste)
├── ObstacleMap              ← contenitore ostacoli dinamici (StaticBody3D)
├── Obstacles                ← ostacoli statici della scena
├── Floor                    ← pavimento con collisioni
│   ├── MeshInstance3D       ← mesh visiva del pavimento (griglia verde)
│   └── CollisionShape3D     ← forma di collisione per raycast
├── Alvik                    ← robot Digital Twin
│   ├── geometry_0           ← mesh principale del corpo Alvik
│   ├── Wheel_Left           ← ruota sinistra (mesh separata)
│   ├── Wheel_Right          ← ruota destra (mesh separata)
│   ├── Wheel_Right_001      ← ruota destra visuale
│   ├── Wheel_Left_001       ← ruota sinistra visuale
│   ├── RayCast_L            ← raycast ToF sinistro (-60°)
│   ├── RayCast_CL           ← raycast ToF centro-sinistro (-30°)
│   ├── RayCast_C            ← raycast ToF centrale (0°)
│   ├── RayCast_CR           ← raycast ToF centro-destro (+30°)
│   └── RayCast_R            ← raycast ToF destro (+60°)
├── CanvasLayer              ← UI sinistra (stato connessione, posa, ToF)
│   └── DebugLabel           ← RichTextLabel con BBCode
├── CanvasLayer2             ← UI coordinate e info
│   ├── InfoLabel            ← label modalità corrente
│   └── CoordLabel           ← label coordinate mouse (tasto H)
└── CanvasLayer3             ← UI menu
    └── MenuLabel            ← menu comandi (tasto M)
```

---

<div style="page-break-before: always;"></div>

### 3.2 Modello 3D — scansione e Blender

Il modello 3D di Alvik è stato ottenuto tramite **scansione fotogrammetrica** con smartphone, utilizzando un'app di scansione 3D. La scansione ha prodotto una mesh poligonale del robot fisico.

La mesh è stata successivamente elaborata in **Blender**:

- **Pulizia della mesh** — rimozione di artefatti di scansione, riduzione del numero di poligoni, chiusura di fori
- **Riscalatura** — la mesh è stata riscalata alle dimensioni fisiche reali di Alvik (dimensioni misurate fisicamente)
- **Creazione ruote** — le ruote sono state modellate come **mesh separate** in Blender, indipendenti dal corpo principale, per poter essere animate indipendentemente in Godot

<br>

La separazione delle ruote è fondamentale per l'animazione: in Godot ogni ruota è un nodo indipendente (`Wheel_Left`, `Wheel_Right`) la cui rotazione viene aggiornata in tempo reale dai valori degli encoder.

---
<br>
<br>

<div align="center">
<img src="AlvikBlender.png"/>

*Modello 3D di Alvik in Blender*
</div>

---

<div style="page-break-before: always;"></div>

### 3.3 Animazione ruote con encoder

I valori `WheelLeft` e `WheelRight` (velocità in rad/s, ricevuti via DDS dal bridge) vengono usati per animare la rotazione delle ruote in tempo reale:

```gdscript
# digital_twin_v1_7.gd — aggiornamento ogni frame
if wl != null and wr != null:
    wheel_left.rotation.x  -= wl * delta
    wheel_right.rotation.x -= wr * delta
```

La rotazione è sottrattiva (segno negativo) per compensare l'orientamento della mesh. Il parametro `delta` (tempo trascorso dall'ultimo frame) garantisce un'animazione indipendente dalla frequenza di rendering.

---

<br>
<br>

### 3.4 Mappa ostacoli dinamica

Il nodo `ObstacleMap` contiene gli ostacoli rilevati in tempo reale dai 5 sensori ToF. Ogni lettura ToF valida genera uno `StaticBody3D` nella posizione dell'ostacolo rilevato, costruendo progressivamente una mappa 3D dell'ambiente circostante.

Gli ostacoli vengono usati in due modi:
1. **Visualizzazione** — mostrano graficamente gli ostacoli rilevati nella scena 3D
2. **Collision detection** — il raycast del click mouse verifica se il punto selezionato è occupato da un ostacolo prima di aggiungerlo come waypoint:

```gdscript
# selezione_waypoint_v1_7.gd
var collider = result.get("collider")
if collider != null and collider is StaticBody3D \
        and collider.get_parent().name == "ObstacleMap":
    _log("[color=red]Posizione non raggiungibile — ostacolo![/color]")
    return
```

La mappa si cancella con il tasto `C` in Godot (`clear_obstacle_map()`).

---

<div style="page-break-before: always;"></div>

<div align="center">

<img src="AlvikFisicoVistaAlto.jpeg" width="41%" height="280px" style="transform: rotate(90deg);"/> <img src="GodotOstacoli.png" width="48%" height="280px"/>

*Vista dall'alto di Alvik fisico nell'ambiente reale (sinistra) — mappa ostacoli dinamica nel Digital Twin (destra)*

</div>


---
<br>
<br>

<div align="center">

<img src="AlvikFisicoOstacoli.jpeg" width="48%" height="280px"/> <img src="GodotOstacoliLaterale.png" width="48%" height="280px"/>

*Vista laterale di Alvik fisico nell'ambiente reale (sinistra) — mappa ostacoli dinamica nel Digital Twin (destra)*

</div>

---
<br>
<br>

### 3.5 Raycast ToF

I 5 nodi `RayCast_L/CL/C/CR/R` simulano visivamente i sensori ToF del robot fisico. Sono posizionati sul nodo `Alvik` e puntano nelle direzioni corrispondenti (-60°, -30°, 0°, +30°, +60°). La lunghezza di ogni raggio viene aggiornata in tempo reale con il valore del sensore corrispondente, fornendo una rappresentazione visiva del campo di rilevamento.

---

<div style="page-break-before: always;"></div>

### 3.6 Sistema di viste camera

Sono disponibili 5 viste selezionabili da tastiera:

| Tasto | Vista | Comportamento |
|---|---|---|
| `1` | Dall'alto | Camera zenitale, segue il robot |
| `2` | Da dietro | Camera a terza persona posteriore |
| `3` | Prima persona | Camera sul "muso" del robot |
| `4` | Laterale destra | Camera sul lato destro, `look_at` sul robot |
| `5` | Laterale sinistra | Camera sul lato sinistro, `look_at` sul robot |

Tutte le viste seguono il robot in movimento. Lo zoom è controllato dalla rotellina del mouse.

---
<br>
<br>

### 3.7 Sistema UI — CanvasLayer

L'interfaccia utente è organizzata su tre `CanvasLayer` sovrapposti alla scena 3D:

**CanvasLayer — UI sinistra** (`DebugLabel`)
- Stato connessione Bridge e Alvik (verde/rosso)
- Posa corrente: X, Y, Theta (aggiornata a 50Hz modificabile tramite `if _log_pose_timer`)
- Letture ToF: L, CL, C, CR, R in cm (aggiornate a 50Hz modificabile tramite `if _log_pose_timer`)
- Log operativo con timestamp centesimale `[HH:MM:SS.cs]`

**CanvasLayer2 — UI centrale/destra**
- `InfoLabel` — modalità corrente e istruzioni contestuali
- `CoordLabel` — label coordinate mouse (attivata con `H`), mostra le coordinate Godot e Alvik in cm del punto sotto il cursore

**CanvasLayer3 — Menu**
- `MenuLabel` — menu comandi completo (attivato con `M`), con sezioni NAVIGAZIONE, MODALITÀ, SISTEMA formattate con BBCode e colori

---
<div style="page-break-before: always;"></div>

### 3.8 Marker waypoint con Label3D

In modalità Waypoint ogni punto selezionato genera un marker visivo composto da:
- Un nodo 3D (pallino rosso) posizionato sul pavimento
- Un `Label3D` giallo con numero del punto e coordinate in cm, traslato lateralmente per non coprire il marker nella vista dall'alto (`position = Vector3(-0.08, 0.05, -0.08)`)
- Billboard abilitato — il label è sempre rivolto verso la camera

```gdscript
var label = Label3D.new()
label.text = "%d: (%.0f, %.0f) cm" % [index, point.z / CM_TO_M, point.x / CM_TO_M]
label.font_size     = 12
label.modulate      = Color(1.0, 1.0, 0.2, 1.0)
label.billboard     = BaseMaterial3D.BILLBOARD_ENABLED
label.no_depth_test = true
label.position      = Vector3(-0.08, 0.05, -0.08)
```

---
<br>
<br>

<div align="center">

<img src="GodotWaypoints.png"/>

*Modalità Waypoint con marker rossi e label coordinate gialle*

</div>

---

<div style="page-break-before: always;"></div>

## 4. Script del progetto

### 4.1 Script GDScript — Godot Engine 4

Il progetto Godot utilizza **3 script GDScript**, uno dei quali è registrato come Autoload.

---

#### DDS_v1_7.gd — Autoload

**Tipo:** Autoload (singleton globale)  
**Registrazione:** `Project → Project Settings → Autoload`

Essendo registrato come **Autoload**, questo script è istanziato automaticamente all'avvio dell'applicazione Godot e rimane in memoria per tutta la durata della sessione. È accessibile globalmente da qualsiasi altro script tramite il nome `DDS`.

**Motivazione implementativa:** centralizzare la ricezione della telemetria DDS in un unico punto accessibile da tutta la scena, evitando la necessità di passare riferimenti tra nodi.

**Funzionalità:**
- Apre e gestisce il socket UDP :4444 in ricezione
- Decodifica il protocollo DDS binario custom (header magic + variabili float/int)
- Mantiene un dizionario di variabili aggiornate: `X`, `Y`, `Theta`, `WheelLeft`, `WheelRight`, `ToF_L/CL/C/CR/R`, `TickId`, `T_robot_ms`
- Espone `DDS.read("NomeVariabile")` per la lettura da qualsiasi script
- Traccia il timestamp dell'ultimo pacchetto ricevuto (`last_receive_msec`) per rilevare la disconnessione

---

#### digital_twin_v1_7.gd

**Tipo:** Script di nodo — attachato al nodo `Alvik`  
**Motivazione implementativa:** gestire l'aggiornamento del Digital Twin in funzione dei dati ricevuti via DDS, separando la logica di visualizzazione dalla logica di controllo.

**Funzionalità:**
- Legge i dati DDS ogni frame tramite `DDS.read()`
- Aggiorna posizione e rotazione del nodo `Alvik` nella scena 3D
- Anima le ruote (`Wheel_Left`, `Wheel_Right`) con i valori degli encoder
- Aggiorna i 5 `RayCast` con le letture ToF
- Genera gli ostacoli dinamici in `ObstacleMap` (`StaticBody3D`) dalle letture ToF
- Aggiorna la UI sinistra (`DebugLabel`) con stato connessione, posa e ToF a 1Hz
- Implementa `clear_obstacle_map()` per cancellare gli ostacoli dinamici
- Gestisce il log con timestamp centesimale tramite `log_debug()`

---

<div style="page-break-before: always;"></div>

#### selezione_waypoint_v1_7.gd

**Tipo:** Script di nodo — attachato al nodo radice `AlvikRobotEnv`  
**Motivazione implementativa:** gestire tutta la logica di interazione utente, le modalità operative e la comunicazione WebSocket con il bridge, separandola dalla logica di visualizzazione.

**Funzionalità principali:**
- Gestisce la macchina a stati delle modalità (`NONE`, `WAYPOINT`, `TELEOP`, `ALIGN`, `PATH`, `EXPLORER`, `SLAM`, `NAV2`)
- Implementa il client WebSocket verso il bridge (:8765)
- Gestisce l'input da tastiera per tutte le modalità
- Implementa il raycast del click mouse sul pavimento per la selezione dei punti
- Gestisce la selezione e invio dei waypoint con spawn dei marker `Label3D`
- Gestisce i path fissi (quadrato, cerchio, triangolo) con calcolo coordinate e theta finale
- Implementa le 5 viste camera con interpolazione
- Gestisce l'allineamento del Digital Twin con `align_rot_y`
- Invia i goal Nav2 con conversione coordinate Godot → ROS2
- Aggiorna la UI destra (`InfoLabel`, `CoordLabel`, `MenuLabel`)
- Processa i messaggi WebSocket in arrivo dal bridge (log, waypoint_reached, nav2_goal_reached)

---

### 4.2 Script Python — Middleware e ROS2

| Script | Posizione | Descrizione |
|---|---|---|
| `bridge_v1_7.py` | `AlvikDigitalTwin_v1_7/` | Middleware unificato: Thread DDS 50Hz + asyncio WebSocket + path following. Gestisce tutte le modalità operative e lancia i processi ROS2 nel container Docker |
| `alvik_ros_bridge.py` | `ros2/src/alvik_bridge/` | Nodo ROS2: converte telemetria DDS in `/odom`, `/tf` (100Hz), `/scan`. Applica le trasformazioni di sistema di riferimento |
| `alvik_pid_controller.py` | `ros2/src/alvik_bridge/` | Nodo ROS2: controller PID a cascata per path following. Macchina a stati ROTATING→MOVING→ADJUSTING con profilo trapezoidale |
| `alvik_launch.py` | `ros2/src/alvik_bridge/` | Launch file ROS2: avvia robot_state_publisher, joint_state_publisher, slam_toolbox (opzionale), localizzazione Nav2 (opzionale) |

---
<div style="page-break-before: always;"></div>

### 4.3 Script Shell — Avvio e arresto

| Script | Descrizione |
|---|---|
| `alvik_start_v1_7.sh` | Avvio completo: X11 forwarding, container Docker, alvik_ros_bridge, bridge Python, Godot. Menu interattivo con scelta tra applicazione principale (ENTER) e calibrazione PID (C) |
| `alvik_stop_v1_7.sh` | Arresto ordinato: bridge Python, alvik_ros_bridge nel container, container Docker, Godot |

---

*Fine Relazione Finale — Alvik Digital Twin v1.7*
