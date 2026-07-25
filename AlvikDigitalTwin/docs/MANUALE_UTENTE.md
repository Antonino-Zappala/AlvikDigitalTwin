<div align="center" style="margin-top: 200px;">

# Alvik Digital Twin v1.7

### Manuale Utente

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

# Manuale Utente — Alvik Digital Twin v1.7

**Versione:** 1.7  
**Data:** Maggio 2026  
**Destinatari:** Utenti del sistema Alvik Digital Twin
  

---
---

## Indice

1. [Introduzione](#1-introduzione)
2. [Requisiti e preparazione](#2-requisiti-e-preparazione)
3. [Avvio del sistema](#3-avvio-del-sistema)
4. [Interfaccia Godot](#4-interfaccia-godot)
5. [Modalità operative](#5-modalità-operative)
   - 5.1 [Teleop](#51-teleop-t)
   - 5.2 [Waypoint](#52-waypoint-s)
   - 5.3 [Path fissi](#53-path-fissi-p)
   - 5.4 [Allineamento](#54-allineamento-a)
   - 5.5 [Esploratore SLAM+PID](#55-esploratore-slampid-e)
   - 5.6 [SLAM standalone](#56-slam-standalone-l)
   - 5.7 [Nav2 autonomo](#57-nav2-autonomo-n)
6. [Calibrazione PID](#6-calibrazione-pid)
7. [Comandi di sistema](#7-comandi-di-sistema)
8. [Risoluzione problemi](#8-risoluzione-problemi)
  

---

## 1. Introduzione

Alvik Digital Twin v1.7 è un sistema di controllo e visualizzazione in tempo reale per il robot Arduino Alvik. Attraverso un'interfaccia 3D interattiva realizzata con Godot Engine 4, è possibile:

- Visualizzare la posizione e l'orientamento del robot fisico in tempo reale
- Controllare il robot manualmente o in modalità autonoma
- Costruire mappe dell'ambiente con SLAM
- Pianificare ed eseguire percorsi autonomi con Nav2
- Calibrare il controller PID

---  
<div style="page-break-before: always;"></div>

## 2. Requisiti e preparazione

### Prerequisiti
- Arduino Alvik acceso e funzionante
- PC connesso alla rete WiFi `Alvik_Robot_WiFi` (rete creata da Alvik)
- Docker in esecuzione sul PC

### Verifica connessione
Prima di avviare il sistema verificare che il PC sia connesso alla rete di Alvik:

```bash
ping 192.168.4.1
```

Se il ping risponde, Alvik è raggiungibile.

---

## 3. Avvio del sistema

### Avvio applicazione principale

```bash
cd ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7
./alvik_start_v1_7.sh
```

Alla comparsa del menu di avvio premere **ENTER** per avviare l'applicazione principale.

---  
<div align="center">  

<img src="ApplicazioneSchermataIniziale.png" width="84%">  

*Menu di avvio dell'applicazione — schermata terminale con le opzioni ENTER/C*  

</div>

---

Il sistema avvia automaticamente nella sequenza:
1. Abilitazione X11 forwarding per Docker
2. Pulizia container precedenti
3. Avvio container ROS2 (`alvik_ros2`) con `alvik_ros_bridge`
4. Avvio `bridge_v1_7.py` (middleware)
5. Avvio Godot Digital Twin

### Verifica avvio corretto
Nella finestra Godot le label in alto a sinistra devono mostrare:

```
Bridge: ● CONNESSO
Alvik:  ● CONNESSO
```
<br>

---  
<div align="center">  

![alt text](GodotBridgeAlvikConnected.png)  
*Interfaccia Godot con Bridge e Alvik connessi — label verdi in alto a sinistra*

</div>  

---

<br>

### Arresto del sistema

```bash
./alvik_stop_v1_7.sh
```  
Oppure premere **Ctrl+C** nella shell di avvio.

<div align="center">  

![alt text](alvik_stop.png)  
*Script di stop delle varie componenti dell'applicazione*

</div>  

---
<div style="page-break-before: always;"></div>

## 4. Interfaccia Godot

### Layout

L'interfaccia Godot è divisa in tre aree:

- **Area sinistra** — stato connessione, posa e sensori ToF
- **Area centrale** — vista 3D del Digital Twin
- **Area destra** — log operativo con timestamp

---  
<div align="center">

<img src="GodotSchermataIniziale.png" width="80%">
<div align="center">Schermata iniziale</div> 

<img src="GodotVistaAlto.png" width="80%">
<div align="center">Vista dall'alto</div> 

</div>

---
<br>

### Navigazione vista 3D

| Tasto | Azione |
|---|---|
| `1` | Vista dall'alto |
| `2` | Vista da dietro |
| `3` | Prima persona |
| `4` | Vista laterale destra |
| `5` | Vista laterale sinistra |
| Rotellina mouse | Zoom avanti/indietro |
  
---
<br>

<div align="center">

<img src="GodotVistaLateraleGriglia.png" width="75%">
<div align="center">Vista laterale</div> 

<img src="GodotVistaFirstPersonGriglia.png" width="75%">
<div align="center">Vista in prima persona</div> 

</div>

---

### Tasti di sistema

| Tasto | Azione |
|---|---|
| `R` | Reset posizione (azzera odometria) |
| `C` | Cancella mappa ostacoli dinamica |
| `G` | Mostra/nasconde griglia e freccia posa iniziale |
| `H` | Toggle coordinate mouse ON/OFF |
| `M` | Mostra menu comandi |
| `Shift` | Nascondi/mostra UI destra (Log Applicazione) |
| `Tab` | Nascondi/mostra UI sinistra (Stato connessione/Posa ToF)|
| `Esc` | Ferma robot / Esci dalla modalità corrente |


### Label coordinate mouse (tasto H)

Con **H** attivo, muovendo il mouse sul pavimento appare una label con le coordinate del punto sotto il cursore:

```
Godot: (0.50, 0.00)
Alvik: (50.3, -0.0) cm
```

Utile per identificare punti precisi prima di inviarli come goal.

---

## 5. Modalità operative

### 5.1 Teleop (T) <span style="font-weight: normal; font-size: 0.85em;">— Controllo manuale del robot tramite tastiera</span>

<div style="float: right; margin-left: 20px; margin-bottom: 10px; text-align: center;">
<img src="GodotTeleop.png" width="440px" height="240px"/><br/>
<em>Modalità Teleop — robot in movimento con raggi ToF verdi</em>
</div>

**Attivazione:** premere `T`  
**Uscita:** premere `Esc` o `T`

| Tasto | Movimento |
|---|---|
| `W` | Avanti |
| `X` | Indietro |
| `Q` | Rotazione sinistra |
| `E` | Rotazione destra |
  
<br>
<br>


> **Nota:** Durante il Teleop il Digital Twin segue in tempo reale la posizione di Alvik fisico, ricevuta via UDP a 50Hz.
<br clear="right"/>

---
### 5.2 Waypoint (S)

Selezione manuale di punti di percorso con il mouse.

**Attivazione:** premere `S`  
**Uscita:** premere `Esc` (cancella i marker) o `S`

**Procedura:**
1. Premere `S` per entrare in modalità Waypoint
2. Cliccare sul pavimento per aggiungere punti — appare un marker rosso con etichetta coordinate
3. Continuare ad aggiungere punti nel percorso desiderato
4. Premere **SPAZIO** per inviare il percorso ad Alvik
5. I marker rimangono visibili durante la navigazione
6. Al completamento del percorso i marker scompaiono automaticamente
7. Premere `Esc` per interrompere e cancellare i marker

---
<div align="center">  

![alt text](GodotSelectWaypoint.png)
*Modalità Waypoint — marker rossi con etichette coordinate sul pavimento*

</div>

---

> **Nota:** Ogni marker mostra il numero del punto e le coordinate in cm, es. `1: (50, 0) cm`.

---

<div style="page-break-before: always;"></div>

### 5.3 Path fissi (P)

Esecuzione di percorsi geometrici predefiniti con ritorno automatico alla posa iniziale.

**Attivazione:** premere `P`  
**Uscita:** premere `Esc`

**Sottomenù path:**

| Tasto | Percorso |
|---|---|
| `Q` | Quadrato |
| `C` | Cerchio |
| `T` | Triangolo |

**Sottomenù dimensioni:**

| Tasto | Dimensione |
|---|---|
| `1` | Piccolo |
| `2` | Medio |
| `3` | Grande |
| `4` | Extra grande |

Al termine di ogni path Alvik torna automaticamente alla posizione e all'orientamento iniziali.

> **Nota:** I tasti `4` e `5` per le viste laterali sono disabilitati durante la selezione del path per evitare conflitti.

---

### 5.4 Allineamento (A)

Rotazione del Digital Twin per allinearlo all'orientamento fisico di Alvik.

**Attivazione:** premere `A`  
**Uscita:** premere `Esc` o `A`

**Procedura:**
1. Premere `A`
2. Ruotare il Digital Twin con il mouse finché coincide con l'orientamento fisico di Alvik
3. Premere `A` o `Esc` per confermare e uscire

> **Nota:** L'allineamento influisce sulle modalità Waypoint e Path fissi, ma **non** sulle coordinate inviate a Nav2, che usa il proprio sistema di riferimento indipendente.

---

<div style="page-break-before: always;"></div>

### 5.5 Esploratore SLAM+PID (E)

Esplorazione autonoma dell'ambiente con costruzione simultanea della mappa.

**Attivazione:** premere `E`  
**Uscita:** premere `Esc` o `E`

Il sistema avvia automaticamente:
- **slam_toolbox** per la costruzione della mappa
- **alvik_pid_controller** per il path following
- **RViz2** con la mappa in costruzione

**Durante l'esplorazione:**
1. Usare il Teleop (`T`) per muovere Alvik nell'ambiente
2. La mappa si costruisce in RViz2 in tempo reale
3. Per salvare la mappa premere `S` prima di uscire dalla modalità

---
<div align="center">  

<img src="SLAM.png" width="60%">

*RViz2 con mappa SLAM in costruzione — mappa grigia con celle nere (ostacoli)*

</div>

---

> **Nota tecnica:**  
> lo SLAM usa odometria pura (`use_scan_matching: false`).  
>   
> Il processo di costruzione mappa avviene in due fasi distinte: 
> - **Posizione stimata** — slam_toolbox usa esclusivamente l'odometria degli encoder (`/odom` e `/tf`) ricevuti da `alvik_ros_bridge` per tracciare la posizione del robot nel tempo
> - **Marcatura ostacoli** — i 5 raggi ToF (`/scan`) vengono usati solo per marcare le celle occupate nella mappa, non per correggere la posizione stimata
> 
> Con `use_scan_matching: true` il robot userebbe i ToF anche per correggere la stima di posizione, ma con soli 5 raggi e letture instabili in ambienti aperti la correzione peggiorava la qualità della mappa rispetto all'odometria pura. La qualità della mappa dipende quindi dalla precisione dell'odometria e migliora esplorando più volte gli stessi percorsi.

---

### 5.6 SLAM standalone (L)

Come l'Esploratore ma senza il PID controller — adatto per la costruzione manuale della mappa con il Teleop.

**Attivazione:** premere `L`  
**Uscita:** premere `Esc` o `L`

**Procedura:**
1. Premere `L` — avvia slam_toolbox e RViz2
2. Usare il Teleop (`T`) per esplorare l'ambiente
3. Premere `S` per salvare la mappa prima di uscire

---

### 5.7 Nav2 autonomo (N)

Navigazione autonoma su mappa precostruita. Richiede che sia stata salvata una mappa in precedenza.

**Attivazione:** premere `N`  
**Uscita:** premere `Esc` o `N`

**Procedura:**
1. Costruire e salvare una mappa con la modalità E o L
2. Posizionare Alvik nella posizione iniziale (dove è stato quando si è avviato lo SLAM)
3. Premere `N` — avvia Nav2 con AMCL (*Adaptive Monte Carlo Localization*) e la mappa salvata
4. Attendere il messaggio `Nav2 pronto — clicca sulla mappa per il goal`
5. Abilitare le coordinate mouse con `H` per identificare il punto desiderato
6. Cliccare sul punto di destinazione nel Digital Twin
7. Alvik si muove autonomamente verso il goal

---

<div align="center">

<img src="GodotNav2.png" height="195px"/> <img src="Nav2.png" height="195px"/>

*Nav2 in esecuzione — Alvik che segue il percorso pianificato con RViz2*

</div>

---

> **Importante:** Prima di avviare Nav2, eseguire il Reset (`R`) in Godot se Alvik è stato mosso dalla posizione iniziale della mappa.

---
<div style="page-break-before: always;"></div>

## 6. Calibrazione PID

La modalità calibrazione PID è un'applicazione separata per ottimizzare i parametri del controller.

### Avvio

```bash
./alvik_start_v1_7.sh
# Premere C al menu
```

Vengono avviati automaticamente:
- `alvik_ros_bridge` nel container Docker
- `alvik_pid_controller`
- **PlotJuggler** per la visualizzazione dei dati

### PlotJuggler — configurazione

1. **Streaming → ROS2 Topic Subscriber → Start**
2. Selezionare `/pid_debug` → Buffer 30sec
3. Trascinare i campi desiderati nel pannello grafico

---

<div align="center">  

<img src="CalibPIDschermataIniziale.png" width="58%">  

*Modalità calibrazione PID — schermata iniziale*  

</div>

<div style="page-break-before: always;"></div>

<div align="center">  

<img src="modalita_cal_PID_avviata.png" width="80%">  

*Modalità calibrazione PID avviata*  

</div>

---

### Legenda /pid_debug

| Campo | Descrizione | Unità |
|---|---|---|
| `data[0]` | x_target — posizione X obiettivo | cm |
| `data[1]` | x_real — posizione X attuale | cm |
| `data[2]` | y_target — posizione Y obiettivo | cm |
| `data[3]` | y_real — posizione Y attuale | cm |
| `data[4]` | theta_target — orientamento obiettivo | gradi |
| `data[5]` | theta_real — orientamento attuale | gradi |
| `data[6]` | vlin — riservato | — |
| `data[7]` | vang — riservato | — |

**Grafici utili:**
- `data[0]` vs `data[1]` — errore posizione X
- `data[2]` vs `data[3]` — errore posizione Y
- `data[4]` vs `data[5]` — errore angolare

### Test da alvik_shell

```bash
alvik_shell   # apri shell nel container

# Test lineare — vai avanti 25cm
ros2 topic pub --once /pid/command std_msgs/msg/String '{data: "goto_rel:25.0,0.0"}'

# Test angolare — ruota 90°
ros2 topic pub --once /pid/command std_msgs/msg/String '{data: "goto_rel:0.0,1.5708"}'

# Stop
ros2 topic pub --once /pid/command std_msgs/msg/String '{data: "stop"}'
```

### Parametri ottimali v1.7

```
kp_lin=1.5   ki_lin=0.8   kp_ang=25.0   ki_ang=0.0
acc=3.0 cm/s²   dec=2.0 cm/s²
vmax_lin=12.0 cm/s   vmax_ang=60.0 deg/s
```
---
<br>

## 7. Comandi di sistema

### Monitoraggio

```bash
# Log bridge in tempo reale
tail -f ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/log/$(ls -t ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/log/ | head -1)

# Stato container
docker ps --filter "name=alvik_ros2" --format "{{.Names}}\t{{.Status}}"

# Shell nel container ROS2
alvik_shell
```
<div style="page-break-before: always;"></div>

### Topic ROS2 utili (da alvik_shell)

```bash
# Posizione corrente
ros2 topic echo /odom --once | grep -A5 position

# Mappa ostacoli
ros2 topic echo /map --once | head -20

# Frequenza TF
ros2 topic hz /tf

# Lista nodi attivi
ros2 node list
```

---
<br>
<br>
<br>

<div align="center">  

<img src="plot_juggler.png" width="100%">  

*Test di calibrazione PID in esecuzione*  

</div>

<div style="page-break-before: always;"></div>

## 8. Risoluzione problemi

### Bridge non connesso

- Verificare che il PC sia connesso alla rete `Alvik_Robot_WiFi`
- Verificare che il bridge sia in esecuzione: `ps aux | grep bridge_v1_7`

### Alvik non connesso

- Verificare che Alvik sia acceso
- Verificare la connessione WiFi: `ping 192.168.4.1`
- Attendere qualche secondo — Alvik impiega ~5s ad avviarsi

### Container non trovato

```bash
# Verificare che il container sia attivo
docker ps --filter "name=alvik_ros2"

# Riavviare il container
cd ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7/ros2
docker-compose down && docker-compose up -d
```

### SLAM — mappa non si costruisce

- Verificare che Alvik si muova — la mappa richiede almeno 5cm di spostamento
- Verificare che `max_laser_range: 2.0` sia impostato in `slam_params.yaml`

### Nav2 — Alvik non raggiunge il goal

- Verificare che la posa iniziale sia corretta (Alvik nella posizione di inizio mappa)
- Eseguire Reset (`R`) e riprovare
- Verificare che la mappa sia stata salvata correttamente

### Salti nella navigazione Nav2

- Causati da latenza WiFi variabile (gap fino a 200ms)
- Il TF è pubblicato a 100Hz continuo per mitigare il problema
- `transform_tolerance: 0.5` in `nav2_params.yaml` compensa i gap

---

*Fine Manuale Utente — Alvik Digital Twin v1.7*
