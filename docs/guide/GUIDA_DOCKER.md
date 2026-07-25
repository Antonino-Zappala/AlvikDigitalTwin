<div align="center" style="margin-top: 200px;">

# Alvik Digital Twin v1.7

### DOCKER — BREVE GUIDA

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

# Guida Docker — Alvik Digital Twin v1.7

**Versione:** 1.7  
**Data:** Maggio 2026

---

## Indice

1. [Introduzione](#1-introduzione)
2. [Installazione](#2-installazione)
3. [Struttura del container](#3-struttura-del-container)
4. [Avvio e arresto](#4-avvio-e-arresto)
5. [Operazioni comuni](#5-operazioni-comuni)
6. [Aggiornamento del container](#6-aggiornamento-del-container)
7. [Risoluzione problemi](#7-risoluzione-problemi)

---

## 1. Introduzione

Il container Docker `alvik_ros2` nasce da una **necessità tecnica fondamentale**: il sistema operativo host è **Ubuntu 20.04**, mentre **ROS2 Humble richiede Ubuntu 22.04** come versione minima supportata. L'installazione nativa di ROS2 Humble su Ubuntu 20.04 non è possibile in modo ufficiale.

Docker risolve questo problema eseguendo un container Ubuntu 22.04 + ROS2 Humble all'interno dell'host Ubuntu 20.04, garantendo:
- **Compatibilità** — ROS2 Humble gira nel suo ambiente nativo
- **Isolamento** — nessun conflitto con le dipendenze dell'host
- **Riproducibilità** — l'ambiente è identico su qualsiasi macchina
- **Semplicità di aggiornamento** — rebuild del container per applicare modifiche

### Scelte progettuali

**Dalla v1.7** del progetto il container usa `docker-compose up -d` invece di `docker-compose run --rm`. Questo garantisce:
- **Nome fisso** `alvik_ros2` ad ogni avvio (invece di nomi casuali `ros2-alvik_ros2-run-XXXXXXXX`)
- **Daemon mode** — il container gira in background
- **Semplicità** — il bridge trova sempre il container con lo stesso nome

### Contenuto del container

| Componente | Versione |
|---|---|
| Ubuntu | 20.04 |
| ROS2 | Humble |
| slam_toolbox | ros-humble |
| nav2_bringup | ros-humble |
| plotjuggler-ros | ros-humble |
| alvik_bridge | package locale |

---

## 2. Installazione

### Prerequisiti

```bash
# Verifica Docker installato
docker --version
docker-compose --version
```

### Build dell'immagine (una tantum)

```bash
cd ~/Scrivania/progetto_alvik/ros2
docker-compose build --no-cache
```

> Il build richiede connessione internet e 10-15 minuti. Va ripetuto solo quando si modificano i file sorgente ROS2.

### Struttura directory

```
ros2/
├── Dockerfile
├── docker-compose.yml
├── config/                    ← volume persistente (mappe, parametri, RViz)
│   ├── alvik_map.pgm
│   ├── alvik_map.yaml
│   ├── alvik.rviz
│   ├── alvik_map.rviz
│   ├── alvik_nav.rviz
│   ├── slam_params.yaml
│   └── nav2_params.yaml
└── src/alvik_bridge/
    └── alvik_bridge/
        ├── alvik_ros_bridge.py
        ├── alvik_pid_controller.py
        ├── alvik_launch.py
        ├── slam_params.yaml
        └── nav2_params.yaml
```

### docker-compose.yml

```yaml
version: "3.3"
services:
  alvik_ros2:
    build: .
    container_name: alvik_ros2      # nome fisso — fondamentale per v1.7
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ./config:/ros2_ws/config    # persistenza mappe e parametri
    stdin_open: true
    tty: true
    command: bash
```

---
<br>

## 3. Struttura del container

### Percorsi importanti

| Percorso nel container | Descrizione |
|---|---|
| `/ros2_ws/install/alvik_bridge/` | Package installato |
| `/ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/` | Script Python |
| `/ros2_ws/install/alvik_bridge/share/alvik_bridge/` | File YAML e launch |
| `/ros2_ws/config/` | Mappe, RViz, parametri (volume persistente) |
| `/root/.ros/log/` | Log ROS2 |

### Nodi disponibili

```bash
# Da alvik_shell
ros2 run alvik_bridge alvik_ros_bridge      # bridge DDS → ROS2
ros2 run alvik_bridge alvik_pid_controller  # PID controller
ros2 launch alvik_bridge alvik_launch.py    # launch completo
```

---

<div style="page-break-after: always;"></div>

## 4. Avvio e arresto

### Avvio automatico (raccomandato)

```bash
cd ~/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7
./alvik_start_v1_7.sh
```

Lo script avvia automaticamente il container con `docker-compose up -d` e lancia `alvik_ros_bridge` al suo interno.

### Avvio manuale

```bash
cd ~/Scrivania/progetto_alvik/ros2

# Ferma eventuali container precedenti
docker-compose down

# Avvia il container in background
docker-compose up -d

# Verifica che sia attivo
docker ps --filter "name=alvik_ros2" --format "{{.Names}}\t{{.Status}}"

# Avvia alvik_ros_bridge nel container
docker exec -it alvik_ros2 bash -c '
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 run alvik_bridge alvik_ros_bridge
'
```

### Arresto

```bash
./alvik_stop_v1_7.sh
```

Oppure manualmente:

```bash
# Ferma alvik_ros_bridge dentro il container
docker exec alvik_ros2 bash -c "pkill -f alvik_ros_bridge"

# Ferma il container
cd ~/Scrivania/progetto_alvik/ros2
docker-compose down
```

> **Nota:** con `docker-compose up -d` il processo `alvik_ros_bridge` gira come daemon dentro il container. Il `pkill` dall'host **non funziona** — bisogna usare `docker exec` per terminarlo.

---
<br>

## 5. Operazioni comuni

### Aprire una shell nel container

```bash
alvik_shell
# equivalente a:
docker exec -it alvik_ros2 bash
```

### Verificare i nodi ROS2 attivi

```bash
alvik_shell
ros2 node list
```

### Verificare i topic

```bash
ros2 topic list
ros2 topic hz /tf          # frequenza TF (atteso ~100Hz)
ros2 topic echo /odom --once
```

### Copiare file dal container all'host

```bash
docker cp alvik_ros2:/ros2_ws/config/alvik_map.yaml \
  ~/Scrivania/progetto_alvik/ros2/config/alvik_map.yaml
```

### Copiare file dall'host al container

```bash
docker cp ~/Scrivania/progetto_alvik/ros2/src/alvik_bridge/alvik_bridge/alvik_ros_bridge.py \
  alvik_ros2:/ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/alvik_ros_bridge.py
```

<div style="page-break-after: always;"></div>

### Aggiornare un nodo senza rebuild

```bash
# 1. Modifica il file sorgente sull'host
# 2. Copia nel container
docker cp ~/Scrivania/progetto_alvik/ros2/src/alvik_bridge/alvik_bridge/alvik_ros_bridge.py \
  alvik_ros2:/ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/alvik_ros_bridge.py

# 3. Riavvia il nodo
docker exec alvik_ros2 bash -c "pkill -f alvik_ros_bridge"
```

---

## 6. Aggiornamento del container

### Modifica file Python — effetto immediato

```bash
# Copia il file modificato nel container
docker cp PERCORSO_FILE_HOST alvik_ros2:PERCORSO_FILE_CONTAINER

# Riavvia il nodo coinvolto
docker exec alvik_ros2 bash -c "pkill -f NOME_NODO"
```

### Modifica file YAML — effetto al prossimo avvio del nodo

```bash
docker cp ~/Scrivania/progetto_alvik/ros2/src/alvik_bridge/alvik_bridge/slam_params.yaml \
  alvik_ros2:/ros2_ws/install/alvik_bridge/share/alvik_bridge/slam_params.yaml
```

### Rendere permanenti le modifiche — rebuild

```bash
cd ~/Scrivania/progetto_alvik/ros2
docker-compose build --no-cache
```

> Dopo il rebuild le modifiche sono permanenti nell'immagine e non richiedono più `docker cp` ad ogni avvio.

<div style="page-break-after: always;"></div>

### Verifica parametri nell'immagine dopo rebuild

```bash
docker run --rm alvik_ros2 bash -c "
  echo '=== max_laser_range ==='
  grep max_laser_range /ros2_ws/install/alvik_bridge/share/alvik_bridge/slam_params.yaml
  echo '=== transform_tolerance ==='
  grep transform_tolerance /ros2_ws/install/alvik_bridge/share/alvik_bridge/nav2_params.yaml
  echo '=== BASE_WIDTH_M ==='
  grep BASE_WIDTH_M /ros2_ws/install/alvik_bridge/lib/python3.10/site-packages/alvik_bridge/alvik_ros_bridge.py | head -1
"
```

---
<br>

## 7. Risoluzione problemi

### Container non trovato

```bash
# Verificare se il container è attivo
docker ps --filter "name=alvik_ros2"

# Se non compare, avviarlo
cd ~/Scrivania/progetto_alvik/ros2
docker-compose up -d
```

### Errore "No such container: bash"

La variabile `$CONTAINER` è vuota o il container non è in esecuzione:

```bash
# Usare direttamente il nome fisso
docker exec -it alvik_ros2 bash
```

### X11 / RViz2 non si apre

```bash
# Abilitare X11 forwarding per Docker
xhost +local:docker
```
<div style="page-break-after: always;"></div>

### Porta già in uso

```bash
# Fermare tutti i container ROS2
docker-compose down
docker stop $(docker ps -q --filter "name=alvik_ros2")
```

### Mappa non trovata in Nav2

La mappa è salvata in `/ros2_ws/config/` che è un volume persistente. Verificare:

```bash
docker exec alvik_ros2 bash -c "ls /ros2_ws/config/"
```

---

*Fine Guida Docker — Alvik Digital Twin v1.7*
