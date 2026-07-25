"""
bridge_v1_7.py
==============
Middleware unificato — Alvik Digital Twin v1.7

Sostituisce bridge_DDS_v1_5.py + bridge_websocket_v1_6.py in un unico processo
Python con due moduli interni che girano in parallelo.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MODULO DDS — thread sincrono RT 50Hz
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IN   UDP :5005  ← Alvik          (CSV telemetria: x,y,theta,rpm,tof,tick)
  OUT  UDP :4444  → Godot          (DDS binario: variabili nominate)
  OUT  UDP :4445  → alvik_ros_bridge (stesso DDS binario per ROS2)
  OUT  UDP :5005  → Alvik          (comandi CMD,vlin,vang / RESET)
  IN   UDP :5006  ← alvik_ros_bridge (/cmd_vel da Nav2, futuro)
  IN   asyncio.Queue ← Modulo WS   (CMD da Godot via WebSocket)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
MODULO WEBSOCKET — asyncio event loop
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  IN/OUT WebSocket :8765 ↔ Godot

  Messaggi ricevuti da Godot:
    waypoints       → lista punti {x,y} in cm per path following
    stop            → ferma path following e Alvik
    reset           → reset odometria Alvik + azzeramento posa ROS2
    cmd             → velocità diretta {vlin, vang}
    pose            → aggiornamento posa corrente per path following
    alignment       → offset rotazione allineamento Digital Twin
    explorer_start  → avvia SLAM + PID controller (Esploratore)
    explorer_stop   → ferma Esploratore, salva mappa opzionale
    slam_start      → avvia SLAM standalone (senza PID)
    slam_stop       → ferma SLAM standalone, salva mappa opzionale
    nav2_start      → avvia localizzazione AMCL + Nav2 + posa iniziale
    nav2_stop       → ferma tutti i nodi Nav2
    nav2_goal       → pubblica goal di navigazione su /goal_pose

  Messaggi inviati a Godot:
    log             → messaggi di stato {level: ok/warn/error, msg}
    waypoint_reached → notifica raggiungimento waypoint {index, total}
    explorer_started/stopped
    slam_started/stopped
    nav2_started/stopped
    nav2_goal_reached/failed

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ARCHITETTURA INTERNA
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Alvik (WiFi)
    │  UDP 50Hz CSV
    ▼
  [Thread DDS] ──── asyncio.Queue ◄──── [asyncio WS handler]
    │                    (CMD)                │
    │  UDP DDS binario                        │  WebSocket JSON
    ▼                                         ▼
  Godot :4444                              Godot :8765
  ROS2  :4445

  [asyncio WS handler] ──── subprocess.Popen ──► Docker container
                                (slam, nav2, pid)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PATTERN TECNICI IMPORTANTI
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  asyncio.Queue — comunicazione thread-safe tra DDS (thread) e WS (asyncio).
    Il thread DDS chiama cmd_queue.get_nowait() in modo non bloccante.
    Il ws_handler chiama await cmd_queue.put() dal loop asyncio.

  run_in_executor + functools.partial — esecuzione di funzioni bloccanti
    (subprocess.run, proc.wait) senza congelare il loop asyncio.
    run_in_executor accetta solo callable senza argomenti, quindi usiamo
    partial(funzione, argomento) per "pre-inserire" i parametri.

  threading.Thread(daemon=True) — il thread DDS è daemon, si chiude
    automaticamente quando il processo principale termina.
"""

from functools import partial
import asyncio
import json
import math
import os
import select
import socket
import struct
import subprocess
import sys
import threading
import time

# ── Sistema di log su file ────────────────────────────────────────────────────
# Cartella /log nella directory del progetto — max 100MB totali
# Cancella i log più vecchi quando si supera il limite
LOG_DIR     = os.path.join(os.path.dirname(os.path.abspath(__file__)), "log")
LOG_MAX_MB  = 100  # dimensione massima cartella log in MB

def _init_log_file() -> object:
    """Crea la cartella log e apre il file di log con timestamp."""
    os.makedirs(LOG_DIR, exist_ok=True)
    _cleanup_old_logs()
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(LOG_DIR, f"bridge_{ts}.log")
    return open(path, "w", buffering=1, encoding="utf-8")

def _cleanup_old_logs():
    """Cancella i log più vecchi se la cartella supera LOG_MAX_MB."""
    try:
        files = sorted([
            os.path.join(LOG_DIR, f)
            for f in os.listdir(LOG_DIR)
            if f.startswith("bridge_") and f.endswith(".log")
        ])
        total = sum(os.path.getsize(f) for f in files)
        max_bytes = LOG_MAX_MB * 1024 * 1024
        while total > max_bytes and files:
            oldest = files.pop(0)
            total -= os.path.getsize(oldest)
            os.remove(oldest)
            print(f"[LOG] Rimosso log vecchio: {os.path.basename(oldest)}")
    except Exception as e:
        print(f"[LOG] Errore cleanup: {e}")

class _TeeOutput:
    """Redirige stdout sia sul terminale che sul file di log."""
    def __init__(self, file):
        self._terminal = sys.stdout
        self._file     = file
    def write(self, msg):
        self._terminal.write(msg)
        try:
            self._file.write(msg)
        except Exception:
            pass
    def flush(self):
        self._terminal.flush()
        try:
            self._file.flush()
        except Exception:
            pass

# Inizializza log su file all'avvio
_log_file  = _init_log_file()
sys.stdout = _TeeOutput(_log_file)
print(f"[LOG] Log su file: {_log_file.name}")
import websockets

# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURAZIONE
# ══════════════════════════════════════════════════════════════════════════════

ALVIK_IP          = "192.168.4.1"
ALVIK_LISTEN_PORT = 5005    # porta telemetria Alvik ← e comandi CMD →

GODOT_IP   = "127.0.0.1"
GODOT_PORT = 4444           # DDS binario verso Godot

ROS_IP   = "127.0.0.1"
ROS_PORT = 4445             # DDS binario verso alvik_ros_bridge

# Porta per ricevere /cmd_vel da Nav2 via alvik_ros_bridge [FUTURO]
ROS_CMD_PORT = 5006

WS_HOST = "0.0.0.0"
WS_PORT = 8765

TICK_HZ       = 50
TICK_INTERVAL = 1.0 / TICK_HZ    # 20ms per tick

# Se non arrivano dati da Alvik per più di STALE_TICKS tick (10 = 200ms),
# il bridge invia TickId=-1 a Godot/ROS2 per segnalare connessione persa
STALE_TICKS = 10

# Path salvataggio mappa dentro il container Docker
# /ros2_ws/maps/ è montato come volume → persiste tra sessioni
MAP_SAVE_PATH = "/ros2_ws/config/alvik_map"

# Path mappa per Nav2 (creata con SLAM o Esploratore, salvata in /config/)
MAP_NAV_PATH = "/ros2_ws/config/alvik_map.yaml"

# Configurazioni RViz2 — dentro il container Docker (/ros2_ws/config/ è volume)
# alvik.rviz     → Fixed Frame=odom, per odometria e SLAM
# alvik_map.rviz → Fixed Frame=map,  per Nav2 con mappa caricata
RVIZ_ODOM_CONFIG = "/ros2_ws/config/alvik.rviz"
RVIZ_SLAM_CONFIG = "/ros2_ws/config/alvik_map.rviz"
RVIZ_NAV_CONFIG  = "/ros2_ws/config/alvik_nav.rviz"

# ══════════════════════════════════════════════════════════════════════════════
# STATO GLOBALE CONDIVISO
# ══════════════════════════════════════════════════════════════════════════════

# Queue CMD: ws_handler mette i comandi, thread DDS li legge.
# Creata in main() perché deve appartenere all'event loop asyncio.
cmd_queue: asyncio.Queue = None

# Posa corrente di Alvik — aggiornata dal messaggio "pose" di Godot.
# Usata dal path following per calcolare direzione al prossimo waypoint.
current_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}

# ── Path following ────────────────────────────────────────────────────────────
waypoints: list = []     # lista di {x,y} in cm
wp_index:  int  = 0      # indice waypoint corrente
following: bool = False  # True quando il path following è attivo

# ── Client WebSocket connessi (in genere solo Godot) ──────────────────────────
ws_clients: set = set()

# ── Stato modalità ROS2 ───────────────────────────────────────────────────────
_explorer_active = False   # True quando Esploratore (SLAM+PID) è attivo
_slam_active     = False   # True quando SLAM standalone è attivo
_nav2_active     = False   # True quando Nav2 è attivo

# Dizionario processi ROS2 gestiti tramite subprocess.Popen
# Chiavi: "slam_nav2" = slam_toolbox+robot_state_publisher
#         "pid"       = alvik_pid_controller (solo Esploratore)
#         "loc"       = localization_launch (AMCL + map_server)
#         "nav2"      = navigation_launch (controller, planner, bt)
_ros_processes = {
    "slam_nav2": None,
    "pid":       None,
    "loc":       None,
    "nav2":      None,
    "rviz":      None,   # RViz2 — avviato con le modalità E/L/N
}

# Offset allineamento in radianti — inviato da Godot, pubblicato su /alvik/alignment
_align_rot_y: float = 0.0

# ══════════════════════════════════════════════════════════════════════════════
# MODULO DDS — costanti, socket e packing
# ══════════════════════════════════════════════════════════════════════════════

# Costanti protocollo DDS custom (compatibile con DDS_v1_5.gd in Godot)
CMD_PUBLISH    = 0x82    # byte comando: pubblica variabile
DDS_TYPE_INT   = 1       # tipo intero (4 byte signed little-endian)
DDS_TYPE_FLOAT = 2       # tipo float (4 byte IEEE 754 little-endian)

# Fattore di conversione RPM → rad/s: RPM * (2π / 60)
RPM_TO_RADS = 2.0 * math.pi / 60.0

# Socket principale: riceve telemetria CSV da Alvik, invia comandi
sock_alvik = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_alvik.bind(("0.0.0.0", ALVIK_LISTEN_PORT))
sock_alvik.setblocking(False)  # non bloccante per il loop DDS RT

# Socket output verso Godot e alvik_ros_bridge
sock_godot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_ros   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Socket per ricevere /cmd_vel da Nav2 [FUTURO]
sock_ros_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_ros_cmd.bind(("0.0.0.0", ROS_CMD_PORT))
sock_ros_cmd.setblocking(False)

# ── DDS packing ───────────────────────────────────────────────────────────────
# Formato pacchetto DDS: [CMD(1B)] [TYPE(1B)] [name_len(1B)] [name(nB)] [value(4B)]
# Interpretato da DDS_v1_5.gd in Godot e da alvik_ros_bridge.py

def _pack_float(name: str, value: float) -> bytes:
    """Impacchetta una variabile DDS di tipo float (4 byte IEEE 754 LE)."""
    nb = name.encode()
    return struct.pack(f"<BBB{len(nb)}sf",
                       CMD_PUBLISH, DDS_TYPE_FLOAT, len(nb), nb, float(value))

def _pack_int(name: str, value: int) -> bytes:
    """Impacchetta una variabile DDS di tipo int (4 byte signed LE)."""
    nb = name.encode()
    return struct.pack(f"<BBB{len(nb)}si",
                       CMD_PUBLISH, DDS_TYPE_INT, len(nb), nb, int(value))

def _send_dds(sock: socket.socket, addr: tuple, name: str, value, is_int=False):
    """Invia un pacchetto DDS a un indirizzo specificato."""
    pkt = _pack_int(name, value) if is_int else _pack_float(name, value)
    sock.sendto(pkt, addr)

def _broadcast(name: str, value, is_int=False):
    """Invia la variabile DDS a Godot e alvik_ros_bridge simultaneamente."""
    _send_dds(sock_godot, (GODOT_IP, GODOT_PORT), name, value, is_int)
    _send_dds(sock_ros,   (ROS_IP,   ROS_PORT),   name, value, is_int)

# ── Parsing telemetria CSV da Alvik ───────────────────────────────────────────

def parse_alvik(raw: str):
    """
    Parsa la stringa CSV inviata da Alvik ogni 20ms (50Hz).

    Formato (12 campi):
      x, y, theta_rad, wl_rpm, wr_rpm,
      tof_L, tof_CL, tof_C, tof_CR, tof_R,
      tick_id, t_robot_ms

    x,y in cm (odometria encoder), theta in rad.
    tof_* in cm (5 sensori frontali, distanza max fisica 15cm).
    Restituisce None se malformato.
    """
    parts = raw.strip().split(',')
    if len(parts) < 12:
        return None
    try:
        return {
            "x":          float(parts[0]),
            "y":          float(parts[1]),
            "theta":      float(parts[2]),
            "wl_rpm":     float(parts[3]),
            "wr_rpm":     float(parts[4]),
            "tof_L":      float(parts[5]),
            "tof_CL":     float(parts[6]),
            "tof_C":      float(parts[7]),
            "tof_CR":     float(parts[8]),
            "tof_R":      float(parts[9]),
            "tick_id":    int(parts[10]),
            "t_robot_ms": int(parts[11]),
        }
    except ValueError as e:
        print(f"[PARSE] Errore: {e} — {raw!r}")
        return None

# ── Comandi verso Alvik ───────────────────────────────────────────────────────

def send_cmd_alvik(vlin: float, vang: float):
    """Invia comando velocità ad Alvik. vlin=cm/s, vang=deg/s."""
    msg = f"CMD,{vlin:.2f},{vang:.2f}".encode()
    sock_alvik.sendto(msg, (ALVIK_IP, ALVIK_LISTEN_PORT))

def send_stop_alvik():
    """Invia velocità zero ad Alvik (stop immediato)."""
    send_cmd_alvik(0.0, 0.0)

def send_reset_alvik():
    """
    Invia RESET ad Alvik: azzera x, y, theta nell'odometria del firmware.
    Usato prima di iniziare una nuova sessione di navigazione.
    """
    try:
        sock_alvik.sendto(b"RESET", (ALVIK_IP, ALVIK_LISTEN_PORT))
        print("[CMD] Reset inviato")
    except Exception as e:
        print(f"[CMD] Errore reset: {e}")

# ══════════════════════════════════════════════════════════════════════════════
# MODULO DDS — loop RT 50Hz in thread separato
# ══════════════════════════════════════════════════════════════════════════════

def dds_loop(loop: asyncio.AbstractEventLoop):
    """
    Loop DDS sincrono a 50Hz, eseguito in un thread daemon separato.

    Perché un thread e non asyncio?
    Il loop DDS richiede timing preciso (20ms per tick). Un thread dedicato
    con time.sleep() garantisce la precisione necessaria senza cedere il
    controllo all'event loop asyncio.

    Ciclo per ogni tick (20ms):
      1. Drain telemetria UDP da Alvik (non bloccante, select timeout=0)
      2. Drain comandi /cmd_vel da Nav2 porta 5006 [FUTURO]
      3. Drain CMD dalla asyncio.Queue (comandi da WebSocket)
      4. Broadcast tutte le variabili DDS a Godot e ROS2
      5. Sleep preciso per mantenere 50Hz
    """
    print("[DDS] Thread avviato")
    tick_id   = 0
    last_pose = None
    t0        = time.monotonic()

    while True:
        loop_start = time.monotonic()
        tick_id   += 1

        # ── 1. Drain telemetria da Alvik ──────────────────────────────────────
        # select() con timeout=0 è non bloccante. In caso di burst mantiene
        # solo l'ultimo pacchetto (il più recente).
        while True:
            ready, _, _ = select.select([sock_alvik], [], [], 0)
            if not ready:
                break
            try:
                data, addr = sock_alvik.recvfrom(256)
                parsed = parse_alvik(data.decode())
                if parsed is not None:
                    last_pose = parsed
                    last_pose["rx_tick"] = tick_id
            except Exception as e:
                print(f"[DDS] Errore ricezione Alvik: {e}")

        # ── 2. Drain CMD da alvik_ros_bridge (/cmd_vel Nav2) [FUTURO] ─────────
        while True:
            ready, _, _ = select.select([sock_ros_cmd], [], [], 0)
            if not ready:
                break
            try:
                data, _ = sock_ros_cmd.recvfrom(64)
                msg = data.decode().strip()
                if msg.startswith("CMD,"):
                    parts = msg.split(',')
                    vlin = float(parts[1])
                    vang = float(parts[2])
                    send_cmd_alvik(vlin, vang)
                    print(f"[DDS] /cmd_vel Nav2: vlin={vlin:.1f} vang={vang:.1f}")
            except Exception as e:
                print(f"[DDS] Errore ricezione ROS CMD: {e}")

        # ── 3. Drain CMD dalla asyncio.Queue ──────────────────────────────────
        # get_nowait() è thread-safe perché non modifica il loop asyncio.
        # Lancia QueueEmpty se vuota — normale, catturato da except.
        try:
            while True:
                cmd = cmd_queue.get_nowait()
                if cmd.get("type") == "cmd":
                    send_cmd_alvik(cmd["vlin"], cmd["vang"])
                elif cmd.get("type") == "reset":
                    send_reset_alvik()
        except Exception:
            pass  # Queue vuota — normale

        # ── 4. Broadcast dati a Godot e alvik_ros_bridge ─────────────────────
        if last_pose is None:
            # Nessun dato ancora ricevuto — segnala a Godot
            _broadcast("TickId", -1, is_int=True)
        else:
            age = tick_id - last_pose["rx_tick"]
            if age > STALE_TICKS:
                # Dato stale (>200ms senza dati) — segnala connessione persa
                _broadcast("TickId", -1, is_int=True)
                print(f"[DDS] STALE — nessun dato da {age} tick")
            else:
                p = last_pose
                wl_rads = p["wl_rpm"] * RPM_TO_RADS
                wr_rads = p["wr_rpm"] * RPM_TO_RADS

                # Broadcast tutte le variabili — lette da Godot con DDS.read()
                _broadcast("X",          p["x"])
                _broadcast("Y",          p["y"])
                _broadcast("Theta",      p["theta"])
                _broadcast("WheelLeft",  wl_rads)
                _broadcast("WheelRight", wr_rads)
                _broadcast("ToF_L",      p["tof_L"])
                _broadcast("ToF_CL",     p["tof_CL"])
                _broadcast("ToF_C",      p["tof_C"])
                _broadcast("ToF_CR",     p["tof_CR"])
                _broadcast("ToF_R",      p["tof_R"])
                _broadcast("t_robot_ms", p["t_robot_ms"], is_int=True)
                _broadcast("TickId",     tick_id, is_int=True)

                # Log di stato ogni secondo
                if tick_id % TICK_HZ == 0:
                    elapsed = time.monotonic() - t0
                    print(
                        f"[DDS] [{elapsed:6.1f}s] tick={tick_id} "
                        f"pos=({p['x']:.1f},{p['y']:.1f},{p['theta']:.2f}rad) "
                        f"tof=[{p['tof_L']:.0f}|{p['tof_CL']:.0f}|"
                        f"{p['tof_C']:.0f}|{p['tof_CR']:.0f}|{p['tof_R']:.0f}]cm"
                    )

        # ── 5. Sleep preciso per mantenere 50Hz ───────────────────────────────
        elapsed = time.monotonic() - loop_start
        sleep   = TICK_INTERVAL - elapsed
        if sleep > 0:
            time.sleep(sleep)

# ══════════════════════════════════════════════════════════════════════════════
# GESTIONE CONTAINER DOCKER
# ══════════════════════════════════════════════════════════════════════════════

# Nome fisso del container Docker — impostato da docker-compose up
CONTAINER_NAME = "alvik_ros2"

def get_container_name() -> str:
    """
    Restituisce il nome fisso del container ROS2 (alvik_ros2).
    Verifica che il container sia in esecuzione prima di restituirlo.
    """
    try:
        result = subprocess.run(
            ["docker", "ps", "--filter", f"name={CONTAINER_NAME}",
             "--format", "{{.Names}}"],
            capture_output=True, text=True, timeout=5
        )
        names = result.stdout.strip().split('\n')
        return CONTAINER_NAME if names and names[0] else ""
    except Exception as e:
        print(f"[DOCKER] Errore get_container: {e}")
        return ""

def _ros2_exec(container: str, cmd: str) -> subprocess.Popen:
    """
    Avvia un comando ROS2 nel container con docker exec (asincrono).
    Fa il source di setup.bash e install/setup.bash prima del comando.
    Restituisce il Popen del processo — non aspetta il completamento.
    """
    return subprocess.Popen([
        "docker", "exec", container, "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        "source /ros2_ws/install/setup.bash && "
        + cmd
    ])

def _ros2_run(container: str, cmd: str, timeout: int = 10):
    """
    Esegue un comando ROS2 nel container e ASPETTA il completamento.
    Usato per comandi brevi come la pubblicazione della posa iniziale.
    """
    subprocess.run([
        "docker", "exec", container, "bash", "-c",
        "source /opt/ros/humble/setup.bash && "
        "source /ros2_ws/install/setup.bash && "
        + cmd
    ], timeout=timeout)

def _save_map(container: str):
    """
    Salva la mappa SLAM corrente tramite il servizio ROS2 slam_toolbox/save_map.
    La mappa viene salvata in /ros2_ws/config/alvik_map (.pgm + .yaml).
    Attende fino a 10s che il servizio risponda.
    """
    """
    Salva la mappa corrente di slam_toolbox tramite il servizio ROS2.
    Crea MAP_SAVE_PATH.pgm (immagine) e MAP_SAVE_PATH.yaml (metadati).
    Il volume Docker /ros2_ws/ è persistente tra le sessioni.
    """
    print("[SLAM] Salvataggio mappa...")
    try:
        subprocess.run([
            "docker", "exec", container, "bash", "-c",
            "mkdir -p /ros2_ws/maps"
        ], timeout=5)
        subprocess.run([
            "docker", "exec", container, "bash", "-c",
            f"source /opt/ros/humble/setup.bash && "
            f"ros2 service call /slam_toolbox/save_map "
            f"slam_toolbox/srv/SaveMap "
            f"\"{{name: {{data: '{MAP_SAVE_PATH}'}}}}\""
        ], timeout=15)
        print(f"[SLAM] Mappa salvata in {MAP_SAVE_PATH}")
    except Exception as e:
        print(f"[SLAM] Errore salvataggio mappa: {e}")

def _kill_processes(container: str, patterns: list):
    """
    Termina i processi nel container Docker che corrispondono ai pattern.
    Usa pkill -f per match su nome/argomenti del processo.
    """
    """
    Termina i processi nel container che corrispondono ai pattern (pkill fallback).
    Usato dopo _stop_process() per garantire che i nodi ROS2 siano terminati.
    """
    pkill_cmd = " ; ".join([f"pkill -f '{p}'" for p in patterns])
    try:
        subprocess.run([
            "docker", "exec", container, "bash", "-c", pkill_cmd
        ], timeout=5)
    except Exception as e:
        print(f"[DOCKER] Errore pkill: {e}")

def _stop_process(name: str):
    """
    Termina un processo locale per nome (pkill -f).
    Usato per fermare RViz2 e altri processi host.
    """
    """
    Termina un processo da _ros_processes in modo sicuro.
    Tenta terminate() (SIGTERM, graceful) poi kill() (SIGKILL, forzato).
    """
    proc = _ros_processes.get(name)
    if proc is not None:
        try:
            proc.terminate()
            proc.wait(timeout=5)
            print(f"[DOCKER] Processo '{name}' fermato")
        except Exception as e:
            print(f"[DOCKER] Errore stop '{name}': {e}")
            try:
                proc.kill()
            except:
                pass
        _ros_processes[name] = None

def _start_terminal(title: str, container: str, cmd: str) -> subprocess.Popen:
    """
    Apre un terminale x-terminal-emulator con docker exec per visualizzare
    l'output del nodo ROS2 in una finestra separata.
    Restituisce il Popen del terminale per poterlo terminare in seguito.
    """
    """
    Avvia un comando ROS2 nel container in un terminale gnome-terminal visibile.
    Utile per i nodi che producono log importanti (PID, slam_toolbox).
    Restituisce il Popen del terminale (non del processo ROS2 interno).

    title   — titolo della finestra terminale
    container — nome del container Docker
    cmd     — comando ROS2 da eseguire (senza source setup.bash)
    """
    full_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source /ros2_ws/install/setup.bash && "
        + cmd
    )
    try:
        proc = subprocess.Popen([
            "gnome-terminal", f"--title={title}", "--",
            "docker", "exec", "-it", container, "bash", "-c", full_cmd
        ])
        print(f"[TERM] Terminale '{title}' aperto")
        return proc
    except Exception as e:
        print(f"[TERM] Errore apertura terminale '{title}': {e}")
        # Fallback: avvia senza terminale (invisibile ma funzionante)
        return _ros2_exec(container, cmd)
    """
    Avvia RViz2 nel container Docker con la configurazione specificata.
    Viene chiamato da explorer_start(), slam_start() e nav2_start().

    Non usa gnome-terminal (il bridge non ha sessione DBUS desktop).
    Usa subprocess.Popen con docker exec direttamente — RViz viene
    visualizzato sul display :0 dell'utente grazie a xhost +local:docker
    che deve essere stato eseguito da alvik_start_v1_7.sh.

    config — path del file .rviz dentro il container:
      RVIZ_SLAM_CONFIG → Fixed Frame=map  (per Esploratore/SLAM)
      RVIZ_NAV_CONFIG  → Fixed Frame=map  (per Nav2 con mappa salvata)
    """
    print(f"[RVIZ] Avvio RViz2 con config: {config}")
    try:
        import os
        # Abilita X11 forwarding per Docker
        subprocess.run(["xhost", "+local:docker"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        display = os.environ.get("DISPLAY", ":0")
        xauth   = os.environ.get("XAUTHORITY",
                                  os.path.expanduser("~/.Xauthority"))

        # Lancia RViz2 in un gnome-terminal separato tramite docker exec.
        # Usiamo gnome-terminal perché ha la propria sessione desktop con
        # accesso al display X11 — docker exec diretto dal bridge non ha
        # accesso al display anche con DISPLAY e XAUTHORITY passati.
        _ros_processes["rviz"] = subprocess.Popen([
            "gnome-terminal",
            "--title=RViz2", "--",
            "bash", "-c",
            f"DISPLAY={display} XAUTHORITY={xauth} "
            f"docker exec -it {container} bash -c "
            f"'source /opt/ros/humble/setup.bash && "
            f"DISPLAY={display} rviz2 -d {config}'; "
            f"exec bash"
        ])
        print(f"[RVIZ] RViz2 avviato (DISPLAY={display})")
    except Exception as e:
        print(f"[RVIZ] Errore avvio RViz2: {e}")

def _stop_rviz():
    """Ferma tutte le istanze di RViz2 in esecuzione sull'host."""
    """
    Termina RViz2 tramite _stop_process() (SIGTERM sul gnome-terminal)
    e pkill dentro il container per terminare il processo rviz2.
    Chiamato da explorer_stop(), slam_stop() e nav2_stop().
    """
    _stop_process("rviz")
    container = get_container_name()
    if container:
        _kill_processes(container, ["rviz2"])
    print("[RVIZ] RViz2 terminato")

def _start_rviz(container: str, config: str):
    """
    Avvia RViz2 nel container con la configurazione specificata.
    Attende che il file di config sia disponibile prima di avviare.
    """
    """
    Avvia RViz2 nel container Docker in un gnome-terminal separato.
    Passa DISPLAY e XAUTHORITY al terminale per l'accesso al display X11.

    config — path del file .rviz dentro il container:
      RVIZ_SLAM_CONFIG → Fixed Frame=map  (per Esploratore/SLAM)
      RVIZ_NAV_CONFIG  → Fixed Frame=map  (per Nav2 con mappa salvata)
    """
    import os
    print(f"[RVIZ] Avvio RViz2 con config: {config}")
    try:
        subprocess.run(["xhost", "+local:docker"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        display = os.environ.get("DISPLAY", ":0")
        xauth   = os.environ.get("XAUTHORITY",
                                  os.path.expanduser("~/.Xauthority"))

        _ros_processes["rviz"] = subprocess.Popen([
            "gnome-terminal",
            "--title=RViz2", "--",
            "bash", "-c",
            f"DISPLAY={display} XAUTHORITY={xauth} "
            f"docker exec -it {container} bash -c "
            f"'source /opt/ros/humble/setup.bash && "
            f"DISPLAY={display} rviz2 -d {config}'; "
            f"exec bash"
        ])
        print(f"[RVIZ] RViz2 avviato (DISPLAY={display})")
    except Exception as e:
        print(f"[RVIZ] Errore avvio RViz2: {e}")

# ══════════════════════════════════════════════════════════════════════════════
# MODALITÀ ESPLORATORE (SLAM + PID)
# ══════════════════════════════════════════════════════════════════════════════
# Avvia slam_toolbox + alvik_pid_controller nel container Docker.
# Il PID gestisce il path following verso i waypoint selezionati in Godot,
# mentre slam_toolbox costruisce la mappa in background (odometria pura,
# use_scan_matching: false — i ToF sono troppo pochi e corti per correggere).

def explorer_start() -> bool:
    """
    Avvia la modalità Esploratore: slam_toolbox + alvik_pid_controller + RViz2.
    Restituisce True se l'avvio ha successo, False in caso di errore.
    Eseguita in run_in_executor per non bloccare il loop asyncio.
    """
    """
    Avvia la modalità Esploratore:
      1. alvik_launch.py slam:=true → robot_state_publisher + slam_toolbox
      2. attesa 10s per inizializzazione SLAM
      3. alvik_pid_controller → path following verso waypoint
    """
    global _explorer_active
    container = get_container_name()
    if not container:
        print("[EXPLORER] Container non trovato")
        return False
    print(f"[EXPLORER] Avvio su container: {container}")
    _ros_processes["slam_nav2"] = _ros2_exec(
        container,
        "ros2 launch alvik_bridge alvik_launch.py slam:=true"
    )
    print("[EXPLORER] Attendo inizializzazione SLAM (10s)...")
    time.sleep(10)
    # Avvia PID in un terminale visibile per monitorare i log
    _ros_processes["pid"] = _start_terminal(
        "PID Controller",
        container,
        "ros2 run alvik_bridge alvik_pid_controller"
    )
    print(f"[EXPLORER] PID avviato")
    _start_rviz(container, RVIZ_SLAM_CONFIG)  # RViz con Fixed Frame=map (SLAM)
    _explorer_active = True
    print("[EXPLORER] Modalità Esploratore avviata")
    return True

def explorer_stop(save_map: bool = True) -> bool:
    """
    Ferma la modalità Esploratore: termina slam_toolbox, PID controller e RViz2.
    Se save_map=True salva la mappa prima di fermare slam_toolbox.
    """
    """
    Ferma la modalità Esploratore.
    save_map=True → salva la mappa prima di terminare i processi.
    """
    global _explorer_active
    container = get_container_name()
    if save_map and container:
        _save_map(container)
    else:
        print("[EXPLORER] Mappa non salvata per scelta utente")
    send_stop_alvik()
    # Termina i Popen lato host
    _stop_process("slam_nav2")
    _stop_process("pid")
    # pkill dentro il container — ferma i processi ROS2 residui
    if container:
        _kill_processes(container, [
            "slam_toolbox", "alvik_pid_controller",
            "alvik_launch", "robot_state_publisher"
        ])
    # Chiude il gnome-terminal del PID controller
    import subprocess as sp
    sp.run(["pkill", "-f", "PID Controller"], capture_output=True)
    sp.run(["pkill", "-f", "alvik_pid_controller"], capture_output=True)
    _stop_rviz()
    _explorer_active = False
    print("[EXPLORER] Modalità Esploratore terminata")
    return True

# ══════════════════════════════════════════════════════════════════════════════
# MODALITÀ SLAM STANDALONE
# ══════════════════════════════════════════════════════════════════════════════
# SLAM senza PID: avvia solo slam_toolbox + robot_state_publisher.
# Alvik viene guidato manualmente con teleop (W/X/Q/E da Godot).
# Utile per creare mappe accurate da usare poi con Nav2.
#
# Differenza con Esploratore:
#   SLAM standalone → teleop manuale, nessun PID, solo mappatura
#   Esploratore     → waypoint automatici, PID attivo, SLAM in background

def slam_start() -> bool:
    """
    Avvia la modalità SLAM standalone: solo slam_toolbox + RViz2, senza PID.
    L'utente controlla Alvik manualmente con il Teleop per costruire la mappa.
    """
    """
    Avvia SLAM standalone (slam_toolbox + robot_state_publisher, senza PID).
    Alvik deve essere guidato manualmente con la modalità Teleop di Godot.
    """
    global _slam_active
    container = get_container_name()
    if not container:
        print("[SLAM] Container non trovato")
        return False
    print(f"[SLAM] Avvio SLAM standalone su container: {container}")
    _ros_processes["slam_nav2"] = _ros2_exec(
        container,
        "ros2 launch alvik_bridge alvik_launch.py slam:=false"
    )
    print("[SLAM] Attendo inizializzazione (8s)...")
    time.sleep(8)
    _start_rviz(container, RVIZ_SLAM_CONFIG)  # RViz con Fixed Frame=map (SLAM)
    _slam_active = True
    print("[SLAM] SLAM standalone avviato — usa teleop per mappare")
    return True

def slam_stop(save_map: bool = True) -> bool:
    """
    Ferma la modalità SLAM standalone.
    Se save_map=True salva la mappa prima di terminare slam_toolbox.
    """
    """
    Ferma SLAM standalone e opzionalmente salva la mappa.
    """
    global _slam_active
    container = get_container_name()
    if save_map and container:
        _save_map(container)
    else:
        print("[SLAM] Mappa non salvata per scelta utente")
    send_stop_alvik()
    _stop_process("slam_nav2")
    if container:
        _kill_processes(container, [
            "slam_toolbox", "alvik_launch", "robot_state_publisher"
        ])
    _stop_rviz()
    _slam_active = False
    print("[SLAM] SLAM standalone terminato")
    return True

# ══════════════════════════════════════════════════════════════════════════════
# MODALITÀ NAV2
# ══════════════════════════════════════════════════════════════════════════════
# Navigazione autonoma su mappa pre-esistente (creata con SLAM o Esploratore).
#
# Sequenza di avvio:
#   1. alvik_launch.py localization:=true → robot_state_pub + map_server + AMCL
#   2. attesa 10s per inizializzazione localizzazione
#   3. pubblicazione posa iniziale (0, 0, π) su /initialpose (20 messaggi)
#      — θ=π perché alvik_ros_bridge applica theta+π all'odometria
#   4. navigation_launch.py → controller_server, planner_server, bt_navigator
#
# Conversione coordinate goal (fatta in Godot, selezione_waypoint_v1_7.gd):
#   ROS2_x = -Godot_z    (Z Godot → X ROS2 negato)
#   ROS2_y = -Godot_x    (X Godot → Y ROS2 negato)

def nav2_start() -> bool:
    """
    Avvia la navigazione autonoma Nav2:
      1. Avvia nav2_bringup con la mappa salvata e i parametri di configurazione
      2. Attende che Nav2 sia pronto (lifecycle manager attivo)
      3. Pubblica la posa iniziale normalizzata su /initialpose per AMCL
      4. Avvia RViz2 con la configurazione di navigazione
    Restituisce True se l'avvio ha successo.
    """
    """
    Avvia la navigazione autonoma Nav2.
    Richiede la mappa in MAP_NAV_PATH (creata con SLAM o Esploratore).
    """
    global _nav2_active
    container = get_container_name()
    if not container:
        print("[NAV2] Container non trovato")
        return False
    print(f"[NAV2] Avvio su container: {container}")

    # Cleanup preventivo — termina eventuali nodi Nav2 residui
    print("[NAV2] Cleanup nodi residui...")
    _kill_processes(container, [
        "nav2_bringup", "controller_server", "planner_server",
        "bt_navigator", "amcl", "map_server", "behavior_server",
        "robot_state_publisher", "lifecycle_manager"
    ])
    time.sleep(2)
    # Step 1: avvia tutto Nav2 con bringup_launch (AMCL + navigation + behavior)
    # Un singolo launch evita il conflitto del behavior_server
    _ros_processes["loc"] = _ros2_exec(
        container,
        f"ros2 launch nav2_bringup bringup_launch.py "
        f"map:={MAP_NAV_PATH} "
        f"params_file:=/ros2_ws/install/alvik_bridge/share/alvik_bridge/nav2_params.yaml"
    )
    print("[NAV2] Attendo inizializzazione Nav2 (8s)...")
    time.sleep(8)

    # Step 2: posa iniziale — 20 messaggi per sicurezza (AMCL potrebbe
    # non ricevere il primo se non è ancora completamente pronto)
    # Usa la posa corrente di Alvik (aggiornata da Godot tramite messaggio "pose")
    # Il quaternione corrisponde a theta corrente con offset +π
    px    = current_pose["x"] / 100.0   # cm → metri
    py    = current_pose["y"] / 100.0
    theta = current_pose["theta"] + math.pi
    qz    = math.sin(theta / 2.0)
    qw    = math.cos(theta / 2.0)
    print(f"[NAV2] Pubblicazione posa iniziale ({px:.2f}, {py:.2f}, {math.degrees(theta):.1f}°)...")
    try:
        for _ in range(20):
            _ros2_run(container,
                f"ros2 topic pub --once /initialpose "
                f"geometry_msgs/msg/PoseWithCovarianceStamped "
                f"'{{header: {{frame_id: map}}, "
                f"pose: {{pose: {{position: {{x: {px:.3f}, y: {py:.3f}, z: 0.0}}, "
                f"orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}}}}}}}'",
                timeout=3
            )
            time.sleep(0.1)
        print("[NAV2] Posa iniziale pubblicata")
    except Exception as e:
        print(f"[NAV2] Errore pubblicazione posa iniziale: {e}")

    # Step 3: navigation già avviata da bringup_launch nel Step 1

    _start_rviz(container, RVIZ_NAV_CONFIG)  # RViz con Fixed Frame=map
    _nav2_active = True
    print("[NAV2] Nav2 pronto — invia goal da Godot")
    return True

def nav2_stop() -> bool:
    """
    Ferma tutti i processi Nav2 nel container e RViz2 sull'host.
    """
    """Ferma tutti i nodi Nav2 in modo ordinato."""
    global _nav2_active
    container = get_container_name()
    # Prima ferma i nodi Nav2 che mandano comandi su /cmd_vel
    _stop_process("nav2")
    _stop_process("loc")
    _stop_process("slam_nav2")
    if container:
        _kill_processes(container, [
            "nav2_bringup", "controller_server", "planner_server",
            "bt_navigator", "amcl", "map_server", "robot_state_publisher"
        ])
    # Poi ferma Alvik — ora Nav2 non manda più comandi
    send_stop_alvik()
    time.sleep(0.5)
    send_stop_alvik()  # doppio stop per sicurezza
    _stop_rviz()
    _nav2_active = False
    print("[NAV2] Nav2 terminato")
    return True

def nav2_send_goal(container: str, x: float, y: float, theta: float = 0.0):
    """
    Invia un goal di navigazione a Nav2 tramite ros2 action send_goal.
    Coordinate in metri nel frame 'map' (sistema di riferimento ROS2).
    Usato invece di topic pub perché l'action gestisce automaticamente
    il timestamp e il feedback — topic pub inviava timestamp=0 ignorato da bt_navigator.
    """
    """
    Pubblica un goal di navigazione su /goal_pose tramite ros2 topic pub.

    x, y in metri nel frame 'map' di ROS2.
    La conversione Godot→ROS2 è già stata fatta in selezione_waypoint_v1_7.gd:
      ros_x = -point.z, ros_y = -point.x

    Il quaternione per theta: z=sin(theta/2), w=cos(theta/2).
    """
    # Se theta è None usa orientamento identità (w=1) — Nav2 non ruoterà
    # alla fine ma manterrà l'orientamento raggiunto durante il percorso
    if theta is None:
        qz, qw = 0.0, 1.0
    else:
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
    try:
        _ros2_run(container,
            f"ros2 action send_goal /navigate_to_pose "
            f"nav2_msgs/action/NavigateToPose "
            f"'{{pose: {{header: {{frame_id: map}}, "
            f"pose: {{position: {{x: {x:.3f}, y: {y:.3f}, z: 0.0}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {qz:.4f}, w: {qw:.4f}}}}}}}}}'",
            timeout=5
        )
        print(f"[NAV2] Goal inviato: ({x:.2f}, {y:.2f}) m, theta={theta:.2f} rad")
    except Exception as e:
        print(f"[NAV2] Errore invio goal: {e}")

# ══════════════════════════════════════════════════════════════════════════════
# ALLINEAMENTO
# ══════════════════════════════════════════════════════════════════════════════

def publish_alignment(rot_y: float):
    """
    Pubblica l'offset di allineamento su /initialpose per aggiornare
    la stima di posizione di AMCL dopo un allineamento del Digital Twin.
    rot_y: rotazione in radianti (asse Z in ROS2).
    Nota: non usato in v1.7 — l'allineamento Godot è indipendente da ROS2.
    """
    """
    Pubblica l'offset di allineamento su /alvik/alignment nel container ROS2.
    alvik_ros_bridge sottoscrive e applica la rotazione all'odometria /odom.
    rot_y in radianti.
    """
    container = get_container_name()
    if container:
        subprocess.Popen([
            "docker", "exec", container, "bash", "-c",
            f"source /opt/ros/humble/setup.bash && "
            f"ros2 topic pub --once /alvik/alignment "
            f"std_msgs/msg/Float32 '{{data: {rot_y}}}'"
        ])

# ══════════════════════════════════════════════════════════════════════════════
# MODULO PATH FOLLOWING
# ══════════════════════════════════════════════════════════════════════════════
# Controllo proporzionale per seguire la lista di waypoint in modalità
# WAYPOINT e PATH. Gira come coroutine asyncio a 20Hz.
#
# Algoritmo per ogni tick:
#   1. Calcola distanza e angolo verso il waypoint corrente
#   2. Se dist < soglia → raggiunto, passa al successivo
#   3. Se errore angolare > soglia → ruota sul posto (vlin=0)
#   4. Altrimenti → avanza proporzionalmente + correzione angolare

WAYPOINT_REACH_CM = 3.0     # cm per considerare il waypoint raggiunto
ANGLE_THRESH_RAD  = 0.15    # rad (≈8.6°) sotto cui si avanza
MAX_VLIN          = 10.0    # cm/s velocità lineare massima
MAX_VANG          = 60.0    # deg/s velocità angolare massima
KP_ANGLE          = 80.0    # guadagno proporzionale angolare
CMD_HZ            = 20      # frequenza comandi path following
CMD_INTERVAL      = 1.0 / CMD_HZ

def compute_command(pose: dict, target: dict):
    """
    Controller proporzionale per il path following del bridge.
    Calcola (vlin cm/s, vang deg/s, dist cm) per raggiungere il target.
    Logica:
      - dist < WAYPOINT_REACH_CM → waypoint raggiunto, restituisce (0,0,dist)
      - |angle_err| > ANGLE_THRESH_RAD → solo rotazione (vlin=0)
      - altrimenti → avanza con correzione angolare
    Distinto dal PID controller ROS2 — più semplice, senza profilo trapezoidale.
    """
    """
    Calcola (vlin, vang, dist) per raggiungere target dalla posa corrente.
    Restituisce (0,0,dist) se il target è già raggiunto.
    """
    dx   = target["x"] - pose["x"]
    dy   = target["y"] - pose["y"]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < WAYPOINT_REACH_CM:
        return 0.0, 0.0, dist
    target_angle = math.atan2(dy, dx)
    angle_err    = target_angle - pose["theta"]
    # Normalizza in [-π, +π]
    while angle_err >  math.pi: angle_err -= 2 * math.pi
    while angle_err < -math.pi: angle_err += 2 * math.pi
    if abs(angle_err) > ANGLE_THRESH_RAD:
        # Errore angolare grande → solo rotazione
        vang = max(-MAX_VANG, min(MAX_VANG,
               math.degrees(angle_err) * KP_ANGLE / 90.0))
        return 0.0, vang, dist
    else:
        # Errore angolare piccolo → avanza + correzione angolare
        vlin = min(MAX_VLIN, dist * 0.5)
        vang = math.degrees(angle_err) * KP_ANGLE / 90.0
        return vlin, vang, dist

async def path_follow_loop():
    """
    Coroutine asyncio attiva per tutta la durata del bridge.
    No-op quando following=False. Invia comandi a 20Hz quando attivo.
    Se l'ultimo waypoint ha il campo "theta", al raggiungimento della posizione
    il robot ruota fino a raggiungere quell'orientamento finale (tolleranza 3°).
    """
    global wp_index, following
    while True:
        await asyncio.sleep(CMD_INTERVAL)
        if not following or wp_index >= len(waypoints):
            if following:
                await cmd_queue.put({"type": "cmd", "vlin": 0.0, "vang": 0.0})
                following = False
                wp_index  = 0
                print("[PATH] Percorso completato")
                await ws_broadcast({"type": "log", "level": "ok",
                                    "msg": "Percorso completato"})
            continue
        target = waypoints[wp_index]
        vlin, vang, dist = compute_command(current_pose, target)

        if dist < WAYPOINT_REACH_CM:
            # Waypoint raggiunto — controlla se è l'ultimo con theta finale
            is_last      = (wp_index == len(waypoints) - 1)
            theta_target = target.get("theta", None)

            if is_last and theta_target is not None:
                # Calcola errore angolare verso il theta finale desiderato
                angle_err = theta_target - current_pose["theta"]
                while angle_err >  math.pi: angle_err -= 2 * math.pi
                while angle_err < -math.pi: angle_err += 2 * math.pi

                if abs(angle_err) > 0.05:  # ~3° tolleranza
                    # Ancora da ruotare — invia solo vang, non avanza
                    vang_final = max(-MAX_VANG, min(MAX_VANG,
                                    math.degrees(angle_err) * KP_ANGLE / 90.0))
                    await cmd_queue.put({"type": "cmd", "vlin": 0.0, "vang": vang_final})
                    continue  # non avanza al waypoint successivo

            print(f"[PATH] Waypoint {wp_index + 1}/{len(waypoints)} raggiunto")
            await ws_broadcast({
                "type":  "waypoint_reached",
                "index": wp_index,
                "total": len(waypoints),
            })
            wp_index += 1
        else:
            await cmd_queue.put({"type": "cmd", "vlin": vlin, "vang": vang})

# ══════════════════════════════════════════════════════════════════════════════
# MODULO WEBSOCKET
# ══════════════════════════════════════════════════════════════════════════════

async def ws_broadcast(data: dict):
    """
    Invia un messaggio JSON a tutti i client WebSocket connessi.
    return_exceptions=True evita che un errore su un client blocchi gli altri.
    """
    if not ws_clients:
        return
    msg = json.dumps(data)
    await asyncio.gather(
        *[c.send(msg) for c in ws_clients],
        return_exceptions=True
    )

async def ws_handler(websocket):
    """
    Handler WebSocket — gestisce una singola connessione da Godot.
    Registra il client nella lista ws_clients per il broadcast.
    Il loop "async for raw in websocket" rimane attivo per tutta la connessione.
    Le operazioni bloccanti (subprocess Docker) vengono eseguite con
    run_in_executor + partial per non congelare il loop asyncio.
    Messaggi gestiti: waypoints, stop, reset, cmd, pose, alignment,
    explorer_start/stop, slam_start/stop, nav2_start/stop/goal.
    Il loop "async for raw in websocket" rimane attivo per tutta la connessione.

    Le operazioni bloccanti (subprocess, sleep) vengono eseguite in
    run_in_executor per non congelare il loop asyncio.
    partial() permette di passare argomenti a run_in_executor.
    """
    global waypoints, wp_index, following, _align_rot_y

    ws_clients.add(websocket)
    addr = websocket.remote_address
    print(f"[WS] Connesso: {addr}")

    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            msg_type = data.get("type")

            # ── ping/pong — keepalive ─────────────────────────────────────────
            if msg_type == "ping":
                await websocket.send(json.dumps({"type": "pong"}))

            # ── waypoints — nuovo percorso da seguire ─────────────────────────
            # Godot invia la lista {x,y} in cm dopo SPAZIO in modalità Waypoint.
            elif msg_type == "waypoints":
                pts = data.get("points", [])
                if not pts:
                    continue
                waypoints = [{"x": float(p["x"]), "y": float(p["y"]),
                              **( {"theta": float(p["theta"])} if "theta" in p else {} )}
                             for p in pts]
                wp_index  = 0
                following = True
                print(f"[PATH] Nuovo percorso: {len(waypoints)} punti")
                for i, p in enumerate(waypoints):
                    print(f"  [{i+1}] x={p['x']:.1f}cm  y={p['y']:.1f}cm")
                await ws_broadcast({
                    "type": "log", "level": "info",
                    "msg":  f"Percorso avviato: {len(waypoints)} punti",
                })

            # ── stop ──────────────────────────────────────────────────────────
            elif msg_type == "stop":
                following = False
                wp_index  = 0
                await cmd_queue.put({"type": "cmd", "vlin": 0.0, "vang": 0.0})
                print("[PATH] Stop ricevuto")
                await ws_broadcast({"type": "log", "level": "warn",
                                    "msg": "Percorso interrotto"})

            # ── pose — posa corrente di Alvik da Godot ────────────────────────
            elif msg_type == "pose":
                current_pose["x"]     = float(data.get("x",     0))
                current_pose["y"]     = float(data.get("y",     0))
                current_pose["theta"] = float(data.get("theta", 0))

            # ── reset — azzera odometria Alvik ────────────────────────────────
            elif msg_type == "reset":
                following = False
                wp_index  = 0
                await cmd_queue.put({"type": "reset"})
                await ws_broadcast({"type": "log", "level": "warn",
                                    "msg": "Posa resettata"})

            # ── cmd — velocità diretta (teleop da Godot) ──────────────────────
            elif msg_type == "cmd":
                vlin = float(data.get("vlin", 0))
                vang = float(data.get("vang", 0))
                await cmd_queue.put({"type": "cmd", "vlin": vlin, "vang": vang})

            # ── alignment — offset rotazione allineamento ─────────────────────
            # Inviato da Godot quando si esce dalla modalità ALIGN.
            # L'offset serve solo a Godot per il calcolo dei waypoint —
            # NON viene pubblicato su ROS2 perché ROS2 ha il suo sistema
            # di riferimento indipendente dalla visualizzazione Godot.
            elif msg_type == "alignment":
                _align_rot_y = float(data.get("rot_y", 0.0))
                print(f"[ALIGN] offset: {_align_rot_y:.3f} rad "
                      f"({math.degrees(_align_rot_y):.1f}°) — solo Godot, non ROS2")
                # Commentati perchè non voglio inviare l'allineamento a ROS ma solo a Godot
                # ROS2 ha il suo sistema di riferimento indipendente
                # In seguito si può prevedere una funzione di allineamento tra i 3 sistemi:
                # - Alvik Fisico
                # - Alvik Godot
                # - Alvik RViz (SLAM e NAV2)
                # loop = asyncio.get_event_loop()
                # await loop.run_in_executor(None, publish_alignment, _align_rot_y)

            # ── explorer_start — Esploratore (SLAM + PID) ────────────────────
            elif msg_type == "explorer_start":
                print("[EXPLORER] Richiesta avvio")
                loop    = asyncio.get_event_loop()
                success = await loop.run_in_executor(None, explorer_start)
                if success:
                    await ws_broadcast({"type": "log", "level": "ok",
                                        "msg": "Modalità Esploratore avviata"})
                    await ws_broadcast({"type": "explorer_started"})
                else:
                    await ws_broadcast({"type": "log", "level": "error",
                                        "msg": "Errore avvio Esploratore"})

            # ── explorer_stop ─────────────────────────────────────────────────
            elif msg_type == "explorer_stop":
                print("[EXPLORER] Richiesta stop")
                save_map = data.get("save_map", True)
                loop     = asyncio.get_event_loop()
                # partial() pre-inserisce save_map perché run_in_executor
                # non supporta argomenti aggiuntivi
                success  = await loop.run_in_executor(
                    None, partial(explorer_stop, save_map))
                if success:
                    msg = ("Esploratore terminato — mappa salvata"
                           if save_map else "Esploratore terminato")
                    await ws_broadcast({"type": "log", "level": "warn", "msg": msg})
                    await ws_broadcast({"type": "explorer_stopped"})

            # ── slam_start — SLAM standalone (senza PID) ─────────────────────
            # Avvia solo slam_toolbox. Alvik va guidato manualmente con teleop.
            elif msg_type == "slam_start":
                print("[SLAM] Richiesta avvio SLAM standalone")
                loop    = asyncio.get_event_loop()
                success = await loop.run_in_executor(None, slam_start)
                if success:
                    await ws_broadcast({"type": "log", "level": "ok",
                                        "msg": "SLAM avviato — usa T per teleop"})
                    await ws_broadcast({"type": "slam_started"})
                else:
                    await ws_broadcast({"type": "log", "level": "error",
                                        "msg": "Errore avvio SLAM"})

            # ── slam_stop ─────────────────────────────────────────────────────
            elif msg_type == "slam_stop":
                print("[SLAM] Richiesta stop SLAM standalone")
                save_map = data.get("save_map", True)
                loop     = asyncio.get_event_loop()
                success  = await loop.run_in_executor(
                    None, partial(slam_stop, save_map))
                if success:
                    msg = ("SLAM terminato — mappa salvata"
                           if save_map else "SLAM terminato")
                    await ws_broadcast({"type": "log", "level": "warn", "msg": msg})
                    await ws_broadcast({"type": "slam_stopped"})

            # ── nav2_start — navigazione autonoma ────────────────────────────
            # Richiede mappa in MAP_NAV_PATH. Avvia AMCL + Nav2.
            elif msg_type == "nav2_start":
                print("[NAV2] Richiesta avvio navigazione autonoma")
                loop    = asyncio.get_event_loop()
                success = await loop.run_in_executor(None, nav2_start)
                if success:
                    await ws_broadcast({"type": "log", "level": "ok",
                                        "msg": "Nav2 pronto — clicca sulla mappa per goal"})
                    await ws_broadcast({"type": "nav2_started"})
                else:
                    await ws_broadcast({"type": "log", "level": "error",
                                        "msg": "Errore avvio Nav2"})

            # ── nav2_stop ─────────────────────────────────────────────────────
            elif msg_type == "nav2_stop":
                print("[NAV2] Richiesta stop Nav2")
                loop    = asyncio.get_event_loop()
                success = await loop.run_in_executor(None, nav2_stop)
                if success:
                    await ws_broadcast({"type": "log", "level": "warn",
                                        "msg": "Nav2 fermato"})
                    await ws_broadcast({"type": "nav2_stopped"})

            # ── nav2_goal — goal di navigazione autonoma ──────────────────────
            # Godot invia x,y in metri (già convertiti da coord Godot a ROS2).
            # Il bridge pubblica su /goal_pose nel container Docker.
            elif msg_type == "nav2_goal":
                x     = float(data.get("x",     0.0))
                y     = float(data.get("y",     0.0))
                theta = float(data.get("theta", 0.0))
                print(f"[NAV2] Goal: ({x:.2f}, {y:.2f}) m  theta={theta:.2f} rad")
                container = get_container_name()
                if container:
                    loop = asyncio.get_event_loop()
                    await loop.run_in_executor(
                        None, partial(nav2_send_goal, container, x, y, theta))
                    await ws_broadcast({"type": "log", "level": "info",
                                        "msg": f"Nav2 goal: ({x:.2f}, {y:.2f}) m"})
                else:
                    await ws_broadcast({"type": "log", "level": "error",
                                        "msg": "Nav2: container non trovato"})

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        ws_clients.discard(websocket)
        print(f"[WS] Disconnesso: {addr}")

# ══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

async def main():
    """
    Entry point asyncio: avvia il thread DDS e il loop asyncio con
    WebSocket server e path follow loop in parallelo con asyncio.gather().
    """
    """
    Avvia tutti i moduli in parallelo:
      1. Thread DDS daemon (timing RT 50Hz)
      2. Server WebSocket asyncio
      3. Coroutine path_follow_loop asyncio

    La asyncio.Queue deve essere creata qui nell'event loop corrente.
    """
    global cmd_queue
    cmd_queue = asyncio.Queue()

    print("=" * 56)
    print("  bridge_v1_7.py — Alvik Digital Twin v1.7")
    print(f"  Alvik      ← UDP :{ALVIK_LISTEN_PORT}  (telemetria CSV)")
    print(f"  Godot DDS  → UDP :{GODOT_PORT}          (variabili DDS)")
    print(f"  ROS2 DDS   → UDP :{ROS_PORT}            (variabili DDS)")
    print(f"  Nav2 CMD   ← UDP :{ROS_CMD_PORT}        (futuro /cmd_vel)")
    print(f"  WebSocket  ↔ :{WS_PORT}                 (comandi Godot)")
    print(f"  DDS tick     {TICK_HZ}Hz  |  Path follow {CMD_HZ}Hz")
    print(f"  Mappa SLAM → {MAP_SAVE_PATH}")
    print(f"  Mappa Nav2 ← {MAP_NAV_PATH}")
    print("=" * 56)

    # Thread DDS — daemon: si chiude automaticamente con il processo principale
    loop = asyncio.get_event_loop()
    dds_thread = threading.Thread(
        target=dds_loop, args=(loop,), daemon=True, name="DDS-Thread"
    )
    dds_thread.start()
    print(f"[MAIN] Thread DDS avviato: {dds_thread.name}")

    # Server WebSocket
    ws_server = await websockets.serve(ws_handler, WS_HOST, WS_PORT)
    print(f"[MAIN] WebSocket server su {WS_HOST}:{WS_PORT}")

    # Esegui path following e WebSocket in parallelo
    await asyncio.gather(
        path_follow_loop(),
        ws_server.wait_closed(),
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nBridge fermato (Ctrl+C).")
        # Cleanup: ferma Alvik e tutti i processi ROS2 attivi
        if _explorer_active:
            print("Arresto Esploratore...")
            explorer_stop(save_map=True)
        if _slam_active:
            print("Arresto SLAM...")
            slam_stop(save_map=True)
        if _nav2_active:
            print("Arresto Nav2...")
            nav2_stop()
        send_stop_alvik()
        # Cleanup finale: pkill globale di tutti i nodi ROS2 residui nel container
        # Questo cattura qualsiasi processo rimasto dopo i singoli stop
        container = get_container_name()
        if container:
            print("[CLEANUP] Terminazione processi ROS2 residui nel container...")
            _kill_processes(container, [
                "slam_toolbox", "alvik_pid_controller", "alvik_ros_bridge",
                "alvik_launch", "robot_state_publisher", "nav2_bringup",
                "controller_server", "planner_server", "bt_navigator",
                "amcl", "map_server", "rviz2"
            ])
    finally:
        # Chiude tutti i socket UDP
        sock_alvik.close()
        sock_godot.close()
        sock_ros.close()
        sock_ros_cmd.close()
        print("[MAIN] Socket chiusi.")
