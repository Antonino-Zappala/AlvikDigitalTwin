#!/bin/bash
# ============================================================
#  alvik_stop_v1_7.sh — Ferma tutto Alvik Digital Twin v1.7
# ============================================================
#
#  Uso: ./alvik_stop_v1_7.sh
#
#  Da eseguire quando si chiude la sessione con la X del terminale
#  invece di Ctrl+C, oppure quando rimangono processi attivi.
#
#  Ferma nell'ordine:
#    1. bridge_v1_7.py
#    2. Container Docker (termina tutti i processi ROS2 al suo interno)
#    3. Fallback pkill per processi sfuggiti
# ============================================================

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${YELLOW}"
echo "  ╔══════════════════════════════════════════════════════════════╗"
echo "  ║           ALVIK DIGITAL TWIN v1.7 — STOP                     ║"
echo "  ╚══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# ── Step 1: Invia SIGTERM allo script principale se ancora attivo ──────────
if [ -f /tmp/alvik_v1_7.pid ]; then
    PID=$(cat /tmp/alvik_v1_7.pid)
    if kill -0 "$PID" 2>/dev/null; then
        echo -e "${YELLOW}[1] Invio SIGTERM allo script principale (PID $PID)...${NC}"
        kill -TERM "$PID" 2>/dev/null
        sleep 2
    fi
    rm -f /tmp/alvik_v1_7.pid
fi

# ── Step 2: Termina il bridge ──────────────────────────────────────────────
echo -e "${YELLOW}[2] Termino bridge_v1_7.py...${NC}"
pkill -f "bridge_v1_7.py" 2>/dev/null
# Killa alvik_ros_bridge dentro il container
docker exec alvik_ros2 bash -c "pkill -f alvik_ros_bridge" 2>/dev/null
# Ferma il container
cd ~/Scrivania/progetto_alvik/ros2 && docker-compose down 2>/dev/null
# Chiude Godot
pkill -f "Godot.*alvik_digital_twin" 2>/dev/null
sleep 1

# ── Step 3: Ferma il container Docker ────────────────────────────────────
# docker stop invia SIGTERM a tutti i processi nel container e poi SIGKILL
echo -e "${YELLOW}[3] Fermo container Docker ROS2...${NC}"
CONTAINERS=$(docker ps --filter "name=ros2-alvik_ros2" --format "{{.Names}}" 2>/dev/null)
if [ -n "$CONTAINERS" ]; then
    docker stop $CONTAINERS 2>/dev/null
    echo -e "${GREEN}    Container fermati: $CONTAINERS${NC}"
else
    echo -e "${GREEN}    Nessun container attivo.${NC}"
fi

# ── Step 4: Fallback pkill ────────────────────────────────────────────────
# Cattura qualsiasi processo ROS2 rimasto fuori dal container
echo -e "${YELLOW}[4] Fallback pkill processi residui...${NC}"
PROCS=(
    "alvik_ros_bridge"
    "alvik_pid_controller"
    "slam_toolbox"
    "alvik_launch"
    "robot_state_publisher"
    "nav2_bringup"
    "controller_server"
    "planner_server"
    "bt_navigator"
    "amcl"
    "map_server"
    "rviz2"
)
for p in "${PROCS[@]}"; do
    if pgrep -f "$p" > /dev/null 2>&1; then
        pkill -f "$p" 2>/dev/null
        echo -e "${GREEN}    Terminato: $p${NC}"
    fi
done

# ── Verifica finale ───────────────────────────────────────────────────────
echo ""
RESIDUI=$(ps aux | grep -E "alvik|slam_toolbox|rviz2" | grep -v grep | grep -v "alvik_stop_v1_7" | wc -l)
if [ "$RESIDUI" -eq 0 ]; then
    echo -e "${GREEN}  ✓ Tutti i processi terminati.${NC}"
else
    echo -e "${RED}  ⚠ Ancora $RESIDUI processi attivi — verifica con: ps aux | grep alvik${NC}"
fi
echo ""
