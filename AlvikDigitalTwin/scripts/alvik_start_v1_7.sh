#!/bin/bash
# ============================================================
#  alvik_start_v1_7.sh — Avvio completo Alvik Digital Twin
#  Versione 1.7 — Tutto in AlvikDigitalTwin_v1_7/
# ============================================================
#
#  Modalità disponibili all'avvio:
#    [ENTER]  — Applicazione principale (Godot + bridge + ROS2 + RViz2)
#    [C]      — Calibrazione PID (bridge + ROS2 + PID + PlotJuggler)
#    [Ctrl+C] — Annulla
#
#  Le modalità operative (Esploratore, SLAM, Nav2) si attivano
#  direttamente dal menu di Godot (tasto M) senza riavviare lo script.
#
#  Struttura directory:
#    AlvikDigitalTwin_v1_7/
#    ├── alvik_start_v1_7.sh       ← questo script
#    ├── bridge_v1_7.py            ← middleware unificato
#    ├── ros2/                     ← container Docker ROS2
#    │   ├── docker-compose.yml
#    │   └── src/alvik_bridge/...
#    └── alvik_digital_twin_v1_7/  ← progetto Godot
#        └── project.godot
# ============================================================

# ══════════════════════════════════════════════════════════════
# CONFIGURAZIONE — modifica questi path se necessario
# ══════════════════════════════════════════════════════════════

# Directory base del progetto v1.7 — tutto qui dentro
PROJECT_DIR="/home/neobot78/Scrivania/progetto_alvik/AlvikDigitalTwin_v1_7"

# Directory ros2 — dentro PROJECT_DIR (docker-compose.yml con path v1.7)
ROS2_DIR="/home/neobot78/Scrivania/progetto_alvik/ros2"

# Binario Godot 4 sul desktop
GODOT_BIN="/home/neobot78/Scrivania/Godot"

# Directory del progetto Godot — dentro PROJECT_DIR
GODOT_PROJECT="$PROJECT_DIR/alvik_digital_twin_v1_7"

# File configurazione RViz2 dentro il container Docker
# /ros2_ws/config/ è montato come volume Docker → persiste tra sessioni
RVIZ_CONFIG="/ros2_ws/config/alvik.rviz"

# File sorgente PID — per leggere i parametri attuali nel pannello calibrazione
PID_FILE="$ROS2_DIR/src/alvik_bridge/alvik_bridge/alvik_pid_controller.py"

# Secondi di attesa per l'avvio del container Docker
WAIT_CONTAINER=5

# ══════════════════════════════════════════════════════════════
# CLEANUP AUTOMATICO
# ══════════════════════════════════════════════════════════════
# Chiamato automaticamente da SIGINT (Ctrl+C) o SIGTERM (kill).
# Ferma il container Docker e il bridge per non lasciare zombie.

stop_all() {
    echo -e "\n\033[1;33m[CLEANUP] Arresto in corso...\033[0m"
    # 1. Termina il bridge (lui fa pkill dei nodi ROS2 prima di chiudersi)
    pkill -f "bridge_v1_7.py" 2>/dev/null
    sleep 1
    # 2. Ferma il container Docker — termina TUTTI i processi al suo interno
    #    inclusi: alvik_ros_bridge, slam_toolbox, pid_controller, rviz2
    docker stop $(docker ps --filter "name=ros2-alvik_ros2" \
        --format "{{.Names}}" 2>/dev/null) 2>/dev/null
    # 3. Fallback pkill diretto — cattura processi sfuggiti al container stop
    pkill -f "alvik_ros_bridge" 2>/dev/null
    pkill -f "alvik_pid_controller" 2>/dev/null
    pkill -f "slam_toolbox" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    # 4. Rimuove il file PID
    rm -f /tmp/alvik_v1_7.pid 2>/dev/null
    echo -e "\033[0;32m[CLEANUP] Fatto.\033[0m"
}
CLEANING=0
cleanup() { [ "$CLEANING" = "1" ] && return; CLEANING=1; stop_all; exit 0; }
trap cleanup INT TERM HUP EXIT

# Salva il PID di questo script in /tmp — usato da alvik_stop_v1_7.sh
# per fermare tutto anche quando si chiude la finestra con la X
echo $$ > /tmp/alvik_v1_7.pid

# ══════════════════════════════════════════════════════════════
# COLORI ANSI
# ══════════════════════════════════════════════════════════════
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
WHITE='\033[1;37m'
GRAY='\033[0;37m'
RED='\033[0;31m'
NC='\033[0m'

# Imposta dimensione terminale (70 colonne × 45 righe — adatta al menu)
printf '\033[8;48;70t'
sleep 0.1   # attende che il terminale ridimensioni

# ══════════════════════════════════════════════════════════════
# FUNZIONE: leggi parametro PID dal file sorgente
# Cerca declare_parameter('nome', valore) e restituisce il valore
# ══════════════════════════════════════════════════════════════
get_pid_param() {
    grep "declare_parameter('$1'" "$PID_FILE" 2>/dev/null | \
        grep -oP "[\d.]+" | tail -1
}

# ══════════════════════════════════════════════════════════════
# PANNELLO PRINCIPALE
# ══════════════════════════════════════════════════════════════
show_main_panel() {
    clear
    echo -e "${CYAN}"
    echo "  ╔══════════════════════════════════════════════════════════════╗"
    echo "  ║      ALVIK DIGITAL TWIN  v1.7                                ║"
    echo "  ║      Arduino Alvik (ESP32 + STM32) + Godot + ROS2 Humble     ║"
    echo "  ╠══════════════════════════════════════════════════════════════╣"
    echo -e "  ║${NC}${WHITE}  ARCHITETTURA${CYAN}                                                ║"
    echo "  ║                                                              ║"
    echo "  ║   Alvik (192.168.4.1:5005)                                   ║"
    echo "  ║       ↕ UDP telemetria CSV 50Hz + comandi CMD                ║"
    echo "  ║   bridge_v1_7  →  Godot DDS  :4444  (X,Y,Theta,ToF...)       ║"
    echo "  ║                →  ROS2 DDS   :4445  (alvik_ros_bridge)       ║"
    echo "  ║                ↔  WebSocket  :8765  ↔  Godot                 ║"
    echo -e "  ╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}  ║${NC}${WHITE}  COMPONENTI AVVIATI${CYAN}                                          ║${NC}"
    echo -e "${CYAN}  ║${NC}                                                              ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  ${GREEN}[1]${NC} ${WHITE}Container ROS2${NC} — alvik_ros_bridge                       ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}      pubblica /odom /tf /scan da telemetria DDS              ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  ${GREEN}[2]${NC} ${WHITE}bridge_v1_7.py${NC} — middleware DDS + WebSocket             ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}      gestisce tutte le modalità operative                    ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  ${GREEN}[3]${NC} ${WHITE}Godot Digital Twin v1.7${NC} — 3D + menu modalità            ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  ${YELLOW}[*]${NC} ${WHITE}RViz2${NC} — avviato con E/L/N (SLAM e Nav2)                 ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  ${YELLOW}[*]${NC} ${WHITE}PID Controller${NC} — avviato con E (Esploratore)            ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}                                                              ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  ${YELLOW}Modalità Godot (M per menu):${NC}                                ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  S=Waypoint    T=Teleop      P=Path    A=Allineamento        ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  E=Esploratore (SLAM+PID)    L=SLAM    N=Nav2                ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  G=Griglia+Freccia  R=Reset  C=Cancella ostacoli             ${CYAN}║${NC}"
    echo -e "${CYAN}  ╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}  ║${NC}${YELLOW}  REQUISITI${NC}                                                   ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  • Alvik acceso e connesso alla rete Alvik_Robot_WiFi        ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  • PC connesso alla rete Alvik_Robot_WiFi                   ${CYAN} ║${NC}"
    echo -e "${CYAN}  ║${NC}  • Docker in esecuzione                                     ${CYAN} ║${NC}"
    echo -e "${CYAN}  ╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}  ║${NC}${YELLOW}  NOTE${NC}                                                        ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  • Prima sessione: esplora con E, salva mappa, poi usa N     ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  • Reset (R) prima di ogni nuova sessione di navigazione     ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  • alvik_shell: accesso rapido al container ROS2             ${CYAN}║${NC}"
    echo -e "${CYAN}  ╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}  ║${NC}                                                              ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  ${GREEN}[ENTER]${NC}  Avvia applicazione principale                      ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}  ${YELLOW}[C]${NC}      Modalità calibrazione PID                         ${CYAN} ║${NC}"
    echo -e "${CYAN}  ║${NC}  ${GRAY}[Ctrl+C]${NC} Annulla                                            ${CYAN}║${NC}"
    echo -e "${CYAN}  ║${NC}                                                             ${CYAN} ║${NC}"
    echo -e "${CYAN}  ╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# ══════════════════════════════════════════════════════════════
# PANNELLO CALIBRAZIONE PID
# ══════════════════════════════════════════════════════════════
show_calibration_panel() {
    clear

    KP_LIN=$(get_pid_param "kp_lin")
    KI_LIN=$(get_pid_param "ki_lin")
    KP_ANG=$(get_pid_param "kp_ang")
    KI_ANG=$(get_pid_param "ki_ang")
    ACC_LIN=$(get_pid_param "acc_lin")
    DEC_LIN=$(get_pid_param "dec_lin")
    VMAX_LIN=$(get_pid_param "vmax_lin")
    VMAX_ANG=$(get_pid_param "vmax_ang")
    ANGLE_THRESH=$(get_pid_param "angle_thresh")
    REACH_THRESH=$(get_pid_param "reach_thresh")

    echo -e "${YELLOW}"
    echo "  ╔══════════════════════════════════════════════════════════════╗"
    echo "  ║           MODALITÀ CALIBRAZIONE PID                          ║"
    echo "  ╠══════════════════════════════════════════════════════════════╣"
    echo -e "  ║${NC}${WHITE}  DESCRIZIONE${YELLOW}                                                 ║"
    echo "  ║                                                              ║"
    echo "  ║  Controllo in cascata a due anelli:                          ║"
    echo "  ║  • Anello ESTERNO (ROS2, ~20Hz) — PID posizione x,y,theta    ║"
    echo "  ║  • Anello INTERNO (STM32, ~1kHz) — PID velocità ruote        ║"
    echo "  ║                                                              ║"
    echo "  ║  Per calibrare: modifica alvik_pid_controller.py             ║"
    echo "  ║  poi colcon build nel container (alvik_shell)                ║"
    echo -e "  ╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}  ║${NC}${WHITE}  PARAMETRI ATTUALI${YELLOW}                                           ║${NC}"
    echo -e "${YELLOW}  ║${NC}                                                              ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  ${CYAN}Velocità lineare:${NC}                                           ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    kp_lin       = ${GREEN}${KP_LIN:-?}${NC}   (guadagno proporzionale)             ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    ki_lin       = ${GREEN}${KI_LIN:-?}${NC}   (guadagno integrale)                 ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    acc_lin      = ${GREEN}${ACC_LIN:-?}${NC}   cm/s² (rampa accelerazione)          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    dec_lin      = ${GREEN}${DEC_LIN:-?}${NC}   cm/s² (rampa decelerazione)          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    vmax_lin     = ${GREEN}${VMAX_LIN:-?}${NC}  cm/s  (velocità massima)             ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}                                                              ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  ${CYAN}Velocità angolare:${NC}                                          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    kp_ang       = ${GREEN}${KP_ANG:-?}${NC}  (guadagno proporzionale)             ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    ki_ang       = ${GREEN}${KI_ANG:-?}${NC}   (guadagno integrale)                 ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    vmax_ang     = ${GREEN}${VMAX_ANG:-?}${NC}  deg/s (velocità massima)             ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}                                                              ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  ${CYAN}Soglie:${NC}                                                     ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    angle_thresh = ${GREEN}${ANGLE_THRESH:-?}${NC} rad  (sotto cui si avanza)            ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}    reach_thresh = ${GREEN}${REACH_THRESH:-?}${NC}  cm   (waypoint raggiunto)             ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}  ║${NC}${WHITE}  COMANDI DI TEST${YELLOW}                                             ║${NC}"
    echo -e "${YELLOW}  ║${NC}  Test lineare:   goto_rel:25.0,0.0     (25cm avanti)         ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  Test angolare:  goto_rel:0.0,1.5708   (90° sinistra)        ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  Test combinato: goto:25.0,25.0,0.0                          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  Parametri ottimali: kp_lin=1.5 ki_lin=0.8 kp_ang=25.0       ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ╠══════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}  ║${NC}                                                              ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  ${GREEN}[ENTER]${NC}  Avvia modalità calibrazione                        ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  ${GRAY}[Ctrl+C]${NC} Annulla                                            ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}                                                              ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ╚══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

# ══════════════════════════════════════════════════════════════
# COMPONENTI COMUNI (avviati da entrambe le modalità)
# ══════════════════════════════════════════════════════════════
avvia_comuni() {
    # X11 forwarding per Docker (necessario per RViz2)
    echo -e "${GREEN}[1/4] Abilitazione X11 forwarding per Docker...${NC}"
    xhost +local:docker > /dev/null 2>&1

    # Ferma container precedenti per evitare conflitti
    echo -e "${GREEN}[2/4] Pulizia container precedenti...${NC}"
    docker stop $(docker ps --filter "name=ros2-alvik_ros2" \
        --format "{{.Names}}" 2>/dev/null) 2>/dev/null

    # Avvia container ROS2 con nome fisso "alvik_ros2" (docker-compose up)
    # alvik_ros_bridge riceve il DDS dal bridge e pubblica /odom /tf /scan
    echo -e "${GREEN}[3/4] Avvio container ROS2 + alvik_ros_bridge...${NC}"
    cd $ROS2_DIR
    docker-compose down 2>/dev/null
    docker-compose up -d 2>/dev/null
    echo -e "${GREEN}    Attendo ${WAIT_CONTAINER}s per il container...${NC}"
    sleep $WAIT_CONTAINER
    gnome-terminal --title="ROS2 alvik_ros_bridge" -- bash -c "
      docker exec -it alvik_ros2 bash -c '
        source /opt/ros/humble/setup.bash
        source /ros2_ws/install/setup.bash
        ros2 run alvik_bridge alvik_ros_bridge
      '
      exec bash
    " &
    sleep 2

    # Avvia bridge_v1_7.py — middleware unificato DDS + WebSocket
    echo -e "${GREEN}[4/4] Avvio bridge_v1_7.py...${NC}"
    gnome-terminal --title="Bridge v1.7" -- bash -c "
      cd $PROJECT_DIR
      python3 bridge_v1_7.py
      exec bash
    " &
    sleep 1
    # Nota: RViz2 viene avviato dal bridge quando si attiva la modalità
    # Esploratore (E) o SLAM (L) — non all'avvio generale.
}

# ══════════════════════════════════════════════════════════════
# MODALITÀ PRINCIPALE
# ══════════════════════════════════════════════════════════════
avvia_principale() {
    avvia_comuni

    # Avvia Godot con il progetto v1.7
    echo -e "${GREEN}[+] Avvio Godot Digital Twin v1.7...${NC}"
    $GODOT_BIN --path $GODOT_PROJECT --resolution 1480x950 &

    echo ""
    echo -e "${CYAN}  ╔══════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}  ║   Applicazione principale avviata!                           ║${NC}"
    echo -e "${CYAN}  ║                                                              ║${NC}"
    echo -e "${CYAN}  ║   In Godot premi M per il menu — modalità:                   ║${NC}"
    echo -e "${CYAN}  ║   [S] Waypoint    — click sul pavimento + SPAZIO invia       ║${NC}"
    echo -e "${CYAN}  ║   [T] Teleop      — W/X avanza/indietro, Q/E ruota           ║${NC}"
    echo -e "${CYAN}  ║   [P] Path        — quadrato/cerchio/triangolo               ║${NC}"
    echo -e "${CYAN}  ║   [A] Allineamento — frecce sposta/ruota, R azzera           ║${NC}"
    echo -e "${CYAN}  ║   [E] Esploratore  — SLAM + PID, waypoint automatici         ║${NC}"
    echo -e "${CYAN}  ║   [L] SLAM         — mappatura con teleop manuale            ║${NC}"
    echo -e "${CYAN}  ║   [N] Nav2         — navigazione autonoma su mappa           ║${NC}"
    echo -e "${CYAN}  ║                                                              ║${NC}"
    echo -e "${CYAN}  ║   Per nuove shell ROS2:  alvik_shell                         ║${NC}"
    echo -e "${CYAN}  ║   Per fermare tutto:     Ctrl+C                              ║${NC}"
    echo -e "${CYAN}  ╚══════════════════════════════════════════════════════════════╝${NC}"
}

# ══════════════════════════════════════════════════════════════
# MODALITÀ CALIBRAZIONE PID
# ══════════════════════════════════════════════════════════════
avvia_calibrazione() {
    avvia_comuni

    # Avvia nodo PID nel container
    echo -e "${YELLOW}[+] Avvio alvik_pid_controller...${NC}"
    gnome-terminal --title="PID Controller" -- bash -c "
      docker exec -it alvik_ros2 bash -c '
        source /opt/ros/humble/setup.bash
        source /ros2_ws/install/setup.bash
        ros2 run alvik_bridge alvik_pid_controller
      '
      exec bash
    " # &
    sleep 1

    # Avvia PlotJuggler per visualizzare /pid_debug in tempo reale
    echo -e "${YELLOW}[+] Avvio PlotJuggler...${NC}"
    gnome-terminal --title="PlotJuggler" -- bash -c "
      docker exec -it alvik_ros2 bash -c '
        source /opt/ros/humble/setup.bash
        DISPLAY=:1 ros2 run plotjuggler plotjuggler
      '
      exec bash
    " &

    printf '\033[8;55;100t'  # Ridimensiona terminale: 55 righe x 100 colonne
    echo ""
    echo -e "${YELLOW}  ╔══════════════════════════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}  ║                            MODALITÀ CALIBRAZIONE PID AVVIATA                             ║${NC}"
    echo -e "${YELLOW}  ╠══════════════════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}  ║${NC}${WHITE}   In PlotJuggler:${YELLOW}                                                                        ║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${GREEN}[1]${NC} Streaming → ROS2 Topic Subscriber → Start                                    ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${GREEN}[2]${NC} Seleziona /pid_debug → Buffer 30sec                                          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${GREEN}[3]${NC} Trascina i campi nel grafico es: data[0] (errore), data[1] (cmd)             ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ╠══════════════════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}  ║${NC}${WHITE}  LEGENDA /pid_debug${YELLOW}                                                                      ║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}data[0]${NC} = x_target      ${CYAN}data[1]${NC} = x_real (posizione X, cm)                       ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}data[2]${NC} = y_target      ${CYAN}data[3]${NC} = y_real (posizione Y, cm)                       ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}data[4]${NC} = theta_target  ${CYAN}data[5]${NC} = theta_real (gradi)                             ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║                                                                                          ║${NC}"
    echo -e "${YELLOW}  ║${NC}  ${WHITE}Grafici utili:${NC}                                                                          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║         data[0] vs data[1]${NC} — errore posizione X                                          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║         data[2] vs data[3]${NC} — errore posizione Y                                          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║         data[4] vs data[5]${NC} — errore angolare                                             ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ╠══════════════════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}  ║${NC}${WHITE}   Significato dei dati:${YELLOW}                                                                  ║${NC}"      
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}- data[0]${NC} = x_target (posizione X obiettivo in cm)                               ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}- data[1]${NC} = x_real (posizione X attuale in cm)                                   ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}- data[2]${NC} = y_target (posizione Y obiettivo in cm)                               ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}- data[3]${NC} = y_real (posizione Y attuale in cm)                                   ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}- data[4]${NC} = theta_target in gradi                                                ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}- data[5]${NC} = theta_real in gradi                                                  ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}- data[6]${NC} = vlin (placeholder*, sempre 0)                                        ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║       ${NC}  ${CYAN}- data[7]${NC} = vang (placeholder*, sempre 0)                                        ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ╠══════════════════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}  ║${NC}${WHITE}   Test (da alvik_shell):   ${YELLOW}                                                              ║${NC}"
    echo -e "${YELLOW}  ║         • Lineare:   goto_rel:25.0,0.0                                                   ║${NC}"
    echo -e "${YELLOW}  ║         • Angolare:  goto_rel:0.0,1.5708  (90°)                                          ║${NC}"
    echo -e "${YELLOW}  ║         • Combinato: goto:25.0,25.0,0.0                                                  ║${NC}"
    echo -e "${YELLOW}  ║                                                                                          ║${NC}"
    echo -e "${YELLOW}  ║ ${NC}${WHITE}  Esempi comandi da alvik_shell:                                                         ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║   ${GREEN}Test lineare — vai avanti 25cm${NC}                                                         ${YELLOW}║${NC}" 
    echo -e "${YELLOW}  ║   ros2 topic pub --once /pid/command std_msgs/msg/String '{data: \"goto_rel:25.0,0.0\"}'   ║${NC}"
    echo -e "${YELLOW}  ║                                                                                          ║${NC}"     
    echo -e "${YELLOW}  ║   ${GREEN}Test angolare — ruota 90°${NC}                                                              ${YELLOW}║${NC}" 
    echo -e "${YELLOW}  ║   ros2 topic pub --once /pid/command std_msgs/msg/String '{data: \"goto_rel:0.0,1.5708\"}' ║${NC}"
    echo -e "${YELLOW}  ║                                                                                          ║${NC}" 
    echo -e "${YELLOW}  ║   ${GREEN}Test combinato — vai a (25, 25) con theta=0${NC}                                            ${YELLOW}║${NC}" 
    echo -e "${YELLOW}  ║   ros2 topic pub --once /pid/command std_msgs/msg/String '{data: \"goto:25.0,25.0,0.0\"}'  ║${NC}"                                                    
    echo -e "${YELLOW}  ╠══════════════════════════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${YELLOW}  ║${NC}${WHITE}  PARAMETRI OTTIMALI v1.7                                                                 ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  ${CYAN}kp_lin${NC}=1.5  ${CYAN}ki_lin${NC}=0.8  ${CYAN}kp_ang${NC}=25.0  ${CYAN}ki_ang${NC}=0.0                                         ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  ${CYAN}acc${NC}=3.0  ${CYAN}dec${NC}=2.0  ${CYAN}vmax_lin${NC}=12.0  ${CYAN}vmax_ang${NC}=60.0                                          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}                                                                                          ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  Per nuove shell ROS2: ${GREEN}alvik_shell${NC}                                                       ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ║${NC}  *Riservato per dati futuri, al momento non pubblicato                                   ${YELLOW}║${NC}"
    echo -e "${YELLOW}  ╚══════════════════════════════════════════════════════════════════════════════════════════╝${NC}"
}

# ══════════════════════════════════════════════════════════════
# MAIN — selezione modalità
# ══════════════════════════════════════════════════════════════
show_main_panel

# Legge un singolo tasto senza ENTER
read -r -n 1 key
echo ""

if [[ "$key" == "c" || "$key" == "C" ]]; then
    show_calibration_panel
    read -r
    avvia_calibrazione
else
    avvia_principale
fi

# wait mantiene lo script attivo per il trap cleanup.
# Per fermare tutto: Ctrl+C oppure esegui alvik_stop_v1_7.sh
echo -e "${GREEN}  Ctrl+C per fermare tutto, oppure esegui: ./alvik_stop_v1_7.sh${NC}"
# Loop infinito — aspetta Ctrl+C o alvik_stop_v1_7.sh
while true; do
    sleep 1
done
