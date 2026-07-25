"""
alvik_pid_controller.py
=======================
Nodo ROS2 — Controllo PID posizione con profilo trapezoidale in velocità.

Architettura a cascata:
  Anello esterno (questo nodo) — controllo POSIZIONE ~20Hz
      feedback: /odom (x, y, theta)
      output:   /cmd_vel (vlin cm/s, vang deg/s) → STM32
  Anello interno (STM32 Alvik) — controllo VELOCITÀ ~1kHz
      feedback: encoder
      output:   PWM motori

Sottoscrive:
  /odom              — posizione e orientamento reali

Pubblica:
  /cmd_vel           — setpoint velocità verso STM32
  /pid_debug         — dati per grafici PlotJuggler
                       [x_target, x_real, y_target, y_real,
                        theta_target, theta_real, vlin, vang]

Comandi via /pid/command (std_msgs/String):
  goto:x,y,theta         — vai a posizione assoluta (cm, cm, rad)
  goto_rel:dist,angle    — movimento relativo (cm, rad)
  path:x1,y1,t1;x2,y2,t2;...  — sequenza waypoint
  stop                   — ferma il robot

Parametri configurabili (ros2 run ... --ros-args -p nome:=valore):
  kp_lin, ki_lin         — guadagni PID velocità lineare
  kp_ang, ki_ang         — guadagni PID velocità angolare
  acc_lin                — accelerazione lineare (cm/s²)
  dec_lin                — decelerazione lineare (cm/s²)
  vmax_lin               — velocità lineare massima (cm/s)
  vmax_ang               — velocità angolare massima (deg/s)
  angle_thresh           — soglia allineamento angolare (rad)
  reach_thresh           — soglia raggiungimento waypoint (cm)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String


# ── Classe PID ─────────────────────────────────────────────────────────────────

class PID:
    """
    Controller PID (Proporzionale-Integrale) con limite sull'output e sull'integrale.
    Usato per l'anello esterno di controllo della posizione.
    limit: valore massimo assoluto dell'output e dell'integrale (anti-windup).
    """
    def __init__(self, kp=1.0, ki=0.0, limit=None):
        self.kp    = kp
        self.ki    = ki
        self.limit = limit

        self._integral  = 0.0
        self._prev_time = None

    def reset(self):
        """Azzera l'integrale e il timestamp — chiamare prima di ogni nuovo percorso."""
        self._integral  = 0.0
        self._prev_time = None

    def update(self, setpoint: float, feedback: float, now: float) -> float:
        """
        Aggiorna il PID e restituisce l'output.
        setpoint: valore desiderato, feedback: valore misurato, now: timestamp (s).
        """
        err = setpoint - feedback
        dt  = (now - self._prev_time) if self._prev_time is not None else 0.0
        self._prev_time = now

        p = self.kp * err

        if dt > 0:
            self._integral += err * dt
            if self.limit:
                self._integral = max(-self.limit, min(self.limit, self._integral))
        i = self.ki * self._integral

        output = p + i
        if self.limit:
            output = max(-self.limit, min(self.limit, output))
        return output


# ── Generatore profilo trapezoidale ────────────────────────────────────────────

class TrapezoidProfile:
    """
    Genera un profilo di velocità trapezoidale in base alla distanza da percorrere.
    rampa salita → velocità costante → rampa discesa
    """

    def __init__(self, v_max: float, acc: float, dec: float):
        self.v_max_cfg = v_max
        self.acc       = acc
        self.dec       = dec
        self._active   = False
        self._t_start  = None
        self._t_acc    = 0.0
        self._t_const  = 0.0
        self._t_dec    = 0.0
        self._t_total  = 0.0
        self._v_peak   = 0.0

    def reset(self):
        self._active  = False
        self._t_start = None

    def start(self, now: float, distance: float):
        self._t_start = now
        self._active  = True
        distance      = abs(distance)

        t_acc   = self.v_max_cfg / self.acc
        d_ramp  = 0.5 * self.v_max_cfg * t_acc

        if 2 * d_ramp >= distance:
            # Profilo triangolare — non raggiunge v_max
            self._v_peak  = math.sqrt(distance * self.acc)
            self._t_acc   = self._v_peak / self.acc
            self._t_dec   = self._v_peak / self.dec
            self._t_const = 0.0
        else:
            self._v_peak  = self.v_max_cfg
            self._t_acc   = t_acc
            self._t_dec   = self.v_max_cfg / self.dec
            d_const       = distance - 2 * d_ramp
            self._t_const = d_const / self.v_max_cfg

        self._t_total = self._t_acc + self._t_const + self._t_dec

    def get_setpoint(self, now: float) -> float:
        if not self._active or self._t_start is None:
            return 0.0

        t = now - self._t_start

        if t >= self._t_total:
            self._active = False
            return 0.0

        if t < self._t_acc:
            return self._v_peak * (t / self._t_acc)
        elif t < self._t_acc + self._t_const:
            return self._v_peak
        else:
            t_dec = t - self._t_acc - self._t_const
            return self._v_peak * (1.0 - t_dec / self._t_dec)

    @property
    def active(self):
        return self._active


# ── Macchina a stati ───────────────────────────────────────────────────────────

class State:
    """
    Macchina a stati del controller PID.
    IDLE → ROTATING → MOVING → ADJUSTING → IDLE
    """
    IDLE      = "IDLE"
    ROTATING  = "ROTATING"    # fase 1: ruota verso il target
    MOVING    = "MOVING"      # fase 2: avanza verso il target
    ADJUSTING = "ADJUSTING"   # fase 3: corregge theta finale


# ── Nodo ROS2 ──────────────────────────────────────────────────────────────────

class AlvikPIDController(Node):

    def __init__(self):
        super().__init__('alvik_pid_controller')

        # ── Parametri ─────────────────────────────────────────────────────────
        self.declare_parameter('kp_lin',       1.5)
        self.declare_parameter('ki_lin',       0.8)
        self.declare_parameter('ki_ang',       0.0)
        self.declare_parameter('kp_ang',       25.0)
        self.declare_parameter('acc_lin',      3.0)
        self.declare_parameter('dec_lin',      2.0)
        self.declare_parameter('vmax_lin',     12.0)
        self.declare_parameter('vmax_ang',     60.0)
        self.declare_parameter('angle_thresh', 0.07)
        self.declare_parameter('reach_thresh', 1.0)

        kp_lin       = self.get_parameter('kp_lin').value
        ki_lin       = self.get_parameter('ki_lin').value
        kp_ang       = self.get_parameter('kp_ang').value
        ki_ang       = self.get_parameter('ki_ang').value
        acc_lin      = self.get_parameter('acc_lin').value
        dec_lin      = self.get_parameter('dec_lin').value
        vmax_lin     = self.get_parameter('vmax_lin').value
        vmax_ang     = self.get_parameter('vmax_ang').value
        self.angle_thresh = self.get_parameter('angle_thresh').value
        self.reach_thresh = self.get_parameter('reach_thresh').value

        # ── PID e profilo ─────────────────────────────────────────────────────
        self.pid_lin     = PID(kp_lin, ki_lin, limit=vmax_lin)
        self.pid_ang     = PID(kp_ang, ki_ang, limit=vmax_ang)
        self.profile_lin = TrapezoidProfile(vmax_lin, acc_lin, dec_lin)
        self.vmax_ang    = vmax_ang

        # ── Stato robot ───────────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # ── Stato macchina ────────────────────────────────────────────────────
        self.state        = State.IDLE
        self.waypoints    = []   # lista di dict {"x", "y", "theta"}
        self.wp_index     = 0
        self.target       = None

        # ── Publisher ─────────────────────────────────────────────────────────
        self.cmd_pub   = self.create_publisher(Twist,             '/cmd_vel',   10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/pid_debug', 10)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.cmd_sub  = self.create_subscription(
            String, '/pid/command', self.command_callback, 10)

        # ── Timer controllo 20Hz ──────────────────────────────────────────────
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"AlvikPIDController avviato\n"
            f"  PID lin: kp={kp_lin} ki={ki_lin} vmax={vmax_lin}cm/s\n"
            f"  PID ang: kp={kp_ang} ki={ki_ang} vmax={vmax_ang}deg/s\n"
            f"  Profilo: acc={acc_lin}cm/s² dec={dec_lin}cm/s²\n"
            f"  Soglie: angle={self.angle_thresh}rad reach={self.reach_thresh}cm"
        )

    # ── Callback odom ─────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        """
        Callback /odom — aggiorna posizione e orientamento correnti.
        Converte posizione da metri a cm e theta dal quaternione ROS2.
        """
        self.x = msg.pose.pose.position.x * 100.0   # m → cm
        self.y = msg.pose.pose.position.y * 100.0
        # Theta dal quaternione
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2.0 * math.atan2(qz, qw)

    # ── Callback comando ──────────────────────────────────────────────────────

    def command_callback(self, msg: String):
        """
        Callback /pid/command — parsa e avvia il movimento.
        Formati supportati: goto:x,y,theta | goto_rel:dist,angle | path:... | stop
        """
        cmd = msg.data.strip()
        self.get_logger().info(f"Comando ricevuto: {cmd}")

        if cmd.lower() == "stop":
            self._stop()
            return

        if cmd.lower().startswith("goto:"):
            # goto:x,y,theta  (cm, cm, rad)
            try:
                parts = cmd[5:].split(',')
                x     = float(parts[0])
                y     = float(parts[1])
                theta = float(parts[2])
                self._start_path([{"x": x, "y": y, "theta": theta}])
            except Exception as e:
                self.get_logger().error(f"Errore goto: {e}")

        elif cmd.lower().startswith("goto_rel:"):
            # goto_rel:dist,angle  (cm, rad)
            try:
                parts = cmd[9:].split(',')
                dist  = float(parts[0])
                angle = float(parts[1])
                tx    = self.x + dist * math.cos(self.theta + angle)
                ty    = self.y + dist * math.sin(self.theta + angle)
                tt    = self.theta + angle
                self._start_path([{"x": tx, "y": ty, "theta": tt}])
            except Exception as e:
                self.get_logger().error(f"Errore goto_rel: {e}")

        elif cmd.lower().startswith("path:"):
            # path:x1,y1,t1;x2,y2,t2;...
            try:
                wps = []
                for wp_str in cmd[5:].split(';'):
                    parts = wp_str.strip().split(',')
                    wps.append({
                        "x":     float(parts[0]),
                        "y":     float(parts[1]),
                        "theta": float(parts[2]),
                    })
                self._start_path(wps)
            except Exception as e:
                self.get_logger().error(f"Errore path: {e}")

    # ── Avvio percorso ────────────────────────────────────────────────────────

    def _start_path(self, waypoints: list):
        """Inizializza la lista waypoint e avvia la navigazione dal primo punto."""
        self.waypoints = waypoints
        self.wp_index  = 0
        self._next_waypoint()

    def _next_waypoint(self):
        """
        Avanza al waypoint successivo nella lista.
        Se già allineato al target salta la fase ROTATING ed entra in MOVING.
        Chiama _stop() quando tutti i waypoint sono stati raggiunti.
        """
        if self.wp_index >= len(self.waypoints):
            self.get_logger().info("Percorso completato")
            self._stop()
            return

        self.target = self.waypoints[self.wp_index]
    
        # Se già allineato al target salta la rotazione
        angle_err = self._angle_to_target()
        if abs(angle_err) < self.angle_thresh:
            print("Già Aliineato\n")
            self._enter_moving()
        else:
            self._enter_rotating()

    def _enter_rotating(self):
        """Entra in fase ROTATING: azzera PID angolare e imposta lo stato."""
        self.state = State.ROTATING
        self.pid_ang.reset()

    def _enter_moving(self):
        """
        Entra in fase MOVING: azzera PID lineare e avvia il profilo trapezoidale.
        Il profilo calcola la distanza residua e genera la rampa di velocità.
        """
        self.state = State.MOVING
        self.pid_lin.reset()
        self.profile_lin.reset()
        now  = self.get_clock().now().nanoseconds / 1e9
        dist = self._dist_to_target()
        self.profile_lin.start(now, distance=dist)

    def _enter_adjusting(self):
        """Entra in fase ADJUSTING: corregge il theta finale al waypoint."""
        self.state = State.ADJUSTING
        self.pid_ang.reset()

    def _stop(self):
        """Ferma il robot: imposta IDLE, azzera waypoint e PID, invia velocità zero."""
        self.state     = State.IDLE
        self.waypoints = []
        self.wp_index  = 0
        self.target    = None
        self.profile_lin.reset()
        self.pid_lin.reset()
        self.pid_ang.reset()
        self._send_cmd(0.0, 0.0)

    # ── Utilità ───────────────────────────────────────────────────────────────

    def _dist_to_target(self) -> float:
        """Distanza euclidea in cm dalla posizione corrente al target."""
        dx = self.target["x"] - self.x
        dy = self.target["y"] - self.y
        return math.sqrt(dx * dx + dy * dy)

    def _angle_to_target(self) -> float:
        """Errore angolare normalizzato in [-π, +π] verso la direzione del target."""
        dx  = self.target["x"] - self.x
        dy  = self.target["y"] - self.y
        ang = math.atan2(dy, dx)
        return self._normalize_angle(ang - self.theta)

    def _normalize_angle(self, a: float) -> float:
        """Normalizza un angolo nell'intervallo [-π, +π]."""
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _theta_error(self) -> float:
        """Errore angolare normalizzato tra theta corrente e theta target del waypoint."""
        return self._normalize_angle(self.target["theta"] - self.theta)

    # ── Loop di controllo ─────────────────────────────────────────────────────

    def control_loop(self):
        """
        Loop di controllo a 20Hz (timer ROS2, periodo 0.05s).
        Implementa la macchina a stati ROTATING → MOVING → ADJUSTING.
        Pubblica /cmd_vel e /pid_debug ad ogni ciclo.
        """
        if self.state == State.IDLE or self.target is None:
            self._publish_debug(0.0, 0.0, 0.0, 0.0)
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == State.ROTATING:
            angle_err = self._angle_to_target()
            dist      = self._dist_to_target()

            if dist < self.reach_thresh:
                # Già sul target — vai ad adjusting
                self._enter_adjusting()
                return

            if abs(angle_err) < self.angle_thresh:
                # Allineato — avanza
                self._enter_moving()
                return

            vang = max(-self.vmax_ang, min(self.vmax_ang,
                       math.degrees(angle_err) * self.pid_ang.kp))
            self._send_cmd(0.0, vang)
            self._publish_debug(
                self.target["x"], self.x,
                self.target["y"], self.y)

        elif self.state == State.MOVING:
            dist      = self._dist_to_target()
            angle_err = self._angle_to_target()

            if dist < self.reach_thresh:
                self._send_cmd(0.0, 0.0)
                self._enter_adjusting()
                return

            # Velocità lineare dal profilo trapezoidale
            sp_lin = self.profile_lin.get_setpoint(now)

            # Se il profilo è finito ma non siamo ancora arrivati
            # usa il PID di posizione
            if not self.profile_lin.active:
                sp_lin = min(self.pid_lin.update(dist, 0.0, now),
                             self.pid_lin.limit)

            # Correzione angolare durante il movimento
            vang = max(-self.vmax_ang * 0.5,
                       min(self.vmax_ang * 0.5,
                           math.degrees(angle_err) * self.pid_ang.kp * 0.3))

            self._send_cmd(sp_lin, vang)
            self._publish_debug(
                self.target["x"], self.x,
                self.target["y"], self.y)

        elif self.state == State.ADJUSTING:
            theta_err = self._theta_error()

            if abs(theta_err) < self.angle_thresh:
                # Waypoint completato
                self.get_logger().info(
                    f"Waypoint {self.wp_index + 1} raggiunto")
                self._send_cmd(0.0, 0.0)
                self.wp_index += 1
                self._next_waypoint()
                return

            vang = max(-self.vmax_ang, min(self.vmax_ang,
                       math.degrees(theta_err) * self.pid_ang.kp))
            self._send_cmd(0.0, vang)
            self._publish_debug(
                self.target["x"], self.x,
                self.target["y"], self.y)

    # ── Invio comandi ─────────────────────────────────────────────────────────

    def _send_cmd(self, vlin_cms: float, vang_degs: float):
        """
        Pubblica un Twist su /cmd_vel.
        Converte vlin da cm/s a m/s e vang da deg/s a rad/s (standard ROS2).
        """
        cmd = Twist()
        cmd.linear.x  = vlin_cms / 100.0          # cm/s → m/s
        cmd.angular.z = math.radians(vang_degs)   # deg/s → rad/s
        self.cmd_pub.publish(cmd)

    def _publish_debug(self, x_target, x_real, y_target, y_real):
        """
        Pubblica i dati di debug su /pid_debug per la visualizzazione in PlotJuggler.
        data = [x_target, x_real, y_target, y_real, theta_target_deg, theta_real_deg, vlin, vang]
        """
        debug = Float32MultiArray()
        debug.data = [
            float(x_target),
            float(x_real),
            float(y_target),
            float(y_real),
            float(math.degrees(self.target["theta"])) if self.target else 0.0,
            float(math.degrees(self.theta)),
            float(0.0),   # vlin placeholder
            float(0.0),   # vang placeholder
        ]
        self.debug_pub.publish(debug)


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AlvikPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
