"""
alvik_ros_bridge.py
===================
Nodo ROS2 — ponte tra Alvik (via UDP DDS) e ROS2.

Riceve da bridge_DDS via UDP :4445 (stesso formato DDS di Godot)
Pubblica:
  /odom          (nav_msgs/Odometry)
  /tf            (odom -> base_link)
  /scan          (sensor_msgs/LaserScan) — 5 punti ToF

Sottoscrive:
  /cmd_vel       (geometry_msgs/Twist) -> CMD UDP -> Alvik

Time Sync:
  Usa TickId come trigger e t_robot_ms come riferimento temporale.
  Al primo tick salva il riferimento ROS2/robot.
  I tick successivi usano t_robot_ms come delta per costruire
  il timestamp sincronizzato con il clock di Alvik.
"""

import math
import socket
import struct

import rclpy
import rclpy.duration
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32

# ── Configurazione ─────────────────────────────────────────────────────────────

DDS_LISTEN_PORT = 4445
ALVIK_IP        = "192.168.4.1"
ALVIK_PORT      = 5005

WHEEL_RADIUS_M  = 0.017
BASE_WIDTH_M    = 0.090

# ── Costanti DDS ───────────────────────────────────────────────────────────────

CMD_PUBLISH    = 0x82
DDS_TYPE_INT   = 1
DDS_TYPE_FLOAT = 2

# ── Nodo ROS2 ──────────────────────────────────────────────────────────────────

class AlvikRosBridge(Node):
    """
    Nodo ROS2 bridge tra il protocollo DDS custom di Alvik e i topic ROS2 standard.

    Riceve la telemetria da bridge_v1_7.py via UDP :4445 (formato DDS binario),
    la converte e pubblica su /odom, /tf, /scan.
    Sottoscrive /cmd_vel e inoltra i comandi ad Alvik via UDP :5005.

    Trasformazioni applicate al sistema di riferimento:
      - Posizione: negata (x_ros = -x_alvik, y_ros = -y_alvik)
      - Orientamento: theta_ros = theta_alvik + π
      - Velocità angolare: vth negata per compensare il verso di rotazione
    """

    def __init__(self):
        super().__init__('alvik_ros_bridge')

        # Publisher
        self.odom_pub       = self.create_publisher(Odometry,   '/odom', 10)
        self.scan_pub       = self.create_publisher(LaserScan,  '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Offset allineamento da Godot
        self.align_offset_theta = 0.0

        # Subscriber allineamento
        self.align_sub = self.create_subscription(
            Float32, '/alvik/alignment', self.alignment_callback, 10)

        # Socket UDP verso Alvik
        self.sock_alvik = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Socket UDP per ricevere dal bridge_DDS
        self.sock_dds = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_dds.bind(("0.0.0.0", DDS_LISTEN_PORT))
        self.sock_dds.setblocking(False)

        # Stato posa
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.vx    = 0.0
        self.vth   = 0.0

        # Buffer variabili DDS
        self.dds_vars = {}

        # Time Sync
        self.last_tick_id   = -1
        self.t_robot_ms_ref = None
        self._tof_history   = {}   # storico campioni ToF per filtro media mobile
        self.t_ros_ref      = None

        # Timer a 100Hz per drenare la coda UDP
        self.create_timer(0.01, self.update)

        self.get_logger().info("AlvikRosBridge avviato — in ascolto su UDP :4445")

    def update(self):
        """
        Callback timer a 100Hz — cuore del nodo.
        1. Drena la coda UDP DDS (tutti i pacchetti disponibili)
        2. Pubblica TF SEMPRE a 100Hz con l'ultima posizione nota
           (evita gap di trasformazione durante latenza WiFi)
        3. Se arriva un nuovo TickId: aggiorna posa, pubblica /odom e /scan
        Solo TF viene pubblicato senza nuovi dati — /odom e /scan
        vengono aggiornati solo quando arriva un tick nuovo.
        """
        self._drain_dds()

        tick_id    = self.dds_vars.get("TickId")
        t_robot_ms = self.dds_vars.get("t_robot_ms")

        now = self.get_clock().now().to_msg()
        # Pubblica TF a 100Hz anche senza nuovi dati WiFi
        if self.x is not None and self.y is not None and self.theta is not None:
            self._publish_tf(now)
        if tick_id is None or tick_id == self.last_tick_id:
            return
        if tick_id < 0:
            return

        self.last_tick_id = tick_id

        x_cm    = self.dds_vars.get("X")
        y_cm    = self.dds_vars.get("Y")
        theta   = self.dds_vars.get("Theta")
        wl_rads = self.dds_vars.get("WheelLeft")
        wr_rads = self.dds_vars.get("WheelRight")

        if x_cm is None or y_cm is None or theta is None:
            return

        self.x     = float(x_cm) * 0.01
        self.y     = float(y_cm) * 0.01
        self.theta = float(theta) + math.pi
        # self.theta = float(theta)

        if wl_rads is not None and wr_rads is not None:
            self.vx  = -((float(wr_rads) + float(wl_rads)) / 2.0 * WHEEL_RADIUS_M)
            # self.vth = (float(wr_rads) - float(wl_rads)) / BASE_WIDTH_M * WHEEL_RADIUS_M
            self.vth = -(float(wr_rads) - float(wl_rads)) / BASE_WIDTH_M * WHEEL_RADIUS_M # inverto la rotazione
        self._publish_odom(now)
        self._publish_tf(now)
        self._publish_scan(now)

    def _get_synced_stamp(self, t_robot_ms):
        """
        Calcola un timestamp ROS2 sincronizzato con il clock di Alvik.
        Al primo tick salva il riferimento ROS2/robot come ancora temporale.
        I tick successivi usano t_robot_ms come delta per ricostruire
        il timestamp corrispondente nel clock ROS2.
        Non usato attivamente — il clock ROS2 diretto è sufficiente.
        """
        now_ros = self.get_clock().now()
        if t_robot_ms is None:
            return now_ros.to_msg()
        if self.t_robot_ms_ref is None:
            self.t_robot_ms_ref = float(t_robot_ms)
            self.t_ros_ref      = now_ros
            return now_ros.to_msg()
        delta_ms = float(t_robot_ms) - self.t_robot_ms_ref
        delta_ns = int(delta_ms * 1_000_000)
        synced   = self.t_ros_ref + rclpy.duration.Duration(nanoseconds=delta_ns)
        return synced.to_msg()

    def _tof_filter(self, key: str, new_val: float) -> float:
        """
        Filtro media mobile + soglia per i sensori ToF.
        TOF_WINDOW=10 campioni (200ms), TOF_THRESHOLD=5cm (sperimentale).
        Se variazione < soglia → rumore → usa media mobile.
        Se variazione >= soglia → ostacolo reale → usa nuovo valore.
        """
        TOF_WINDOW    = 10
        TOF_THRESHOLD = 5.0  # cm
        if key not in self._tof_history:
            self._tof_history[key] = []
        history = self._tof_history[key]
        history.append(new_val)
        if len(history) > TOF_WINDOW:
            history.pop(0)
        avg = sum(history) / len(history)
        return avg if abs(new_val - avg) < TOF_THRESHOLD else new_val

    def _drain_dds(self):
        """
        Svuota tutta la coda UDP DDS in un singolo ciclo.
        BlockingIOError indica coda vuota (socket non bloccante) — uscita normale.
        Aggiorna dds_vars con i valori più recenti di ogni variabile.
        """
        while True:
            try:
                data, _ = self.sock_dds.recvfrom(256)
                name, value = self._parse_dds(data)
                if name:
                    self.dds_vars[name] = value
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f"DDS error: {e}")
                break

    def _parse_dds(self, data: bytes):
        """
        Decodifica un pacchetto DDS binario.
        Formato: [0x82][tipo][len_name][name...][valore 4 byte]
        Restituisce (name, value) o (None, None) se malformato.
        """
        if len(data) < 4:
            return None, None
        cmd  = data[0]
        typ  = data[1]
        nlen = data[2]
        if cmd != CMD_PUBLISH or len(data) < 3 + nlen + 4:
            return None, None
        name   = data[3:3 + nlen].decode()
        offset = 3 + nlen
        if typ == DDS_TYPE_FLOAT:
            value = struct.unpack_from('<f', data, offset)[0]
        elif typ == DDS_TYPE_INT:
            value = struct.unpack_from('<i', data, offset)[0]
        else:
            return None, None
        return name, value

    def _publish_odom(self, now):
        """
        Pubblica l'odometria su /odom (nav_msgs/Odometry).
        Applica le trasformazioni di sistema di riferimento:
          - posizione negata: x_ros = -x_alvik, y_ros = -y_alvik
          - orientamento: quaternione da theta_alvik + π + align_offset
          - velocità lineare negata, angolare diretta
        frame_id = 'odom', child_frame_id = 'base_link'.
        """
        theta = self.theta + self.align_offset_theta  # ← applica offset
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = -self.x
        odom.pose.pose.position.y    = -self.y
        odom.pose.pose.position.z    = 0.0
        # odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        # odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        odom.twist.twist.linear.x    = -self.vx
        odom.twist.twist.angular.z   = self.vth
        self.odom_pub.publish(odom)

    def _publish_tf(self, now):
        """
        Pubblica il TF tree a 100Hz su tf2.
        Pubblica due transform identiche necessarie per ROS2:
          - odom → base_link  (richiesto da Nav2 controller)
          - odom → base_footprint  (richiesto da slam_toolbox)
        Pubblicato continuamente anche senza nuovi dati WiFi per evitare
        il messaggio "Timed out waiting for transform".
        """
        theta = self.theta + self.align_offset_theta  # ← applica offset
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = -self.x
        t.transform.translation.y = -self.y
        t.transform.translation.z = 0.0
        # t.transform.rotation.z    = math.sin(self.theta / 2.0)
        t.transform.rotation.z    = math.sin(theta / 2.0)
        # t.transform.rotation.w    = math.cos(self.theta / 2.0)
        t.transform.rotation.w    = math.cos(theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # odom → base_footprint (necessario per slam_toolbox)
        t2 = TransformStamped()
        t2.header.stamp    = now
        t2.header.frame_id = 'odom'
        t2.child_frame_id  = 'base_footprint'
        t2.transform.translation.x = -self.x
        t2.transform.translation.y = -self.y
        t2.transform.translation.z = 0.0
        # t2.transform.rotation.z    = math.sin(self.theta / 2.0)
        t2.transform.rotation.z    = math.sin(theta / 2.0)
        #t2.transform.rotation.w    = math.cos(self.theta / 2.0)
        t2.transform.rotation.w    = math.cos(theta / 2.0)
        self.tf_broadcaster.sendTransform(t2)

    def _publish_scan(self, now):
        """
        Pubblica il laser scan su /scan (sensor_msgs/LaserScan).
        Converte le 5 letture ToF in un scan con 5 raggi a ±45° (passo 22.5°).
        Applica il filtro media mobile (_tof_filter) per ridurre il rumore.
        Valori fuori range [range_min, range_max] vengono sostituiti con inf.
        frame_id = 'base_footprint'.
        """
        tof_keys = ["ToF_R", "ToF_CR", "ToF_C", "ToF_CL", "ToF_L"]
        scan = LaserScan()
        scan.header.stamp    = now
        scan.header.frame_id = 'base_footprint'
        scan.angle_min       = math.radians(-45.0)
        scan.angle_max       = math.radians(45.0)
        scan.angle_increment = math.radians(22.5)
        scan.range_min       = 0.02
        scan.range_max       = 2.0
        scan.time_increment  = 0.0
        scan.scan_time       = 0.02
        ranges = []
        for key in tof_keys:
            val = self.dds_vars.get(key)
            if val is None:
                ranges.append(float('inf'))
            else:
                filtered_cm = self._tof_filter(key, float(val))
                dist_m = filtered_cm * 0.01
                if dist_m < scan.range_min or dist_m > scan.range_max:
                    ranges.append(float('inf'))
                else:
                    ranges.append(dist_m)
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback /cmd_vel — converte Twist ROS2 in comando UDP per Alvik.
        linear.x (m/s) → vlin (cm/s), clampato a ±20 cm/s.
        angular.z (rad/s) → vang (deg/s), clampato a ±90 deg/s.
        Invia "CMD,vlin,vang" via UDP a Alvik 192.168.4.1:5005.
        """
        vlin_cms  = max(-20.0, min(20.0,  msg.linear.x * 100.0))
        vang_degs = max(-90.0, min(90.0, math.degrees(msg.angular.z)))
        cmd = f"CMD,{vlin_cms:.2f},{vang_degs:.2f}".encode()
        self.sock_alvik.sendto(cmd, (ALVIK_IP, ALVIK_PORT))

    def alignment_callback(self, msg: Float32):
        """
        Callback /alvik/alignment — aggiorna l'offset angolare di allineamento.
        L'offset viene applicato a theta in _publish_odom() e _publish_tf().
        Nota: non usato attivamente in v1.7 — l'allineamento è gestito in Godot.
        """
        self.align_offset_theta = msg.data
        self.get_logger().info(f"Allineamento ricevuto: {math.degrees(msg.data):.1f}°")



def main(args=None):
    """Entry point ROS2 — inizializza il nodo e avvia il loop di spin."""
    rclpy.init(args=args)
    node = AlvikRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
