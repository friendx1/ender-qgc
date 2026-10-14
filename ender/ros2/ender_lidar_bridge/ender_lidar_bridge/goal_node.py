"""ROS2-Node: nimmt Flugziele von der Bodenstation per UDP an und gibt sie als PoseStamped an den Planer.

Das Ziel kommt als Punkt im Koordinatensystem der Drohne. Die Node rechnet ihn mit der Pose
zum Scan-Zeitpunkt (aus /Odometry) ins Weltsystem um, prüft ihn und antwortet mit einer
Bestätigung an den Absender. Format: siehe goal_protocol.py (EGOL v1).

Start:
    ros2 run ender_lidar_bridge goal_node
"""

import socket

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .goal_logic import PoseHistory, Pose, evaluate_goal
from .goal_protocol import STATUS_OK, Ack, decode_goal, encode_ack

_RECV_BYTES = 256


class GoalNode(Node):
    def __init__(self) -> None:
        super().__init__("ender_goal_node")
        self.declare_parameter("listen_port", 19901)
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("goal_frame", "camera_init")
        self.declare_parameter("goal_z", 1.5)            # SUPER überschreibt z mit fsm/click_height
        self.declare_parameter("max_distance", 40.0)     # Meter von der Drohne zum Ziel
        self.declare_parameter("max_age_s", 20.0)        # so alt darf der Scan beim Senden höchstens sein (er ist beim Wählen eingefroren)
        self.declare_parameter("max_pose_gap_s", 0.3)    # größter Abstand zwischen Scan und nächster Pose

        p = self.get_parameter
        self._frame = str(p("goal_frame").value)
        self._goal_z = float(p("goal_z").value)
        self._max_distance = float(p("max_distance").value)
        self._max_age_ms = int(float(p("max_age_s").value) * 1000)
        self._max_gap_ms = int(float(p("max_pose_gap_s").value) * 1000)

        self._history = PoseHistory()
        self._last_seq = None
        self._last_ack = None

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", int(p("listen_port").value)))
        self._sock.setblocking(False)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(Odometry, p("odom_topic").value, self._on_odom, qos)
        self._goal_pub = self.create_publisher(PoseStamped, p("goal_topic").value, 10)
        self.create_timer(0.02, self._poll_socket)
        self.get_logger().info(
            f"Warte auf Ziele auf UDP {p('listen_port').value}, Ausgabe: {p('goal_topic').value} "
            f"(Frame {self._frame}, max {self._max_distance:.0f} m)"
        )

    def _on_odom(self, msg: Odometry) -> None:
        stamp_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000
        pos, ori = msg.pose.pose.position, msg.pose.pose.orientation
        self._history.add(Pose(stamp_ms, (pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w)))

    def _poll_socket(self) -> None:
        while True:
            try:
                data, sender = self._sock.recvfrom(_RECV_BYTES)
            except BlockingIOError:
                return
            except OSError as e:
                self.get_logger().warning(f"Empfang fehlgeschlagen: {e}", throttle_duration_sec=5.0)
                return
            goal = decode_goal(data)
            if goal is None:
                continue
            if goal.goal_seq == self._last_seq and self._last_ack is not None:
                # Wiederholung (Bestätigung ging verloren): nur noch einmal bestätigen, nicht neu setzen
                self._reply(self._last_ack, sender)
                continue
            decision = evaluate_goal(goal, self._history, self._max_distance,
                                     self._max_age_ms, self._max_gap_ms)
            ack = Ack(decision.status, goal.goal_seq, decision.world_x, decision.world_y)
            self._last_seq, self._last_ack = goal.goal_seq, ack
            if decision.status == STATUS_OK:
                self._publish(decision.world_x, decision.world_y)
                self.get_logger().info(
                    f"Ziel {goal.goal_seq}: Drohne ({goal.x:.2f}, {goal.y:.2f}) m "
                    f"-> Welt ({decision.world_x:.2f}, {decision.world_y:.2f}) m"
                )
            else:
                self.get_logger().warning(f"Ziel {goal.goal_seq} abgelehnt, Status {decision.status}")
            self._reply(ack, sender)

    def _publish(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = self._frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = self._goal_z
        msg.pose.orientation.w = 1.0
        self._goal_pub.publish(msg)

    def _reply(self, ack: Ack, sender) -> None:
        try:
            self._sock.sendto(encode_ack(ack), sender)
        except OSError as e:
            self.get_logger().warning(f"Bestätigung nicht gesendet: {e}", throttle_duration_sec=5.0)


def main() -> None:
    rclpy.init()
    node = GoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():  # Strg+C hat den Kontext unter Umständen schon beendet
            rclpy.shutdown()


if __name__ == "__main__":
    main()
