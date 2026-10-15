"""ROS2-Node: schickt die Hindernis-Karte (SUPER inflate map) per TCP zur Bodenstation.

Die Karte kommt schon gepackt von Birks pointcloud_compressor_node (Topic
/rog_map/inf_occ_compressed). Diese Node reicht das Paket unverändert weiter und schickt
dazu ein paar Mal pro Sekunde die Pose der Drohne, damit die Bodenstation die Karte
(Weltsystem) zur Drohne ausrichten kann. Format: siehe map_protocol.py (EMAP v1).

Die Node ist der Server, die Bodenstation verbindet sich. Solange niemand verbunden ist,
abonniert sie nichts: dann arbeitet auch Birks Kompressor nicht (er packt nur, wenn jemand zuhört).
Ist die Bodenstation verbunden, läuft die Pose (die freie Kamera braucht sie). Die Karte wird erst
abonniert, wenn die Bodenstation sie per Steuernachricht verlangt ("Hindernisse an").

Start (der Kompressor muss laufen):
    ros2 run ender_lidar_bridge map_relay_node
"""

import socket
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt8MultiArray

from .map_protocol import TYPE_CONTROL, OutQueue, StreamDecoder, decode_control, encode_map, encode_pose

_SEND_CHUNK = 64 * 1024


class MapRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("ender_map_relay")
        self.declare_parameter("listen_port", 19902)
        self.declare_parameter("map_topic", "/rog_map/inf_occ_compressed")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("map_rate_hz", 1.0)    # höchstens so viele Karten pro Sekunde weitergeben
        self.declare_parameter("pose_rate_hz", 10.0)  # Posen pro Sekunde

        p = self.get_parameter
        self._map_topic = str(p("map_topic").value)
        self._odom_topic = str(p("odom_topic").value)
        self._min_map_interval = 1.0 / max(0.05, float(p("map_rate_hz").value))
        pose_interval = 1.0 / max(0.5, float(p("pose_rate_hz").value))

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("0.0.0.0", int(p("listen_port").value)))
        self._server.listen(1)
        self._server.setblocking(False)

        self._client: Optional[socket.socket] = None
        self._queue = OutQueue()
        self._decoder = StreamDecoder()
        self._want_map = False
        self._map_sub = None
        self._odom_sub = None
        self._latest_odom: Optional[Odometry] = None
        self._last_map_at = 0.0

        self._sent_bytes = 0
        self._stats_at = time.monotonic()

        self.create_timer(0.02, self._service)
        self.create_timer(pose_interval, self._queue_pose)
        self.get_logger().info(
            f"Warte auf die Bodenstation auf TCP {p('listen_port').value} "
            f"(Karte {self._map_topic} nur auf Wunsch, höchstens {1 / self._min_map_interval:.1f} Hz)"
        )

    # --- Verbindung -------------------------------------------------------------------

    def _service(self) -> None:
        self._accept()
        if self._client is not None:
            self._read_client()
        if self._client is not None:
            self._flush()
        self._log_stats()

    def _accept(self) -> None:
        try:
            client, address = self._server.accept()
        except BlockingIOError:
            return
        except OSError as e:
            self.get_logger().warning(f"Annehmen fehlgeschlagen: {e}", throttle_duration_sec=5.0)
            return
        if self._client is not None:
            self._drop_client("neue Verbindung ersetzt die alte")
        client.setblocking(False)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._client = client
        self._queue = OutQueue()
        self._decoder = StreamDecoder()
        self._latest_odom = None
        self._sent_bytes, self._stats_at = 0, time.monotonic()
        self._want_map = False
        self._subscribe_odom()
        self.get_logger().info(f"Bodenstation {address[0]} verbunden, sende die Pose (Karte nur auf Wunsch)")

    def _read_client(self) -> None:
        """Die Bodenstation schickt nur Steuernachrichten. Lesen wir 0 Bytes, hat sie getrennt."""
        try:
            data = self._client.recv(256)
        except BlockingIOError:
            return
        except OSError:
            self._drop_client("Verbindung abgebrochen")
            return
        if not data:
            self._drop_client("Bodenstation hat getrennt")
            return
        for msg_type, payload in self._decoder.feed(data):
            if msg_type == TYPE_CONTROL:
                want_map = decode_control(payload)
                if want_map is not None:
                    self._set_want_map(want_map)

    def _drop_client(self, reason: str) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except OSError:
                pass
        self._client = None
        self._want_map = False
        self._unsubscribe_map()
        self._unsubscribe_odom()
        self.get_logger().info(f"{reason}, stoppe den Abruf von Pose und Karte")

    # --- Abos nur solange jemand zuhört -------------------------------------------------

    def _subscribe_odom(self) -> None:
        # Best Effort, so wie Birks Kompressor veröffentlicht
        odom_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, odom_qos)

    def _unsubscribe_odom(self) -> None:
        if self._odom_sub is not None:
            self.destroy_subscription(self._odom_sub)
        self._odom_sub = None

    def _set_want_map(self, want_map: bool) -> None:
        if want_map == self._want_map:
            return
        self._want_map = want_map
        if want_map:
            map_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
            self._map_sub = self.create_subscription(UInt8MultiArray, self._map_topic, self._on_map, map_qos)
            self.get_logger().info(f"Karte verlangt, abonniere {self._map_topic}")
        else:
            self._unsubscribe_map()
            self.get_logger().info("Karte nicht mehr verlangt, stoppe den Abruf der Karte")

    def _unsubscribe_map(self) -> None:
        if self._map_sub is not None:
            self.destroy_subscription(self._map_sub)
        self._map_sub = None

    # --- Daten --------------------------------------------------------------------------

    def _on_map(self, msg: UInt8MultiArray) -> None:
        now = time.monotonic()
        if now - self._last_map_at < self._min_map_interval:
            return
        self._last_map_at = now
        blob = bytes(msg.data)
        if blob:
            self._queue.put_map(encode_map(blob))

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _queue_pose(self) -> None:
        if self._client is None or self._latest_odom is None:
            return
        msg = self._latest_odom
        pos, ori = msg.pose.pose.position, msg.pose.pose.orientation
        stamp_ms = msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000
        self._queue.put_pose(encode_pose(stamp_ms, (pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w)))

    def _flush(self) -> None:
        while True:
            pending = self._queue.peek()
            if not len(pending):
                return
            try:
                sent = self._client.send(pending[:_SEND_CHUNK])
            except BlockingIOError:
                return  # der Sendepuffer ist voll, beim nächsten Takt geht es weiter
            except OSError:
                self._drop_client("Senden fehlgeschlagen")
                return
            self._queue.consume(sent)
            self._sent_bytes += sent

    def _log_stats(self) -> None:
        now = time.monotonic()
        elapsed = now - self._stats_at
        if elapsed < 5.0 or self._client is None:
            return
        self.get_logger().info(
            f"{self._sent_bytes * 8 / elapsed / 1e6:.2f} Mbit/s zur Bodenstation, "
            f"{self._queue.dropped_maps} Karten übersprungen (Verbindung zu langsam)"
        )
        self._stats_at, self._sent_bytes = now, 0


def main() -> None:
    rclpy.init()
    node = MapRelayNode()
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
