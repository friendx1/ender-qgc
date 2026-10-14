"""ROS2-Node: liest die Punktwolke von FAST-LIO2, dünnt sie aus und schickt sie per UDP zur Funke.

Start:
    ros2 run ender_lidar_bridge bridge_node --ros-args -p dest_ip:=192.168.144.20
"""

import socket
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from .downsample import reduce_points
from .protocol import encode_frame


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("ender_lidar_bridge")
        self.declare_parameter("topic", "/cloud_registered_body")
        self.declare_parameter("dest_ip", "192.168.144.20")
        self.declare_parameter("dest_port", 19900)
        self.declare_parameter("voxel_size", 0.10)   # Meter, ein Punkt je Würfel dieser Kantenlänge
        self.declare_parameter("max_points", 6000)    # Obergrenze je Scan
        self.declare_parameter("max_rate_hz", 5.0)    # höchstens so viele Scans pro Sekunde senden
        self.declare_parameter("max_range", 40.0)     # Meter, weiter entfernte Punkte werden verworfen
        self.declare_parameter("send_intensity", False)
        self.declare_parameter("packet_gap_ms", 1.0)  # Pause zwischen den Paketen eines Scans (WLAN verliert sonst Bursts)

        p = self.get_parameter
        self._dest = (p("dest_ip").value, int(p("dest_port").value))
        self._voxel = float(p("voxel_size").value)
        self._max_points = int(p("max_points").value)
        self._min_interval = 1.0 / max(0.1, float(p("max_rate_hz").value))
        self._max_range = float(p("max_range").value)
        self._intensity = bool(p("send_intensity").value)
        self._packet_gap = max(0.0, float(p("packet_gap_ms").value)) / 1000.0

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = 0
        self._last_sent = 0.0
        self._sent_bytes = 0
        self._sent_frames = 0
        self._stats_at = time.monotonic()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointCloud2, p("topic").value, self._on_cloud, qos)
        self.get_logger().info(
            f"Sende {p('topic').value} nach {self._dest[0]}:{self._dest[1]} "
            f"(Voxel {self._voxel} m, max {self._max_points} Punkte, {1 / self._min_interval:.1f} Hz)"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        if now - self._last_sent < self._min_interval:
            return
        self._last_sent = now

        names = ["x", "y", "z"] + (["intensity"] if self._intensity else [])
        raw = point_cloud2.read_points(msg, field_names=names, skip_nans=True)
        pts = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float32)
        inten = raw["intensity"] if self._intensity else None
        pts, inten = reduce_points(pts, self._voxel, self._max_points, self._max_range, inten)

        stamp_ms = int(msg.header.stamp.sec * 1000 + msg.header.stamp.nanosec // 1_000_000)
        packets = encode_frame(pts, self._seq, stamp_ms, inten)
        self._seq += 1
        try:
            for index, packet in enumerate(packets):
                if index and self._packet_gap:
                    time.sleep(self._packet_gap)
                self._sock.sendto(packet, self._dest)
                self._sent_bytes += len(packet)
        except OSError as e:
            self.get_logger().warning(f"Senden fehlgeschlagen: {e}", throttle_duration_sec=5.0)
            return
        self._sent_frames += 1

        elapsed = now - self._stats_at
        if elapsed >= 5.0:
            self.get_logger().info(
                f"{self._sent_frames / elapsed:.1f} Scans/s, {len(pts)} Punkte, "
                f"{self._sent_bytes * 8 / elapsed / 1e6:.2f} Mbit/s"
            )
            self._stats_at, self._sent_bytes, self._sent_frames = now, 0, 0


def main() -> None:
    rclpy.init()
    node = BridgeNode()
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
