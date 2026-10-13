"""Schickt eine künstliche Punktwolke (Raum mit Wänden, Boden und einer Säule) im EPCL-Format.

So lässt sich der Empfänger in der Bodenstation testen, ohne Drohne und ROS.
Start (aus dem Ordner ros2/ender_lidar_bridge):
    python3 tools/fake_sender.py --ip 192.168.144.20 --port 19900 --hz 5
"""

import argparse
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ender_lidar_bridge.protocol import encode_frame  # noqa: E402


def room(rng: np.random.Generator, n: int, yaw: float) -> np.ndarray:
    """Raum 8 x 6 x 3 m um die Drohne, dazu eine Säule. Die Drohne dreht sich langsam (yaw)."""
    pts = []
    for _ in range(n):
        a = rng.uniform(0, 2 * math.pi)
        z = rng.uniform(-1.2, 1.8)
        dx, dy = math.cos(a), math.sin(a)
        t = min(4.0 / abs(dx) if dx else 1e9, 3.0 / abs(dy) if dy else 1e9)
        pts.append((dx * t, dy * t, z))
    for _ in range(n // 3):  # Boden
        pts.append((rng.uniform(-4, 4), rng.uniform(-3, 3), -1.2))
    for _ in range(n // 6):  # Säule bei (2, 1)
        a = rng.uniform(0, 2 * math.pi)
        pts.append((2 + 0.3 * math.cos(a), 1 + 0.3 * math.sin(a), rng.uniform(-1.2, 1.8)))
    p = np.array(pts) + rng.normal(0, 0.01, (len(pts), 3))
    c, s = math.cos(-yaw), math.sin(-yaw)
    return p @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]).T


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=19900)
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--points", type=int, default=3000)
    args = ap.parse_args()
    rng = np.random.default_rng(0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq, t0 = 0, time.monotonic()
    print(f"Sende nach {args.ip}:{args.port} mit {args.hz} Hz, Strg+C beendet")
    while True:
        yaw = (time.monotonic() - t0) * 0.3
        for packet in encode_frame(room(rng, args.points, yaw), seq, int(time.monotonic() * 1000)):
            sock.sendto(packet, (args.ip, args.port))
        seq += 1
        time.sleep(1.0 / args.hz)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
