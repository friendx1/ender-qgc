"""Spielt die Relay-Node nach: ein TCP-Server, der eine künstliche Hindernis-Karte (EMAP v1) schickt.

So lässt sich der Karten-Empfänger in der Bodenstation testen, ohne Drohne, SUPER und ROS.
Die Drohne fliegt dabei langsam im Kreis durch einen Raum mit Hülle um die Wände und eine Säule.
Braucht: pip install zstandard numpy
Start (aus dem Ordner ros2/ender_lidar_bridge):
    python3 tools/fake_map_server.py --port 19902
Dazu kann tools/fake_sender.py den Scan schicken (dort dreht sich die Drohne nur auf der Stelle,
die Karte passt dann nicht zum Scan, zum Prüfen des Empfangs reicht es).
"""

import argparse
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ender_lidar_bridge.compressed_cloud import pack_cloud  # noqa: E402
from ender_lidar_bridge.map_protocol import OutQueue, encode_map, encode_pose  # noqa: E402

LEAF = 0.4  # Rasterweite wie in config/inf_map_compressor.yaml


def inflated_room() -> np.ndarray:
    """Hülle (Sicherheitsabstand) um einen Raum von 16 x 12 m und um eine Säule, im Weltsystem."""
    pts = []
    for z in np.arange(0.0, 3.0 + 1e-6, LEAF):
        for x in np.arange(-8.0, 8.0 + 1e-6, LEAF):
            pts += [(x, -6.0, z), (x, 6.0, z)]
        for y in np.arange(-6.0, 6.0 + 1e-6, LEAF):
            pts += [(-8.0, y, z), (8.0, y, z)]
        for a in np.arange(0, 2 * math.pi, 0.5):  # Säule bei (3, 2), Radius 0.8 m
            pts.append((3 + 0.8 * math.cos(a), 2 + 0.8 * math.sin(a), z))
    return np.array(pts)


def stress_cloud(count: int, leaf: float, seed: int = 1) -> np.ndarray:
    """Lastprobe: `count` Punkte auf den Flächen vieler Kisten im Umkreis von 20 m, gerastert wie die echte Karte.

    Die Punkte liegen auf Raster `leaf` und sind ohne Doppelte, wie bei einer Belegungskarte. Es kommen etwas
    weniger als `count` heraus, wenn Kisten sich überdecken.
    """
    rng = np.random.default_rng(seed)
    out = np.empty((0, 3))
    while len(out) < count:
        centre = rng.uniform(-18.0, 18.0, 2)
        size = rng.uniform(1.0, 6.0, 3)
        n = 4000
        pts = rng.uniform(-0.5, 0.5, (n, 3))
        face = rng.integers(0, 3, n)                              # eine Koordinate auf die Fläche legen
        pts[np.arange(n), face] = rng.choice([-0.5, 0.5], n)
        pts = pts * size + np.array([centre[0], centre[1], size[2] / 2])
        pts = np.round(pts / leaf) * leaf
        out = np.unique(np.vstack([out, pts]), axis=0)
    out = out[rng.permutation(len(out))[:count]]
    return out


def pose_at(t: float):
    """Kreis mit 3 m Radius, die Drohne schaut in Flugrichtung, Höhe 1,5 m."""
    w = 0.25  # rad/s
    x, y = 3.0 * math.cos(w * t), 3.0 * math.sin(w * t)
    yaw = w * t + math.pi / 2
    return (x, y, 1.5), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=19902)
    ap.add_argument("--map-hz", type=float, default=1.0)
    ap.add_argument("--pose-hz", type=float, default=10.0)
    ap.add_argument("--points", type=int, default=0,
                    help="Lastprobe: Karte mit so vielen Punkten (z. B. 20000, 100000) statt des kleinen Raums")
    ap.add_argument("--grow", type=float, default=0.0,
                    help="Lastprobe: Punkte pro Sekunde, um die die Karte bis --points wächst (0 = gleich voll)")
    ap.add_argument("--leaf", type=float, default=0.2, help="Raster der Lastprobe in Metern (Standard 0,2)")
    args = ap.parse_args()

    full = stress_cloud(args.points, args.leaf) if args.points > 0 else inflated_room()
    cloud = full
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", args.port))
    server.listen(1)
    print(f"Warte auf die Bodenstation auf TCP {args.port} ({len(cloud)} Punkte), Strg+C beendet")

    while True:
        client, address = server.accept()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        client.settimeout(5.0)
        print(f"Bodenstation {address[0]} verbunden")
        queue, t0, next_map, next_pose = OutQueue(), time.monotonic(), 0.0, 0.0
        try:
            while True:
                t = time.monotonic() - t0
                if t >= next_pose:
                    stamp_ms = int(time.time() * 1000)
                    queue.put_pose(encode_pose(stamp_ms, *pose_at(t)))
                    next_pose = t + 1.0 / args.pose_hz
                if t >= next_map:
                    if args.points > 0 and args.grow > 0:
                        cloud = full[: max(1000, min(len(full), int(1000 + args.grow * t)))]
                    now = time.time()
                    blob = pack_cloud(cloud, int(now), int((now % 1) * 1e9))
                    queue.put_map(encode_map(blob))
                    if args.points > 0:
                        print(f"Karte: {len(cloud)} Punkte, {len(blob) / 1000:.0f} kB", end="\r", flush=True)
                    next_map = t + 1.0 / args.map_hz
                pending = queue.peek()
                if len(pending):
                    queue.consume(client.send(pending))
                else:
                    time.sleep(0.01)
        except (OSError, socket.timeout) as e:
            print(f"Verbindung beendet: {e}")
            client.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
