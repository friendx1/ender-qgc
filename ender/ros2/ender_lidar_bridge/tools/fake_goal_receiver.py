"""Nimmt Flugziele (EGOL) an und bestätigt sie, ohne Drohne und ROS.

So lässt sich das Klicken in der Bodenstation am PC testen. Die Drohne steht dabei still
an einer festen Pose; Ziele werden mit derselben Logik geprüft und umgerechnet wie auf dem Onboard-PC.
Start (aus dem Ordner ros2/ender_lidar_bridge):
    python3 tools/fake_goal_receiver.py --port 19901
"""

import argparse
import math
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ender_lidar_bridge.goal_logic import Pose, PoseHistory, evaluate_goal  # noqa: E402
from ender_lidar_bridge.goal_protocol import STATUS_OK, Ack, decode_goal, encode_ack  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=19901)
    ap.add_argument("--pose", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("X", "Y", "YAW_DEG"),
                    help="feste Pose der Drohne im Weltsystem")
    args = ap.parse_args()

    yaw = math.radians(args.pose[2])
    orientation = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"Warte auf Ziele auf UDP {args.port}, Drohne steht bei {args.pose[:2]} mit Kurs {args.pose[2]} Grad")
    while True:
        data, sender = sock.recvfrom(256)
        goal = decode_goal(data)
        if goal is None:
            continue
        # Die Drohne steht still: Pose zum Scan-Zeitpunkt ist immer dieselbe, der Scan gilt als frisch.
        history = PoseHistory()
        now_ms = int(time.time() * 1000)
        history.add(Pose(now_ms, (args.pose[0], args.pose[1], 0.0), orientation))
        goal.scan_stamp_ms = now_ms & 0xFFFFFFFF
        decision = evaluate_goal(goal, history)
        print(f"Ziel {goal.goal_seq}: Drohne ({goal.x:.2f}, {goal.y:.2f}) m "
              f"-> Status {decision.status}, Welt ({decision.world_x:.2f}, {decision.world_y:.2f}) m"
              + ("" if decision.status == STATUS_OK else "  (abgelehnt)"))
        sock.sendto(encode_ack(Ack(decision.status, goal.goal_seq, decision.world_x, decision.world_y)), sender)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
