"""Prüfung und Umrechnung eines Flugziels. Kein ROS-Import, damit alles ohne ROS getestet werden kann.

Das Ziel kommt als Punkt im Koordinatensystem der Drohne (x vorn, y links, z oben), so wie
die Punktwolke. SUPER erwartet das Weltsystem von FAST-LIO (camera_init). Die Umrechnung
braucht die Pose der Drohne zu dem Zeitpunkt, an dem der Scan entstand, in dem geklickt wurde,
nicht die aktuelle Pose, denn die Drohne ist seitdem weitergeflogen.
"""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

from .goal_protocol import (
    STATUS_BAD_VALUE, STATUS_NO_POSE, STATUS_OK, STATUS_STALE, STATUS_TOO_FAR, Goal, is_finite,
)

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w

_WRAP = 1 << 32


@dataclass
class Pose:
    stamp_ms: int  # volle Millisekunden (nicht auf 32 Bit gekürzt)
    position: Vec3
    orientation: Quat


class PoseHistory:
    """Kurzer Verlauf der Drohnenposen (aus /Odometry), um die Pose zum Scan-Zeitpunkt zu finden."""

    def __init__(self, maxlen: int = 400) -> None:
        self._poses: Deque[Pose] = deque(maxlen=maxlen)

    def add(self, pose: Pose) -> None:
        self._poses.append(pose)

    @property
    def latest_ms(self) -> Optional[int]:
        return self._poses[-1].stamp_ms if self._poses else None

    def nearest(self, stamp_ms: int) -> Optional[Pose]:
        if not self._poses:
            return None
        return min(self._poses, key=lambda p: abs(p.stamp_ms - stamp_ms))


@dataclass
class Decision:
    status: int
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0


def unwrap_stamp(stamp32: int, reference_ms: int) -> int:
    """Macht aus dem auf 32 Bit gekürzten Scan-Zeitstempel wieder volle Millisekunden,
    und zwar den Wert, der am nächsten an reference_ms liegt."""
    diff = (reference_ms - stamp32) & (_WRAP - 1)
    if diff >= _WRAP // 2:
        diff -= _WRAP
    return reference_ms - diff


def rotate(q: Quat, v: Vec3) -> Vec3:
    """Dreht den Vektor v mit dem Quaternion q (x, y, z, w)."""
    qx, qy, qz, qw = q
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if n == 0.0:
        return v
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    vx, vy, vz = v
    # t = 2 * (q_vec x v);  v' = v + w * t + q_vec x t
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def body_to_world(pose: Pose, x: float, y: float, z: float = 0.0) -> Vec3:
    rx, ry, rz = rotate(pose.orientation, (x, y, z))
    px, py, pz = pose.position
    return (rx + px, ry + py, rz + pz)


def evaluate_goal(
    goal: Goal,
    history: PoseHistory,
    max_distance: float = 40.0,
    max_age_ms: int = 3000,
    max_pose_gap_ms: int = 300,
) -> Decision:
    """Entscheidet, ob ein Ziel angenommen wird, und rechnet es ins Weltsystem um."""
    if not is_finite(goal.x, goal.y):
        return Decision(STATUS_BAD_VALUE)
    if (goal.x * goal.x + goal.y * goal.y) ** 0.5 > max_distance:
        return Decision(STATUS_TOO_FAR)
    latest = history.latest_ms
    if latest is None:
        return Decision(STATUS_NO_POSE)
    scan_ms = unwrap_stamp(goal.scan_stamp_ms, latest)
    if latest - scan_ms > max_age_ms:
        return Decision(STATUS_STALE)
    pose = history.nearest(scan_ms)
    if pose is None or abs(pose.stamp_ms - scan_ms) > max_pose_gap_ms:
        return Decision(STATUS_NO_POSE)
    wx, wy, wz = body_to_world(pose, goal.x, goal.y)
    return Decision(STATUS_OK, wx, wy, wz)
