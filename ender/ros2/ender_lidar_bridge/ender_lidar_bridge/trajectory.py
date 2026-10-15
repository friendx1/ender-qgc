"""Bahn aus den Marker-Nachrichten von SUPER (visualization_msgs/MarkerArray) zusammensetzen.

Kein ROS-Import, damit alles ohne ROS getestet werden kann.

SUPER zeichnet seine Bahnen für RViz als viele kurze Pfeile (Marker vom Typ ARROW mit zwei Punkten,
Anfang und Ende), pro Bahn ein Marker mit eigener id. Jede Nachricht beginnt mit DELETEALL und
enthält dann die ganze Bahn neu. Hier wird derselbe Stand geführt wie in RViz (DELETEALL, ADD/MODIFY,
DELETE) und daraus eine Folge von Linien gemacht, die sich leicht übertragen lässt.
"""

import math
from typing import Dict, List, Sequence, Tuple

Point = Tuple[float, float, float]

ACTION_ADD = 0
ACTION_DELETE = 2
ACTION_DELETEALL = 3
TYPE_ARROW = 0
TYPE_LINE_STRIP = 4
TYPE_LINE_LIST = 5

JOIN_TOLERANCE = 1e-3  # m: ein Stück schließt an das vorige an, wenn sein Anfang so nah an dessen Ende liegt


def _finite(point: Point) -> bool:
    return all(math.isfinite(c) for c in point)


class MarkerTrajectory:
    """Der Stand einer Bahn, wie ihn RViz aus den Markern aufbauen würde."""

    def __init__(self) -> None:
        # (ns, id) -> (frame_id, Typ, Punkte)
        self._markers: Dict[Tuple[str, int], Tuple[str, int, List[Point]]] = {}
        self.dirty = False

    def clear(self) -> None:
        if self._markers:
            self.dirty = True
        self._markers.clear()

    def apply(self, action: int, ns: str, marker_id: int, marker_type: int, frame_id: str,
              points: Sequence[Point]) -> None:
        if action == ACTION_DELETEALL:
            self.clear()
        elif action == ACTION_DELETE:
            if self._markers.pop((ns, marker_id), None) is not None:
                self.dirty = True
        elif action == ACTION_ADD:
            if marker_type in (TYPE_ARROW, TYPE_LINE_STRIP, TYPE_LINE_LIST):
                self._markers[(ns, marker_id)] = (frame_id, marker_type, list(points))
                self.dirty = True

    def polylines(self) -> List[Tuple[str, List[Point]]]:
        """(Frame, Punkte) je Linie. Aufeinanderfolgende Pfeile gleichen Frames, die aneinander anschließen,
        werden zu einer Linie, in der Reihenfolge der ids."""
        lines: List[Tuple[str, List[Point]]] = []
        current_frame = None
        current: List[Point] = []

        def finish() -> None:
            nonlocal current, current_frame
            if len(current) >= 2:
                lines.append((current_frame, current))
            current, current_frame = [], None

        def add_segment(frame: str, start: Point, end: Point) -> None:
            nonlocal current, current_frame
            joins = (current and frame == current_frame and
                     math.dist(current[-1], start) <= JOIN_TOLERANCE)
            if not joins:
                finish()
                current, current_frame = [start], frame
            current.append(end)

        for key in sorted(self._markers, key=lambda k: (k[0], k[1])):
            frame, marker_type, points = self._markers[key]
            points = [p for p in points if _finite(p)]
            if marker_type == TYPE_ARROW:
                if len(points) == 2:
                    add_segment(frame, points[0], points[1])
            elif marker_type == TYPE_LINE_STRIP:
                finish()
                if len(points) >= 2:
                    lines.append((frame, points))
            elif marker_type == TYPE_LINE_LIST:
                for i in range(0, len(points) - 1, 2):
                    add_segment(frame, points[i], points[i + 1])
        finish()
        self.dirty = False
        return lines


def rotate(quaternion: Tuple[float, float, float, float], v: Point) -> Point:
    """Dreht v mit dem Quaternion (x, y, z, w)."""
    qx, qy, qz, qw = quaternion
    # t = 2 * (q_vec x v), v' = v + w * t + q_vec x t
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (v[0] + qw * tx + (qy * tz - qz * ty),
            v[1] + qw * ty + (qz * tx - qx * tz),
            v[2] + qw * tz + (qx * ty - qy * tx))


def transform_points(points: Sequence[Point], translation: Point,
                     quaternion: Tuple[float, float, float, float]) -> List[Point]:
    """Punkte aus einem Frame in einen anderen: erst drehen, dann verschieben (wie tf2)."""
    out = []
    for p in points:
        r = rotate(quaternion, p)
        out.append((r[0] + translation[0], r[1] + translation[1], r[2] + translation[2]))
    return out


def limit_points(lines: List[List[Point]], max_points: int) -> List[List[Point]]:
    """Höchstens etwa max_points Punkte insgesamt: gleichmäßig ausdünnen, Anfang und Ende jeder Linie bleiben."""
    total = sum(len(line) for line in lines)
    if max_points <= 0 or total <= max_points:
        return lines
    step = math.ceil(total / max_points)
    thinned = []
    for line in lines:
        kept = line[::step]
        if kept[-1] != line[-1]:
            kept.append(line[-1])
        thinned.append(kept)
    return thinned
