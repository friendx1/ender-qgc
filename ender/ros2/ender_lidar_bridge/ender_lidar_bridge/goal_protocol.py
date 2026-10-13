"""Übertragungsformat für Flugziele (EGOL v1) von der Bodenstation zum Onboard-PC.

Kein ROS-Import, damit Kodierung und Dekodierung ohne ROS getestet werden können.

Die Bodenstation schickt ein Ziel als Punkt im Koordinatensystem der Drohne (Meter,
x nach vorn, y nach links) und nennt dazu den Zeitstempel des Scans, in dem der Punkt
angeklickt wurde. Der Onboard-PC rechnet den Punkt mit der Pose zu diesem Zeitpunkt in das
Weltsystem des Planers um und antwortet mit einer Bestätigung (EGOA) an den Absender.
Alle Zahlen sind Little-Endian.

Ziel, Bodenstation -> Drohne (22 Bytes):
    4s  magic          "EGOL"
    B   version        1
    B   kind           1 = Ziel setzen
    I   goal_seq       fortlaufende Nummer des Ziels (vom Absender)
    I   scan_stamp_ms  stamp_ms des EPCL-Scans, in dem geklickt wurde
    f   x              Meter, nach vorn
    f   y              Meter, nach links

Bestätigung, Drohne -> Bodenstation (18 Bytes):
    4s  magic          "EGOA"
    B   version        1
    B   status         siehe STATUS_*
    I   goal_seq       Nummer des bestätigten Ziels
    f   world_x        Ziel im Weltsystem des Planers (Meter), bei Ablehnung 0
    f   world_y
"""

import math
import struct
from dataclasses import dataclass
from typing import Optional

GOAL_MAGIC = b"EGOL"
ACK_MAGIC = b"EGOA"
VERSION = 1
KIND_SET_GOAL = 1

STATUS_OK = 0
STATUS_STALE = 1      # Scan ist zu alt
STATUS_TOO_FAR = 2    # Ziel liegt außerhalb der erlaubten Entfernung
STATUS_NO_POSE = 3    # keine passende Pose der Drohne vorhanden
STATUS_BAD_VALUE = 4  # Zahl ist nicht endlich

GOAL = struct.Struct("<4sBBIIff")
ACK = struct.Struct("<4sBBIff")


@dataclass
class Goal:
    goal_seq: int
    scan_stamp_ms: int
    x: float
    y: float


@dataclass
class Ack:
    status: int
    goal_seq: int
    world_x: float
    world_y: float


def encode_goal(goal: Goal) -> bytes:
    return GOAL.pack(GOAL_MAGIC, VERSION, KIND_SET_GOAL, goal.goal_seq & 0xFFFFFFFF,
                     goal.scan_stamp_ms & 0xFFFFFFFF, goal.x, goal.y)


def decode_goal(datagram: bytes) -> Optional[Goal]:
    """Liefert None bei jedem Paket, das kein gültiges Ziel ist."""
    if len(datagram) != GOAL.size:
        return None
    magic, version, kind, seq, stamp, x, y = GOAL.unpack(datagram)
    if magic != GOAL_MAGIC or version != VERSION or kind != KIND_SET_GOAL:
        return None
    return Goal(seq, stamp, x, y)


def encode_ack(ack: Ack) -> bytes:
    return ACK.pack(ACK_MAGIC, VERSION, ack.status, ack.goal_seq & 0xFFFFFFFF,
                    ack.world_x, ack.world_y)


def decode_ack(datagram: bytes) -> Optional[Ack]:
    if len(datagram) != ACK.size:
        return None
    magic, version, status, seq, wx, wy = ACK.unpack(datagram)
    if magic != ACK_MAGIC or version != VERSION:
        return None
    return Ack(status, seq, wx, wy)


def is_finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)
