"""Übertragungsformat der Punktwolke (EPCL v1) zwischen Onboard-PC und Bodenstation.

Kein ROS-Import, damit Kodierung und Dekodierung ohne ROS getestet werden können.

Jeder Scan wird in kleine UDP-Pakete zerlegt (je höchstens MAX_PAYLOAD Bytes, damit nichts
fragmentiert wird). Alle Zahlen sind Little-Endian.

Kopf (24 Bytes):
    4s  magic         "EPCL"
    B   version       1
    B   flags         Bit 0: je Punkt folgt zusätzlich die Intensität (uint8)
    I   frame_seq     fortlaufende Nummer des Scans
    I   stamp_ms      Zeitstempel des Scans in Millisekunden (läuft über, nur zur Anzeige)
    I   total_points  Anzahl Punkte im gesamten Scan
    H   chunk_index   Nummer dieses Pakets (ab 0)
    H   chunk_count   Anzahl Pakete des Scans
    H   points_here   Anzahl Punkte in diesem Paket

Punkt: x, y, z als int16 in Zentimetern (Bereich +-327 m), optional danach Intensität als uint8.
"""

import struct
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

MAGIC = b"EPCL"
VERSION = 1
FLAG_INTENSITY = 0x01
HEADER = struct.Struct("<4sBBIIIHHH")
MAX_PAYLOAD = 1200
CM_PER_METER = 100.0
INT16_MAX = 32767


def bytes_per_point(with_intensity: bool) -> int:
    return 7 if with_intensity else 6


def encode_frame(
    points_m: np.ndarray,
    frame_seq: int,
    stamp_ms: int,
    intensity: Optional[np.ndarray] = None,
    max_payload: int = MAX_PAYLOAD,
) -> List[bytes]:
    """Zerlegt einen Scan (N x 3, Meter) in Datagramme. Punkte außerhalb von +-327 m werden verworfen."""
    pts = np.asarray(points_m, dtype=np.float64).reshape(-1, 3)
    cm = np.rint(pts * CM_PER_METER)
    keep = np.all(np.abs(cm) <= INT16_MAX, axis=1)
    cm = cm[keep].astype("<i2")
    with_i = intensity is not None
    if with_i:
        inten = np.clip(np.asarray(intensity).reshape(-1)[keep], 0, 255).astype("u1")
    flags = FLAG_INTENSITY if with_i else 0
    per_point = bytes_per_point(with_i)
    per_chunk = max(1, (max_payload - HEADER.size) // per_point)
    total = len(cm)
    chunk_count = max(1, -(-total // per_chunk))  # Aufrunden, auch ein leerer Scan ergibt ein Paket
    out: List[bytes] = []
    for index in range(chunk_count):
        lo, hi = index * per_chunk, min(total, (index + 1) * per_chunk)
        if with_i:
            body = np.empty(hi - lo, dtype=[("p", "<i2", 3), ("i", "u1")])
            body["p"], body["i"] = cm[lo:hi], inten[lo:hi]
            data = body.tobytes()
        else:
            data = cm[lo:hi].tobytes()
        head = HEADER.pack(MAGIC, VERSION, flags, frame_seq & 0xFFFFFFFF, stamp_ms & 0xFFFFFFFF,
                           total, index, chunk_count, hi - lo)
        out.append(head + data)
    return out


@dataclass
class Frame:
    frame_seq: int
    stamp_ms: int
    points_m: np.ndarray  # N x 3, Meter
    intensity: Optional[np.ndarray] = None  # N, uint8


@dataclass
class _Partial:
    stamp_ms: int
    total: int
    chunk_count: int
    flags: int
    chunks: Dict[int, bytes] = field(default_factory=dict)


class FrameAssembler:
    """Setzt Datagramme zu Scans zusammen. Unvollständige Scans werden verworfen,
    sobald ein neuerer Scan beginnt, denn eine halbe Punktwolke sähe falsch aus."""

    def __init__(self) -> None:
        self._partial: Dict[int, _Partial] = {}
        self.completed = 0
        self.dropped = 0
        self.bad_packets = 0

    def feed(self, datagram: bytes) -> Optional[Frame]:
        if len(datagram) < HEADER.size:
            self.bad_packets += 1
            return None
        magic, version, flags, seq, stamp, total, index, count, here = HEADER.unpack_from(datagram)
        per_point = bytes_per_point(bool(flags & FLAG_INTENSITY))
        if (magic != MAGIC or version != VERSION or count == 0 or index >= count
                or len(datagram) != HEADER.size + here * per_point):
            self.bad_packets += 1
            return None
        # ältere unvollständige Scans fallen weg
        for old in [s for s in self._partial if _older(s, seq)]:
            del self._partial[old]
            self.dropped += 1
        part = self._partial.setdefault(seq, _Partial(stamp, total, count, flags))
        part.chunks[index] = datagram[HEADER.size:]
        if len(part.chunks) < part.chunk_count:
            return None
        del self._partial[seq]
        self.completed += 1
        raw = b"".join(part.chunks[i] for i in range(part.chunk_count))
        if flags & FLAG_INTENSITY:
            arr = np.frombuffer(raw, dtype=[("p", "<i2", 3), ("i", "u1")])
            return Frame(seq, stamp, arr["p"].astype(np.float32) / CM_PER_METER, arr["i"].copy())
        arr = np.frombuffer(raw, dtype="<i2").reshape(-1, 3)
        return Frame(seq, stamp, arr.astype(np.float32) / CM_PER_METER)


def _older(a: int, b: int) -> bool:
    """True, wenn Scan-Nummer a vor b liegt (mit Überlauf bei 32 Bit)."""
    return a != b and ((b - a) & 0xFFFFFFFF) < 0x80000000
