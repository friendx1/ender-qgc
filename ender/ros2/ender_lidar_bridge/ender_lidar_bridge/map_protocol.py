"""Übertragungsformat der Hindernis-Karte (EMAP v1) zwischen Onboard-PC und Bodenstation.

Kein ROS-Import, damit alles ohne ROS getestet werden kann.

Die Karte ist groß und ändert sich langsam, deshalb läuft sie über TCP (verlorene Teile werden
wiederholt) und nicht über UDP wie der Scan. Der Onboard-PC ist der Server, die Bodenstation
verbindet sich. Über dieselbe Verbindung kommt ein kleiner Strom aus Posen, damit die Bodenstation
die Karte (Weltsystem) zur Drohne ausrichten kann, ohne die Karte neu zu senden.

Jede Nachricht beginnt mit einem Kopf (12 Bytes, Little-Endian):
    4s  magic        "EMAP"
    B   version      1
    B   type         1 = Karte, 2 = Pose, 3 = Steuerung
    H   reserved     0
    I   payload_len  Länge der Nutzdaten in Bytes

Typ 1, Karte: die Nutzdaten sind unverändert das zstd-Paket, das Birks
    pointcloud_compressor_node auf <topic>_compressed veröffentlicht. Entpackt:
        I stamp_sec, I stamp_nanosec, d anchor_x, d anchor_y, d anchor_z, I num_points,
        H frame_id_len, dann frame_id, dann num_points * (int16 x, y, z in Zentimetern relativ
        zum Anker). Die Punkte liegen im Weltsystem (camera_init).

Typ 2, Pose der Drohne im Weltsystem (44 Bytes):
    I stamp_ms, d x, d y, d z, f qx, f qy, f qz, f qw

Typ 3, Steuerung (Bodenstation -> Drohne, 1 Byte): Flags. Bit 0 = die Karte wird gebraucht.
    Die Pose läuft immer, sobald die Bodenstation verbunden ist (die freie Kamera braucht sie).
    Die Karte kostet Rechenzeit und Bandbreite und kommt deshalb erst, wenn die Bodenstation sie
    verlangt. Bis zur ersten Steuernachricht gilt: keine Karte.
"""

import struct
from collections import namedtuple
from typing import List, Optional, Tuple

MAGIC = b"EMAP"
VERSION = 1
TYPE_MAP = 1
TYPE_POSE = 2
TYPE_CONTROL = 3
FLAG_WANT_MAP = 0x01
HEADER = struct.Struct("<4sBBHI")
POSE = struct.Struct("<Iddd4f")
MAX_PAYLOAD = 4 * 1024 * 1024  # größer als jede sinnvolle Karte; schützt vor kaputten Längenfeldern

Pose = namedtuple("Pose", "stamp_ms position orientation")


def encode_message(msg_type: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"Nutzdaten zu groß: {len(payload)} Bytes")
    return HEADER.pack(MAGIC, VERSION, msg_type, 0, len(payload)) + payload


def encode_map(compressed_blob: bytes) -> bytes:
    return encode_message(TYPE_MAP, compressed_blob)


def encode_pose(stamp_ms: int, position: Tuple[float, float, float],
                orientation: Tuple[float, float, float, float]) -> bytes:
    """orientation: Quaternion (x, y, z, w)."""
    payload = POSE.pack(stamp_ms & 0xFFFFFFFF, *position, *orientation)
    return encode_message(TYPE_POSE, payload)


def encode_control(want_map: bool) -> bytes:
    return encode_message(TYPE_CONTROL, bytes([FLAG_WANT_MAP if want_map else 0]))


def decode_control(payload: bytes) -> Optional[bool]:
    """Gibt zurück, ob die Karte gewünscht wird (None bei kaputten Nutzdaten)."""
    if len(payload) != 1:
        return None
    return bool(payload[0] & FLAG_WANT_MAP)


def decode_pose(payload: bytes) -> Optional[Pose]:
    if len(payload) != POSE.size:
        return None
    stamp_ms, x, y, z, qx, qy, qz, qw = POSE.unpack(payload)
    return Pose(stamp_ms, (x, y, z), (qx, qy, qz, qw))


class StreamDecoder:
    """Setzt aus dem TCP-Strom wieder einzelne Nachrichten zusammen.

    Findet den Kopf nicht, werden Bytes bis zum nächsten "EMAP" verworfen, so fängt sich der
    Empfänger nach einem Fehler wieder.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_bytes = 0

    def feed(self, data: bytes) -> List[Tuple[int, bytes]]:
        self._buffer += data
        messages = []
        while True:
            if len(self._buffer) < HEADER.size:
                break
            magic, version, msg_type, _reserved, length = HEADER.unpack_from(self._buffer)
            if magic != MAGIC or version != VERSION or length > MAX_PAYLOAD:
                self._skip_to_next_magic()
                continue
            end = HEADER.size + length
            if len(self._buffer) < end:
                break
            messages.append((msg_type, bytes(self._buffer[HEADER.size:end])))
            del self._buffer[:end]
        return messages

    def _skip_to_next_magic(self) -> None:
        index = self._buffer.find(MAGIC, 1)
        drop = index if index > 0 else max(1, len(self._buffer) - (len(MAGIC) - 1))
        self.bad_bytes += drop
        del self._buffer[:drop]


class OutQueue:
    """Was als Nächstes an einen Empfänger geht, wenn die Verbindung langsam ist.

    Es zählt nur das Neueste: ein noch nicht begonnener Stand wird von einem neueren ersetzt.
    Eine Nachricht, die schon zur Hälfte gesendet ist, wird zu Ende gesendet (TCP kennt kein
    halbes Paket). Eine Pose geht vor einer Karte, weil sie klein ist und die Ausrichtung braucht.
    """

    def __init__(self) -> None:
        self._current = b""
        self._offset = 0
        self._map: Optional[bytes] = None
        self._pose: Optional[bytes] = None
        self.dropped_maps = 0

    def put_map(self, message: bytes) -> None:
        if self._map is not None:
            self.dropped_maps += 1
        self._map = message

    def put_pose(self, message: bytes) -> None:
        self._pose = message

    def peek(self) -> memoryview:
        """Die Bytes, die als Nächstes gesendet werden sollen (leer, wenn nichts ansteht)."""
        if self._offset >= len(self._current):
            self._current, self._offset = b"", 0
            if self._pose is not None:
                self._current, self._pose = self._pose, None
            elif self._map is not None:
                self._current, self._map = self._map, None
        return memoryview(self._current)[self._offset:]

    def consume(self, sent: int) -> None:
        self._offset += sent
