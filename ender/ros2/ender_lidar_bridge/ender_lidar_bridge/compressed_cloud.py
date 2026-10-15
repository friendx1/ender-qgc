"""Birks Paketformat für komprimierte Punktwolken (pointcloud_compressor_node) in Python.

Die Relay-Node braucht das nicht, sie reicht die Pakete unverändert weiter. Es dient den Tests
und dem Test-Server (tools/fake_map_server.py), die ein solches Paket erzeugen und prüfen.
Braucht das Paket "zstandard" (pip install zstandard), das auf der Drohne nicht nötig ist.

Entpackt:
    I stamp_sec, I stamp_nanosec, d anchor_x, d anchor_y, d anchor_z, I num_points, H frame_id_len
    frame_id (frame_id_len Bytes), dann num_points * (int16 x, y, z in Zentimetern relativ zum Anker)
"""

import struct

import numpy as np

HEADER = struct.Struct("<IIdddIH")  # 38 Bytes, ohne Füllbytes (Birk packt die Struktur mit pack(1))


def pack_cloud(points: np.ndarray, stamp_sec: int, stamp_nanosec: int, frame_id: str = "world",
               level: int = 3) -> bytes:
    import zstandard

    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    anchor = pts.mean(axis=0) if len(pts) else np.zeros(3)
    rel = np.clip(np.round((pts - anchor) * 100.0), -32767, 32767).astype("<i2")
    fid = frame_id.encode()
    raw = HEADER.pack(stamp_sec, stamp_nanosec, *anchor, len(pts), len(fid)) + fid + rel.tobytes()
    return zstandard.ZstdCompressor(level=level).compress(raw)


def unpack_cloud(blob: bytes):
    """Gibt (stamp_sec, stamp_nanosec, frame_id, Punkte in Metern als Array (n, 3)) zurück."""
    import zstandard

    raw = zstandard.ZstdDecompressor().decompress(blob)
    sec, nsec, ax, ay, az, count, id_len = HEADER.unpack_from(raw)
    offset = HEADER.size
    frame_id = raw[offset:offset + id_len].decode()
    offset += id_len
    rel = np.frombuffer(raw, dtype="<i2", count=count * 3, offset=offset).reshape(-1, 3)
    if offset + count * 6 != len(raw):
        raise ValueError("Länge passt nicht zur Punktzahl")
    return sec, nsec, frame_id, rel.astype(np.float64) / 100.0 + np.array([ax, ay, az])
