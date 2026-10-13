"""Ausdünnen einer Punktwolke: ein Punkt je Voxel, danach Obergrenze für die Punktzahl."""

from typing import Optional, Tuple

import numpy as np


def reduce_points(
    points: np.ndarray,
    voxel_size: float,
    max_points: int,
    max_range: float,
    intensity: Optional[np.ndarray] = None,
    rng: Optional[np.random.Generator] = None,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """points: N x 3 in Metern, Ursprung = Drohne. Gibt (Punkte, Intensität) zurück."""
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    inten = None if intensity is None else np.asarray(intensity).reshape(-1)
    ok = np.all(np.isfinite(pts), axis=1)
    if max_range > 0:
        ok &= np.linalg.norm(pts, axis=1) <= max_range
    pts = pts[ok]
    if inten is not None:
        inten = inten[ok]
    if len(pts) == 0:
        return pts, inten
    if voxel_size > 0:
        keys = np.floor(pts / voxel_size).astype(np.int64)
        _, first = np.unique(keys, axis=0, return_index=True)
        first.sort()
        pts = pts[first]
        if inten is not None:
            inten = inten[first]
    if max_points > 0 and len(pts) > max_points:
        rng = rng or np.random.default_rng()
        pick = np.sort(rng.choice(len(pts), size=max_points, replace=False))
        pts = pts[pick]
        if inten is not None:
            inten = inten[pick]
    return pts, inten
