from __future__ import annotations

import math
from typing import Any, Iterable, Sequence


def _coerce_float(value: object | None) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _self_intersects(points_xy: Sequence[tuple[float, float]]) -> bool:
    if len(points_xy) < 4:
        return False

    def orient(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
        return ((b[0] - a[0]) * (c[1] - a[1])) - ((b[1] - a[1]) * (c[0] - a[0]))

    def segments_intersect(
        a1: tuple[float, float],
        a2: tuple[float, float],
        b1: tuple[float, float],
        b2: tuple[float, float],
    ) -> bool:
        o1 = orient(a1, a2, b1)
        o2 = orient(a1, a2, b2)
        o3 = orient(b1, b2, a1)
        o4 = orient(b1, b2, a2)
        return (o1 * o2) < 0.0 and (o3 * o4) < 0.0

    return segments_intersect(points_xy[0], points_xy[1], points_xy[2], points_xy[3]) or segments_intersect(
        points_xy[1], points_xy[2], points_xy[3], points_xy[0]
    )


def _rotate_start(rows: list[tuple[Any, float, float]], start_idx: int) -> list[tuple[Any, float, float]]:
    return rows[start_idx:] + rows[:start_idx]


def _clockwise_rows(rows: list[tuple[Any, float, float]]) -> list[tuple[Any, float, float]]:
    center_x = sum(float(row[1]) for row in rows) / float(len(rows))
    center_y = sum(float(row[2]) for row in rows) / float(len(rows))
    ordered = sorted(
        rows,
        key=lambda row: math.atan2(float(row[2]) - center_y, float(row[1]) - center_x),
        reverse=True,
    )
    start_idx = min(
        range(len(ordered)),
        key=lambda idx: (-float(ordered[idx][2]), float(ordered[idx][1])),
    )
    ordered = _rotate_start(ordered, start_idx)
    if len(ordered) >= 4 and float(ordered[1][1]) < float(ordered[-1][1]):
        ordered = [ordered[0]] + list(reversed(ordered[1:]))
    return ordered


def _canonicalize_quad_rows(rows: list[tuple[Any, float, float]]) -> list[tuple[Any, float, float]]:
    if len(rows) != 4:
        return list(rows)

    by_screen = sorted(rows, key=lambda row: (-float(row[2]), float(row[1])))
    top = sorted(by_screen[:2], key=lambda row: float(row[1]))
    bottom = sorted(by_screen[2:], key=lambda row: float(row[1]), reverse=True)
    ordered = top + bottom
    ordered_xy = [(float(row[1]), float(row[2])) for row in ordered]
    if not _self_intersects(ordered_xy):
        return ordered

    ordered = _clockwise_rows(rows)
    ordered_xy = [(float(row[1]), float(row[2])) for row in ordered]
    if not _self_intersects(ordered_xy):
        return ordered

    swapped = [ordered[0], ordered[1], ordered[3], ordered[2]]
    swapped_xy = [(float(row[1]), float(row[2])) for row in swapped]
    if not _self_intersects(swapped_xy):
        return swapped
    return ordered


def normalize_footprint_corner_dicts(corners: Iterable[dict[str, Any]] | None) -> list[dict[str, Any]]:
    rows: list[tuple[dict[str, Any], float, float]] = []
    for corner in corners or []:
        if not isinstance(corner, dict):
            continue
        lat = _coerce_float(corner.get("latitude") or corner.get("Latitude"))
        lon = _coerce_float(corner.get("longitude") or corner.get("Longitude"))
        if lat is None or lon is None:
            continue
        rows.append((dict(corner), float(lon), float(lat)))
        if len(rows) >= 4:
            break
    if len(rows) < 4:
        return [row[0] for row in rows]
    return [row[0] for row in _canonicalize_quad_rows(rows)]


def normalize_footprint_ring(
    ring: Iterable[Sequence[float]] | None,
    *,
    closed: bool = True,
) -> list[list[float]]:
    points: list[list[float]] = []
    for coord in ring or []:
        if not isinstance(coord, (list, tuple)) or len(coord) < 2:
            continue
        lon = _coerce_float(coord[0])
        lat = _coerce_float(coord[1])
        if lon is None or lat is None:
            continue
        points.append([float(lon), float(lat)])
    if len(points) >= 2 and points[0] == points[-1]:
        points = points[:-1]
    rows = [(list(point), float(point[0]), float(point[1])) for point in points[:4]]
    if len(rows) < 4:
        ordered = [row[0] for row in rows]
    else:
        ordered = [row[0] for row in _canonicalize_quad_rows(rows)]
    if closed and ordered:
        return ordered + [list(ordered[0])]
    return ordered


__all__ = [
    "normalize_footprint_corner_dicts",
    "normalize_footprint_ring",
]
