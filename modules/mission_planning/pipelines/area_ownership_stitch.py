# -*- coding: utf-8 -*-
"""잔여 AREA 조각의 제한형 convex-hull 소유영역 생성기.

재계획 시 footprint 차감이 남긴 여러 조각은 촬영 의무로는 서로 분리되어
있지만, UAV 소유권을 다시 나눌 때는 하나의 논리 AREA로 취급해야 한다.

운영 규칙은 단순 convex hull이다. 다만 hull이 최초/안정 소유 AREA보다
커지는 것을 막기 위해 ``limit_geometry``가 있으면 반드시 그 경계로 자르고,
최종 결과가 입력 조각을 포함하면서 제한 경계를 넘지 않는지 다시 검증한다.
정확한 촬영 의무는 별도 workload geometry가 담당하므로 이 결과는 소유권
분할에만 사용한다.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from shapely.geometry import MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union


_MAX_FRAGMENTS = 40
_CONTAINMENT_TOLERANCE_M = 0.05
_AREA_TOLERANCE_M2 = 1.0


def _polygons(geometry: BaseGeometry | None) -> List[Polygon]:
    if geometry is None or geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, MultiPolygon):
        return [
            child
            for child in geometry.geoms
            if isinstance(child, Polygon) and not child.is_empty
        ]
    return [
        child
        for child in getattr(geometry, "geoms", [])
        if isinstance(child, Polygon) and not child.is_empty
    ]


def _valid(geometry: BaseGeometry) -> BaseGeometry:
    if geometry.is_valid:
        return geometry
    return geometry.buffer(0)


def _outer_shell(geometry: BaseGeometry | None) -> Optional[Polygon]:
    """Return one hole-free limiting shell, or None when it is ambiguous."""

    if geometry is None or geometry.is_empty:
        return None
    normalized = _valid(geometry)
    polygons = _polygons(normalized)
    if len(polygons) != 1:
        return None
    shell = Polygon(polygons[0].exterior)
    if shell.is_empty or float(shell.area or 0.0) <= 0.0:
        return None
    return shell


def _contains_source(candidate: Polygon, source: BaseGeometry) -> bool:
    try:
        missing = source.difference(candidate.buffer(_CONTAINMENT_TOLERANCE_M))
    except Exception:
        return False
    return bool(missing.is_empty or float(getattr(missing, "area", 0.0) or 0.0) <= _AREA_TOLERANCE_M2)


def convex_hull_area_fragments_xy(
    fragments: Sequence[Polygon],
    *,
    limit_geometry: BaseGeometry | None = None,
) -> Tuple[Optional[Polygon], Dict[str, Any]]:
    """Build one convex-hull ownership envelope with an optional hard boundary.

    ``limit_geometry`` is the original/stable AREA ownership polygon. When
    available, the returned polygon never extends outside that outer boundary.
    If clipping the hull disconnects it, the original single boundary itself is
    used as the safe ownership envelope; exact filming still stays clipped to
    the remaining workload.
    """

    info: Dict[str, Any] = {
        "method": "convex_hull",
        "partsBefore": 0,
        "partsAfter": 0,
        "sourceAreaM2": 0.0,
        "rawHullAreaM2": 0.0,
        "limitAreaM2": 0.0,
        "resultAreaM2": 0.0,
        "addedAreaM2": 0.0,
        "clippedToLimit": False,
        "usedLimitFallback": False,
    }
    try:
        parts = [
            _valid(poly)
            for poly in fragments
            if isinstance(poly, Polygon)
            and not poly.is_empty
            and float(poly.area or 0.0) > 0.0
        ]
        parts = _polygons(unary_union(parts)) if parts else []
        info["partsBefore"] = len(parts)
        if not parts:
            info["reason"] = "no_valid_fragments"
            return None, info
        if len(parts) > _MAX_FRAGMENTS:
            info["reason"] = "fragment_cap_exceeded"
            return None, info

        source = unary_union(parts)
        source_area_m2 = float(source.area or 0.0)
        info["sourceAreaM2"] = source_area_m2

        raw_hull = _valid(source.convex_hull)
        if not isinstance(raw_hull, Polygon) or raw_hull.is_empty:
            info["reason"] = "convex_hull_failed"
            return None, info
        info["rawHullAreaM2"] = float(raw_hull.area or 0.0)

        result = raw_hull
        limit_shell = _outer_shell(limit_geometry)
        if limit_shell is not None:
            info["limitAreaM2"] = float(limit_shell.area or 0.0)
            clipped = _valid(raw_hull.intersection(limit_shell))
            clipped_polygons = _polygons(clipped)
            if (
                len(clipped_polygons) == 1
                and _contains_source(clipped_polygons[0], source)
            ):
                result = clipped_polygons[0]
                info["clippedToLimit"] = not result.equals(raw_hull)
            elif _contains_source(limit_shell, source):
                # The hull may miss the narrow connector of a concave original
                # AREA and become disconnected after clipping. The original
                # boundary is still a safe single ownership envelope and is
                # the strict maximum area allowed by this contract.
                result = limit_shell
                info["clippedToLimit"] = True
                info["usedLimitFallback"] = True
            else:
                info["reason"] = "source_outside_limit"
                return None, info

            outside = result.difference(
                limit_shell.buffer(_CONTAINMENT_TOLERANCE_M)
            )
            if (
                not outside.is_empty
                and float(getattr(outside, "area", 0.0) or 0.0)
                > _AREA_TOLERANCE_M2
            ):
                info["reason"] = "limit_containment_failed"
                return None, info
            if float(result.area or 0.0) > float(limit_shell.area or 0.0) + _AREA_TOLERANCE_M2:
                info["reason"] = "limit_area_exceeded"
                return None, info

        if not _contains_source(result, source):
            info["reason"] = "source_containment_failed"
            return None, info

        info["partsAfter"] = 1
        info["resultAreaM2"] = float(result.area or 0.0)
        info["addedAreaM2"] = max(
            0.0,
            float(info["resultAreaM2"]) - source_area_m2,
        )
        return result, info
    except Exception as exc:  # pragma: no cover - shapely 내부 실패 방어
        info["reason"] = f"error:{exc}"
        return None, info


# 이전 내부 호출명은 유지하되 구현은 더 이상 closing이 아니다.
stitch_area_fragments_xy = convex_hull_area_fragments_xy


__all__ = [
    "convex_hull_area_fragments_xy",
    "stitch_area_fragments_xy",
]
