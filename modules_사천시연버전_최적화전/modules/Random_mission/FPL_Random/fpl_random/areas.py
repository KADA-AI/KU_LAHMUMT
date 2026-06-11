"""
Static presets for the auto-mission generation area and start reference points.

Kept in a dedicated package so other generators can import without hardcoding.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Tuple

from . import config


@dataclass(frozen=True)
class LatLon:
    latitude: float
    longitude: float

    def as_tuple(self) -> Tuple[float, float]:
        return self.latitude, self.longitude

    def to_dict(self) -> dict:
        return {"latitude": self.latitude, "longitude": self.longitude}


@dataclass(frozen=True)
class ScenarioArea:
    """Axis-aligned square/rectangle defined by southwest and northeast corners."""

    southwest: LatLon
    northeast: LatLon

    @property
    def corners(self) -> Tuple[LatLon, LatLon, LatLon, LatLon]:
        sw = self.southwest
        ne = self.northeast
        nw = LatLon(latitude=ne.latitude, longitude=sw.longitude)
        se = LatLon(latitude=sw.latitude, longitude=ne.longitude)
        return (sw, nw, ne, se)

    def to_area_lat_lon_list(self) -> List[dict]:
        """Return list of dicts suitable for areaLatLonList serialization."""
        return [pt.to_dict() for pt in self.corners]


# 자동 임무 생성 구역(3배/5배 좌표는 config에 정의).
AUTO_MISSION_AREA = ScenarioArea(
    southwest=LatLon(*config.AUTO_MISSION_AREA_SW),
    northeast=LatLon(*config.AUTO_MISSION_AREA_NE),
)

# 구역 내 시작 참조점(가장자리 순서: 서쪽, 동쪽, 남쪽, 북쪽).
START_REFERENCE_POINTS: Tuple[LatLon, ...] = tuple(
    LatLon(lat, lon) for lat, lon in config.START_REFERENCE_POINTS_RAW
)


def all_start_points_dicts() -> List[dict]:
    return [p.to_dict() for p in START_REFERENCE_POINTS]


def iter_start_points(shuffle: bool = False, rng=None) -> Iterable[LatLon]:
    """
    Yield start reference points. Optionally shuffle with a provided RNG.

    Args:
        shuffle: If True, iterate in random order.
        rng: Optional random.Random instance for deterministic shuffling.
    """
    if not shuffle:
        return iter(START_REFERENCE_POINTS)
    import random

    seq: List[LatLon] = list(START_REFERENCE_POINTS)
    (rng or random).shuffle(seq)
    return iter(seq)
