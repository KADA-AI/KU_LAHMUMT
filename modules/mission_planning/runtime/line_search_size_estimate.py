from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Any


HEAVY_SCAN_LINES = 3831
HEAVY_LINE_SEARCH_COORDS = 7662
WARN_LINE_SEARCH_COORDS = 1644
FLIGHTPATH_SLA_MS = 2000.0
HYBRID_SLA_MS = 3000.0

# User-observed current/reexecute scaling anchors.
OBSERVED_SMALL_COORDS = 1644
OBSERVED_SMALL_FLIGHTPATH_MS = 600.0
OBSERVED_HEAVY_COORDS = 7662
OBSERVED_HEAVY_FLIGHTPATH_MS = 93600.0


@dataclass(frozen=True)
class LineSearchSizeEstimate:
    scanLines: int = 0
    lineSearchCoords: int = 0
    lineSearchWaypoints: int = 0
    largestLineSearchCoords: int = 0
    estimatedJsonBytes: int = 0
    minFov: float | None = None
    maxFov: float | None = None
    minLineSweepSpacingM: float | None = None
    maxLineSweepSpacingM: float | None = None
    estimatedCriticalPathMs: float | None = None
    isWarn: bool = False
    isHeavy: bool = False
    reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _to_float(value: Any) -> float | None:
    try:
        if value is None or value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int(value: Any) -> int | None:
    try:
        if value is None or value == "":
            return None
        return int(float(value))
    except (TypeError, ValueError):
        return None


def _json_size(value: Any) -> int:
    return len(json.dumps(value, ensure_ascii=False, sort_keys=True, default=str).encode("utf-8"))


def _non_none_min(values: list[float]) -> float | None:
    return min(values) if values else None


def _non_none_max(values: list[float]) -> float | None:
    return max(values) if values else None


def _estimate_ms_from_coords(coords: int) -> float | None:
    if coords <= 0:
        return None
    if coords <= OBSERVED_SMALL_COORDS:
        return round(coords * (OBSERVED_SMALL_FLIGHTPATH_MS / OBSERVED_SMALL_COORDS), 3)
    exponent = math.log(OBSERVED_HEAVY_FLIGHTPATH_MS / OBSERVED_SMALL_FLIGHTPATH_MS) / math.log(
        OBSERVED_HEAVY_COORDS / OBSERVED_SMALL_COORDS
    )
    scale = OBSERVED_SMALL_FLIGHTPATH_MS / (OBSERVED_SMALL_COORDS**exponent)
    return round(scale * (coords**exponent), 3)


def _finish_estimate(
    *,
    scan_lines: int,
    coords: int,
    waypoints: int,
    largest_coords: int,
    json_bytes: int,
    fovs: list[float],
    spacings: list[float],
) -> LineSearchSizeEstimate:
    is_warn = coords >= WARN_LINE_SEARCH_COORDS or largest_coords >= WARN_LINE_SEARCH_COORDS
    is_heavy = coords >= HEAVY_LINE_SEARCH_COORDS or scan_lines >= HEAVY_SCAN_LINES or largest_coords >= HEAVY_LINE_SEARCH_COORDS
    reasons: list[str] = []
    if coords >= HEAVY_LINE_SEARCH_COORDS:
        reasons.append(f"lineSearchCoords>={HEAVY_LINE_SEARCH_COORDS}")
    elif coords >= WARN_LINE_SEARCH_COORDS:
        reasons.append(f"lineSearchCoords>={WARN_LINE_SEARCH_COORDS}")
    if scan_lines >= HEAVY_SCAN_LINES:
        reasons.append(f"scanLines>={HEAVY_SCAN_LINES}")
    if largest_coords >= HEAVY_LINE_SEARCH_COORDS:
        reasons.append(f"largestLineSearchCoords>={HEAVY_LINE_SEARCH_COORDS}")
    elif largest_coords >= WARN_LINE_SEARCH_COORDS:
        reasons.append(f"largestLineSearchCoords>={WARN_LINE_SEARCH_COORDS}")
    return LineSearchSizeEstimate(
        scanLines=int(scan_lines),
        lineSearchCoords=int(coords),
        lineSearchWaypoints=int(waypoints),
        largestLineSearchCoords=int(largest_coords),
        estimatedJsonBytes=int(json_bytes),
        minFov=_non_none_min(fovs),
        maxFov=_non_none_max(fovs),
        minLineSweepSpacingM=_non_none_min(spacings),
        maxLineSweepSpacingM=_non_none_max(spacings),
        estimatedCriticalPathMs=_estimate_ms_from_coords(int(coords)),
        isWarn=bool(is_warn),
        isHeavy=bool(is_heavy),
        reason=", ".join(reasons) if reasons else "below_line_heavy_threshold",
    )


def estimate_from_metrics(metrics: dict[str, Any]) -> LineSearchSizeEstimate:
    scan_lines = _to_int(metrics.get("scanLines")) or _to_int(metrics.get("scanLinePoints")) or 0
    coords = _to_int(metrics.get("lineSearchCoords")) or _to_int(metrics.get("areaMergedCoords")) or 0
    waypoints = _to_int(metrics.get("lineSearchWaypoints")) or _to_int(metrics.get("areaLineSearchWaypointsBuilt")) or 0
    spacing = _to_float(metrics.get("lineSweepSpacingM"))
    fov = _to_float(metrics.get("FOV") or metrics.get("fieldOfView"))
    return _finish_estimate(
        scan_lines=scan_lines,
        coords=coords,
        waypoints=waypoints,
        largest_coords=coords,
        json_bytes=_json_size(metrics),
        fovs=[fov] if fov is not None else [],
        spacings=[spacing] if spacing is not None else [],
    )


def estimate_from_flight_path(flight_path: dict[str, Any]) -> LineSearchSizeEstimate:
    waypoints = flight_path.get("waypointList") if isinstance(flight_path, dict) else None
    if not isinstance(waypoints, list):
        return _finish_estimate(
            scan_lines=0,
            coords=0,
            waypoints=0,
            largest_coords=0,
            json_bytes=0,
            fovs=[],
            spacings=[],
        )
    total_coords = 0
    line_waypoints = 0
    largest_coords = 0
    line_search_payloads: list[Any] = []
    fovs: list[float] = []
    spacings: list[float] = []
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty")
        if not isinstance(filming, dict):
            continue
        fov = _to_float(filming.get("fieldOfView") or filming.get("FOV"))
        if fov is not None:
            fovs.append(fov)
        spacing = _to_float(filming.get("lineSweepSpacingM"))
        if spacing is not None:
            spacings.append(spacing)
        line_search = filming.get("lineSearch")
        if not isinstance(line_search, dict):
            continue
        coords = line_search.get("coordinateList")
        if not isinstance(coords, list):
            continue
        count = len(coords)
        line_waypoints += 1
        total_coords += count
        largest_coords = max(largest_coords, count)
        line_search_payloads.append(line_search)
    return _finish_estimate(
        scan_lines=0,
        coords=total_coords,
        waypoints=line_waypoints,
        largest_coords=largest_coords,
        json_bytes=_json_size(line_search_payloads),
        fovs=fovs,
        spacings=spacings,
    )


def should_use_line_heavy_engine(estimate: LineSearchSizeEstimate | dict[str, Any]) -> bool:
    if isinstance(estimate, LineSearchSizeEstimate):
        return bool(estimate.isHeavy)
    return estimate_from_metrics(estimate).isHeavy
