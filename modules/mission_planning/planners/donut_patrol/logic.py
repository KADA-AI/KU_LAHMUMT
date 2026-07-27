"""Donut (annulus) patrol planner for Type 4 boundary missions (production port).

Ported from test/donut_patrol_sim (prototype validated 2026-07-09); the test
folder is NOT a runtime dependency. Pure logic only - no GUI.

The donut is treated as a CLOSED CORRIDOR around the hole, mirroring the
production 0303 line-mission split between the flight route and the camera:

- aircraft route  = closed ring loop offset from the hole (offset >= turn
  radius, round joins, so every corner arc is flyable)
- camera work     = radial ``lineSearch`` sweeps across the annulus emitted as
  0303-style ``filmingProperty`` (operationMode=2) rows

Capture law (shared with production planning):

- FOV comes from the area FOV DB (selection only) or the runtime manual FOV
- sweep spacing = nadir vertical footprint x (1 - vertical overlap) at the
  aircraft altitude layer (capture_geometry unified law)
- sweep stations are spaced by ARC LENGTH ON THE BAND'S OUTER EDGE, i.e. at
  the widest end of the radial fan, so the fan-out can no longer open gaps
  near the outer boundary
- aircraft speed comes from capture_speed_plan and is then clamped so every
  sweep finishes inside its transit window at <= camera_speed_cap_mps
  (ICD lineSearch.searchSpeed limit)

Multi-aircraft partition strategies:

- ``band``     (default): the radial gap is split into N annular bands, one
  per aircraft. Each aircraft circles its own lane and only sweeps its band,
  so nobody duplicates anybody else's strip.
- ``windmill``: all aircraft sweep the full gap but the stations are assigned
  round-robin and the aircraft fly phase-offset like windmill blades; the
  donut is fully covered once per cooperative lap.

Assumes the donut is star-shaped around the hole centroid (convex-ish outer
boundary and hole), which matches the Type 4 boundary-region inputs. Coverage
is measured explicitly (sweep strips vs donut area) so a violation of that
assumption shows up as a coverage drop instead of a silent gap.
"""

from __future__ import annotations

import json
import math
import sys
from bisect import bisect_right as _bisect_right
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from shapely.geometry import LineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.geometry.polygon import orient
from shapely.ops import unary_union


EARTH_M_PER_DEG_LAT = 111_132.92

try:
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
        d0303 as _d0303,
    )
except Exception:
    _d0303 = None

try:
    from modules.mission_planning.MissionPlanner import capture_geometry as _capture
except Exception:
    _capture = None

# ICD lineSearch.searchSpeed is validated to 0..1000 m/s.
ICD_SEARCH_SPEED_MAX_MPS = 1000.0


@dataclass(frozen=True)
class LocalFrame:
    lat0: float
    lon0: float
    cos_lat0: float

    @classmethod
    def from_latlon_rows(cls, rows: Iterable[dict[str, Any]]) -> "LocalFrame":
        coords = [
            (float(row["latitude"]), float(row["longitude"]))
            for row in rows
            if isinstance(row, dict)
            and row.get("latitude") is not None
            and row.get("longitude") is not None
        ]
        if not coords:
            raise ValueError("no coordinates available for local frame")
        lat0 = sum(lat for lat, _lon in coords) / float(len(coords))
        lon0 = sum(lon for _lat, lon in coords) / float(len(coords))
        return cls(lat0=lat0, lon0=lon0, cos_lat0=max(1e-6, math.cos(math.radians(lat0))))

    def to_xy(self, coord: dict[str, Any]) -> tuple[float, float]:
        lat = float(coord["latitude"])
        lon = float(coord["longitude"])
        x = (lon - self.lon0) * EARTH_M_PER_DEG_LAT * self.cos_lat0
        y = (lat - self.lat0) * EARTH_M_PER_DEG_LAT
        return (float(x), float(y))

    def to_latlon(self, x: float, y: float, *, altitude: int = 0) -> dict[str, Any]:
        lat = self.lat0 + float(y) / EARTH_M_PER_DEG_LAT
        lon = self.lon0 + float(x) / (EARTH_M_PER_DEG_LAT * self.cos_lat0)
        return {
            "latitude": round(float(lat), 8),
            "longitude": round(float(lon), 8),
            "altitude": int(altitude),
        }


@dataclass(frozen=True)
class PatrolConfig:
    strategy: str = "band"  # "band" | "windmill"
    aircraft_count: int = 3
    aircraft_ids: tuple[int, ...] = (4, 5, 6)
    laps: int = 3
    speed_mps: float = 0.0  # 0 = capture-law aircraft speed
    altitude_m: int = 0  # 0 = runtime altitude layers (1000/1010/1020)
    turn_radius_m: float = 0.0  # 0 = runtime dubins_turn_radius_m
    turn_step_deg: float = 0.0  # 0 = runtime turn_step_deg
    fov_deg: float = 0.0  # 0 = area FOV DB selection / runtime manual FOV
    separation_m: float = 0.0  # legacy fallback for the no-capture-law path
    sweep_spacing_m: float = 0.0  # 0 = capture-law footprint spacing
    sweep_bearing_deg: float = 180.0  # reference bearing for the DB span
    use_area_db: bool = True
    sensor_type: int = 1
    search_speed_weight: float = 0.0  # 0 = runtime AREA_SEARCH_SPEED_WEIGHT
    directions: tuple[str, ...] = ()  # () = auto (band: alternate, windmill: same)
    phase_fractions: tuple[float, ...] = ()  # () = auto (k/N)
    lane_offsets_m: tuple[float, ...] = ()  # () = auto placement inside feasible span
    edge_margin_m: float = 30.0  # lane clearance from the outer boundary
    band_overlap_fraction: float = 0.01  # inter-band seam overlap (fraction of gap)
    camera_speed_cap_mps: float = ICD_SEARCH_SPEED_MAX_MPS
    # Route waypoints are kept only at features: ONE at each corner entry and
    # ONE at each corner exit (a shallow corner collapses to a single apex
    # WP), plus straight-leg fillers at route_wp_spacing_m. Waypoints closer
    # than min_wp_separation_m are merged because the real aircraft cannot
    # track tightly packed waypoints. The radial sweeps of each leg are packed
    # into the departure waypoint's lineSearch as one zigzag polyline,
    # matching the production 0303 grouping convention.
    route_wp_spacing_m: float = 0.0  # 0 = runtime uav_wp_interval_m (fallback 2000)
    min_wp_separation_m: float = 300.0  # aircraft cannot track WPs closer than this
    corner_turn_eps_deg: float = 3.0  # vertex turn angle that marks a corner arc
    max_linesearch_coords: int = 0  # 0 = runtime max_linesearch_coords_per_waypoint


@dataclass
class DonutMission:
    input_mission_id: int
    input_mission_type: int
    region_type: int
    frame: LocalFrame
    outer_latlon: list[dict[str, Any]]
    holes_latlon: list[list[dict[str, Any]]]
    polygon_xy: Polygon
    hole_polygons_xy: list[Polygon]


@dataclass(frozen=True)
class MissionPlanningProfile:
    """Mission-wide knobs shared by every aircraft (FOV/DB/turn geometry)."""

    source: str
    strategy: str
    bearing_deg: float
    raw_fov_deg: float
    fov_deg: float
    separation_m: float
    turn_radius_m: float
    turn_step_deg: float
    interpolation_points: int
    search_speed_weight: float
    camera_speed_cap_mps: float
    min_aircraft_speed_mps: float
    max_aircraft_speed_mps: float
    db_config: dict[str, float] | None = None
    db_span_m: float | None = None
    db_width_ref_m: float | None = None
    db_max_segment_m: float | None = None

    def as_dict(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "source": self.source,
            "strategy": self.strategy,
            "bearingDeg": round(float(self.bearing_deg), 3),
            "rawFovDeg": round(float(self.raw_fov_deg), 3),
            "fovDeg": round(float(self.fov_deg), 3),
            "separationM": round(float(self.separation_m), 3),
            "turnRadiusM": round(float(self.turn_radius_m), 3),
            "turnStepDeg": round(float(self.turn_step_deg), 3),
            "interpolationPoints": int(self.interpolation_points),
            "searchSpeedWeight": round(float(self.search_speed_weight), 3),
            "cameraSpeedCapMps": round(float(self.camera_speed_cap_mps), 3),
            "minAircraftSpeedMps": round(float(self.min_aircraft_speed_mps), 3),
            "maxAircraftSpeedMps": round(float(self.max_aircraft_speed_mps), 3),
        }
        if self.db_config is not None:
            out["dbConfig"] = {key: round(float(value), 6) for key, value in self.db_config.items()}
            out["dbSpanM"] = round(float(self.db_span_m or 0.0), 3)
            out["dbWidthRefM"] = round(float(self.db_width_ref_m or 0.0), 3)
            out["dbMaxSegmentM"] = round(float(self.db_max_segment_m or 0.0), 3)
        return out


@dataclass(frozen=True)
class CaptureSpec:
    """Per-aircraft capture-law resolution at its altitude layer."""

    altitude_m: int
    footprint_horizontal_m: float
    footprint_vertical_m: float
    sweep_spacing_m: float
    coverage_width_m: float
    aircraft_speed_mps: float
    aircraft_speed_kmh: float
    scan_interval_mean_s: float
    max_sweep_length_m: float
    camera_speed_max_used_mps: float
    camera_limited: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "altitudeM": int(self.altitude_m),
            "footprintHorizontalM": round(float(self.footprint_horizontal_m), 3),
            "footprintVerticalM": round(float(self.footprint_vertical_m), 3),
            "sweepSpacingM": round(float(self.sweep_spacing_m), 3),
            "coverageWidthM": round(float(self.coverage_width_m), 3),
            "aircraftSpeedMps": round(float(self.aircraft_speed_mps), 3),
            "aircraftSpeedKmh": round(float(self.aircraft_speed_kmh), 3),
            "scanIntervalMeanS": round(float(self.scan_interval_mean_s), 3),
            "maxSweepLengthM": round(float(self.max_sweep_length_m), 3),
            "cameraSpeedMaxUsedMps": round(float(self.camera_speed_max_used_mps), 3),
            "cameraLimited": bool(self.camera_limited),
        }


PASS_FLYBY = 1
PASS_FLYOVER = 3


@dataclass
class SweepLine:
    """One radial sweep station (geometric record; coverage/debug only).

    Emission-wise the stations of a route leg are packed together into the
    departure waypoint's lineSearch (see LineSearchGroup), so a SweepLine no
    longer maps 1:1 onto a waypoint.
    """

    sweep_line_id: int
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    start_coordinate: dict[str, Any]
    end_coordinate: dict[str, Any]
    sweep_length_m: float
    strip: BaseGeometry | None = None  # coverage strip (not serialized)
    reveal_eta_s: float = 0.0  # lap-1 time the camera finishes this station

    def as_dict(self) -> dict[str, Any]:
        return {
            "sweepLineID": int(self.sweep_line_id),
            "sweepLengthM": round(float(self.sweep_length_m), 3),
            "revealEtaS": round(float(self.reveal_eta_s), 3),
            "startCoordinate": dict(self.start_coordinate),
            "endCoordinate": dict(self.end_coordinate),
            "xy": [
                [round(float(self.start_x), 3), round(float(self.start_y), 3)],
                [round(float(self.end_x), 3), round(float(self.end_y), 3)],
            ],
        }


@dataclass
class LineSearchGroup:
    """Zigzag camera polyline covering all sweep stations of one route leg."""

    group_id: int
    coords_xy: list[tuple[float, float]]
    coordinate_list: list[dict[str, Any]]
    cumulative_m: list[float]  # polyline arc length at each coordinate
    search_speed_mps: float
    length_m: float
    sweep_line_ids: list[int]
    interpolation_points: int

    def as_dict(self) -> dict[str, Any]:
        return {
            "lineSearchGroupID": int(self.group_id),
            "coordinateCount": len(self.coordinate_list),
            "lengthM": round(float(self.length_m), 3),
            "searchSpeed": round(float(self.search_speed_mps), 3),
            "sweepLineIDs": [int(v) for v in self.sweep_line_ids],
        }


@dataclass
class Waypoint:
    waypoint_id: int
    lap: int
    x: float
    y: float
    latitude: float
    longitude: float
    altitude: int
    speed_mps: float
    heading_deg: float
    eta_s: float
    cumulative_m: float
    next_waypoint_id: int = 0
    waypoint_pass_type: int = PASS_FLYBY
    fov_deg: float = 2.4
    sensor_type: int = 1
    line_search: LineSearchGroup | None = None

    def as_dict(self) -> dict[str, Any]:
        row = {
            "waypointID": int(self.waypoint_id),
            "coordinate": {
                "latitude": round(float(self.latitude), 8),
                "longitude": round(float(self.longitude), 8),
                "altitude": int(self.altitude),
            },
            "speed": round(float(self.speed_mps), 3),
            "eta": round(float(self.eta_s), 3),
            "ecf": 0.0,
            "nextWaypointID": int(self.next_waypoint_id),
            "waypointPassType": int(self.waypoint_pass_type),
            "_debug": {
                "lap": int(self.lap),
                "xy": [round(float(self.x), 3), round(float(self.y), 3)],
                "headingDeg": round(float(self.heading_deg), 2),
            },
            "cumulativeM": round(float(self.cumulative_m), 3),
        }
        if self.line_search is not None:
            row["filmingProperty"] = {
                "fieldOfView": round(float(self.fov_deg), 3),
                "sensorType": int(self.sensor_type),
                "operationMode": 2,
                "lineSearch": {
                    "coordinateList": [dict(coord) for coord in self.line_search.coordinate_list],
                    "searchSpeed": round(float(self.line_search.search_speed_mps), 3),
                    "interpolationPoints": int(self.line_search.interpolation_points),
                },
            }
            row["_debug"]["lineSearchGroup"] = self.line_search.as_dict()
        return row


@dataclass
class AircraftPlan:
    aircraft_id: int
    direction: str
    strategy: str
    band_inner_fraction: float
    band_outer_fraction: float
    lane_offset_m: float
    turn_radius_m: float
    loop_length_m: float
    altitude_m: int = 0
    phase_fraction: float = 0.0
    min_hole_boundary_gap_m: float = 0.0
    turn_radius_violated: bool = False
    route_nodes_per_lap: int = 0
    route_wp_spacing_m: float = 0.0
    min_wp_separation_m: float = 0.0
    route_min_leg_m: float = 0.0
    route_max_turn_deg: float = 0.0
    max_linesearch_coords: int = 0
    capture_spec: CaptureSpec | None = None
    planning_profile: MissionPlanningProfile | None = None
    waypoints: list[Waypoint] = field(default_factory=list)
    sweep_lines: list[SweepLine] = field(default_factory=list)
    footprint_union: BaseGeometry | None = None
    coverage_ratio: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        film_wps = [wp for wp in self.waypoints if wp.line_search is not None]
        row = {
            "aircraftID": int(self.aircraft_id),
            "direction": self.direction,
            "strategy": self.strategy,
            "bandFraction": [
                round(float(self.band_inner_fraction), 4),
                round(float(self.band_outer_fraction), 4),
            ],
            "laneOffsetM": round(float(self.lane_offset_m), 3),
            "turnRadiusM": round(float(self.turn_radius_m), 3),
            "turnRadiusViolated": bool(self.turn_radius_violated),
            "loopLengthM": round(float(self.loop_length_m), 3),
            "altitudeM": int(self.altitude_m),
            "phaseFraction": round(float(self.phase_fraction), 3),
            "minHoleBoundaryGapM": round(float(self.min_hole_boundary_gap_m), 3),
            "coverageRatio": round(float(self.coverage_ratio), 5),
            "routeNodesPerLap": int(self.route_nodes_per_lap),
            "routeWpSpacingM": round(float(self.route_wp_spacing_m), 3),
            "minWpSeparationM": round(float(self.min_wp_separation_m), 3),
            "routeMinLegM": round(float(self.route_min_leg_m), 3),
            "routeMaxTurnDeg": round(float(self.route_max_turn_deg), 3),
            "maxLineSearchCoordsPerWp": int(self.max_linesearch_coords),
            "waypointCount": len(self.waypoints),
            "filmWaypointCount": len(film_wps),
            "maxLineSearchCoordsUsed": max(
                (len(wp.line_search.coordinate_list) for wp in film_wps),
                default=0,
            ),
            "sweepLineCount": len(self.sweep_lines),
            "sweepLineList": [row.as_dict() for row in self.sweep_lines],
            "waypointList": [wp.as_dict() for wp in self.waypoints],
            "footprintAreaM2": (
                round(float(self.footprint_union.area), 3)
                if self.footprint_union is not None and not self.footprint_union.is_empty
                else 0.0
            ),
        }
        if self.capture_spec is not None:
            row["captureSpec"] = self.capture_spec.as_dict()
        if self.planning_profile is not None:
            row["planningProfile"] = self.planning_profile.as_dict()
        return row


@dataclass(frozen=True)
class RadialRay:
    """One radial cut of the annulus: hole exit -> outer boundary hit."""

    theta_rad: float
    inner_xy: tuple[float, float]
    outer_xy: tuple[float, float]

    def point_at(self, fraction: float) -> tuple[float, float]:
        f = max(0.0, min(1.0, float(fraction)))
        return (
            self.inner_xy[0] + (self.outer_xy[0] - self.inner_xy[0]) * f,
            self.inner_xy[1] + (self.outer_xy[1] - self.inner_xy[1]) * f,
        )

    @property
    def gap_m(self) -> float:
        return _distance(self.inner_xy, self.outer_xy)


# ---------------------------------------------------------------------------
# 0201 payload -> DonutMission
# ---------------------------------------------------------------------------


def load_0201_payload(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def build_donut_mission_from_0201(
    payload: dict[str, Any],
    *,
    input_mission_id: int | None = None,
) -> DonutMission:
    missions = payload.get("inputMissionList")
    if not isinstance(missions, list):
        raise ValueError("inputMissionList is missing")

    candidate: dict[str, Any] | None = None
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        if input_mission_id is not None and int(mission.get("inputMissionID", 0) or 0) != int(input_mission_id):
            continue
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        areas = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
        has_outer = any(isinstance(area, dict) and not bool(area.get("isHole")) for area in areas)
        has_hole = any(isinstance(area, dict) and bool(area.get("isHole")) for area in areas)
        if has_outer and has_hole:
            candidate = mission
            break

    if candidate is None:
        raise ValueError("no area mission with both outer area and hole area was found")

    detail = candidate.get("missionDetail") if isinstance(candidate.get("missionDetail"), dict) else {}
    areas = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    outer_rows: list[dict[str, Any]] = []
    hole_rows: list[list[dict[str, Any]]] = []
    for area in areas:
        if not isinstance(area, dict):
            continue
        coords = _normalize_coord_rows(area.get("coordinateList"))
        if len(coords) < 3:
            continue
        if bool(area.get("isHole")):
            hole_rows.append(coords)
        elif not outer_rows:
            outer_rows = coords

    if len(outer_rows) < 3 or not hole_rows:
        raise ValueError("outer area or hole area coordinates are invalid")

    frame = LocalFrame.from_latlon_rows([*outer_rows, *[coord for hole in hole_rows for coord in hole]])
    outer_xy = [frame.to_xy(row) for row in outer_rows]
    holes_xy = [[frame.to_xy(row) for row in hole] for hole in hole_rows]
    polygon = Polygon(outer_xy, holes_xy)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    if polygon.is_empty:
        raise ValueError("donut polygon is empty")

    hole_polygons = [Polygon(hole) for hole in holes_xy]
    return DonutMission(
        input_mission_id=int(candidate.get("inputMissionID", 0) or 0),
        input_mission_type=int(candidate.get("inputMissionType", 0) or 0),
        region_type=int(candidate.get("regionType", 0) or 0),
        frame=frame,
        outer_latlon=outer_rows,
        holes_latlon=hole_rows,
        polygon_xy=polygon,
        hole_polygons_xy=hole_polygons,
    )


# ---------------------------------------------------------------------------
# Planning profile (FOV DB / runtime bridge)
# ---------------------------------------------------------------------------


def resolve_planning_profile(mission: DonutMission, config: PatrolConfig | None = None) -> MissionPlanningProfile:
    cfg = config or PatrolConfig()
    bearing_deg = float(cfg.sweep_bearing_deg)
    default_sep = _runtime_float("default_sweep_separation_m", _d0303_default("DEFAULT_SWEEP_SEPARATION_M", 1000.0))
    separation_m = float(cfg.separation_m) if float(cfg.separation_m or 0.0) > 0.0 else float(default_sep)
    raw_fov_deg = float(cfg.fov_deg) if float(cfg.fov_deg or 0.0) > 0.0 else _runtime_manual_fov(
        "area_custom_fov_deg",
        _d0303_default("AREA_CUSTOM_FOV_DEG", 2.4),
    )
    fov_deg = float(raw_fov_deg)
    db_config: dict[str, float] | None = None
    db_span_m: float | None = None
    db_width_ref_m: float | None = None
    db_max_segment_m: float | None = None
    source = "manual" if float(cfg.fov_deg or 0.0) > 0.0 else "runtime"

    if cfg.use_area_db and _d0303 is not None and float(cfg.fov_deg or 0.0) <= 0.0:
        try:
            outer_llh = [(float(row["latitude"]), float(row["longitude"])) for row in mission.outer_latlon]
            meta = _d0303._select_area_db_config(outer_llh, float(bearing_deg))
        except Exception:
            meta = None
        if isinstance(meta, dict) and isinstance(meta.get("config"), dict):
            raw_cfg = meta["config"]
            db_config = {
                key: float(raw_cfg.get(key, 0.0) or 0.0)
                for key in ("fov", "sep", "width", "vel", "dps", "foot")
                if raw_cfg.get(key) is not None
            }
            if db_config.get("sep", 0.0) > 0.0 and float(cfg.separation_m or 0.0) <= 0.0:
                separation_m = float(db_config["sep"])
            if db_config.get("fov", 0.0) > 0.0:
                raw_fov_deg = float(db_config["fov"])
                fov_deg = _apply_adjusted_db_fov(raw_fov_deg)
            db_span_m = _safe_float(meta.get("span_m"))
            db_width_ref_m = _safe_float(meta.get("width_ref_m"))
            db_max_segment_m = _safe_float(meta.get("max_segment_m"))
            source = "area_fov_db"

    search_weight = float(cfg.search_speed_weight) if float(cfg.search_speed_weight or 0.0) > 0.0 else _runtime_float(
        "area_search_speed_weight",
        _d0303_default("AREA_SEARCH_SPEED_WEIGHT", 1.0),
    )
    turn_radius = float(cfg.turn_radius_m) if float(cfg.turn_radius_m or 0.0) > 0.0 else _runtime_float(
        "dubins_turn_radius_m",
        _d0303_default("DUBINS_TURN_RADIUS_M", 500.0),
    )
    turn_step = float(cfg.turn_step_deg) if float(cfg.turn_step_deg or 0.0) > 0.0 else _runtime_float(
        "turn_step_deg",
        15.0,
    )
    if _runtime_bool("line_sweep_interpolation_enabled", False):
        interpolation_points = int(round(_runtime_float(
            "sweep_line_interp_points",
            _d0303_default("SWEEP_LINE_INTERP_POINTS", 3),
        )))
        interpolation_points = max(2, int(interpolation_points))
    else:
        interpolation_points = 2
    min_speed, max_speed = _capture_speed_window()
    return MissionPlanningProfile(
        source=source,
        strategy=str(cfg.strategy).lower(),
        bearing_deg=float(bearing_deg),
        raw_fov_deg=float(raw_fov_deg),
        fov_deg=float(fov_deg),
        separation_m=float(separation_m),
        turn_radius_m=float(turn_radius),
        turn_step_deg=float(turn_step),
        interpolation_points=int(interpolation_points),
        search_speed_weight=float(search_weight),
        camera_speed_cap_mps=float(cfg.camera_speed_cap_mps),
        min_aircraft_speed_mps=float(min_speed),
        max_aircraft_speed_mps=float(max_speed),
        db_config=db_config,
        db_span_m=db_span_m,
        db_width_ref_m=db_width_ref_m,
        db_max_segment_m=db_max_segment_m,
    )


# ---------------------------------------------------------------------------
# Patrol plan construction
# ---------------------------------------------------------------------------


def build_patrol_plans(mission: DonutMission, config: PatrolConfig | None = None) -> list[AircraftPlan]:
    cfg = config or PatrolConfig()
    aircraft_ids = _resolve_aircraft_ids(cfg)
    if not aircraft_ids:
        return []
    if not mission.hole_polygons_xy:
        raise ValueError("donut mission has no hole polygon")
    strategy = str(cfg.strategy).lower()
    if strategy not in ("band", "windmill"):
        raise ValueError(f"unknown strategy: {cfg.strategy!r} (expected 'band' or 'windmill')")

    profile = resolve_planning_profile(mission, cfg)
    hole_union = unary_union(mission.hole_polygons_xy)
    center = hole_union.centroid
    center_xy = (float(center.x), float(center.y))
    gap_min = max(1.0, float(hole_union.distance(mission.polygon_xy.exterior)))

    count = len(aircraft_ids)
    altitudes = [
        _aircraft_altitude_m(int(aid), int(cfg.altitude_m or 0)) for aid in aircraft_ids
    ]
    specs = [
        _resolve_capture_law(profile, cfg, altitude_m=alt) for alt in altitudes
    ]  # list of dicts: spacing / footprint / speed plan

    lane_offsets, effective_turn_radius, turn_violated = _resolve_lane_offsets(
        cfg,
        profile,
        gap_min=gap_min,
        count=count,
        strategy=strategy,
    )
    directions = _resolve_directions(cfg, strategy, count)
    phases = _resolve_phases(cfg, count)

    # Radial rays. band: each aircraft gets its own ray set spaced on its band
    # outer edge. windmill: one shared set spaced on the outer boundary using
    # the tightest per-aircraft spacing, assigned round-robin.
    band_fractions = _band_fractions(strategy, count, float(cfg.band_overlap_fraction))
    rays_per_aircraft: list[list[RadialRay]] = []
    if strategy == "band":
        for idx in range(count):
            _f_in, f_out = band_fractions[idx]
            rays = _trace_station_rays(
                mission,
                center_xy,
                station_fraction=f_out,
                spacing_m=float(specs[idx]["sweep_spacing_m"]),
            )
            rays_per_aircraft.append(rays)
    else:
        shared_spacing = min(float(spec["sweep_spacing_m"]) for spec in specs)
        shared = _trace_station_rays(
            mission,
            center_xy,
            station_fraction=1.0,
            spacing_m=shared_spacing,
        )
        for idx in range(count):
            rays_per_aircraft.append(shared[idx::count])

    plans: list[AircraftPlan] = []
    for idx, aircraft_id in enumerate(aircraft_ids):
        f_in, f_out = band_fractions[idx]
        loop = _build_lane_loop(
            mission,
            hole_union,
            lane_offsets[idx],
            turn_step_deg=float(profile.turn_step_deg),
        )
        plan = _build_single_plan(
            mission,
            cfg,
            profile,
            aircraft_id=int(aircraft_id),
            spec=specs[idx],
            rays=rays_per_aircraft[idx],
            band_fraction=(f_in, f_out),
            center_xy=center_xy,
            loop=loop,
            direction=directions[idx],
            phase_fraction=phases[idx],
            lane_offset_m=float(lane_offsets[idx]),
            turn_radius_m=float(effective_turn_radius),
            turn_radius_violated=bool(turn_violated),
            gap_min_m=float(gap_min),
            altitude_m=int(altitudes[idx]),
        )
        plans.append(plan)
    return plans


def _build_single_plan(
    mission: DonutMission,
    cfg: PatrolConfig,
    profile: MissionPlanningProfile,
    *,
    aircraft_id: int,
    spec: dict[str, Any],
    rays: list[RadialRay],
    band_fraction: tuple[float, float],
    center_xy: tuple[float, float],
    loop: LineString,
    direction: str,
    phase_fraction: float,
    lane_offset_m: float,
    turn_radius_m: float,
    turn_radius_violated: bool,
    gap_min_m: float,
    altitude_m: int,
) -> AircraftPlan:
    f_in, f_out = band_fraction
    loop_length = max(float(loop.length), 1.0)
    ccw = str(direction).lower() != "cw"

    # Station entries: sweep segment + route anchor on the lane loop.
    stations: list[dict[str, Any]] = []
    for ray in rays:
        sweep_start = ray.point_at(f_in)
        sweep_end = ray.point_at(f_out)
        sweep_len = _distance(sweep_start, sweep_end)
        if sweep_len < 0.5:
            continue
        # Route anchor = ray x lane intersection. A nearest-point projection
        # would pile corner stations onto a short stretch of the lane corner
        # arc and collapse the transit windows (camera-limited); the radial
        # intersection keeps the along-lane arcs proportional to the fan.
        wp_xy = _ray_lane_point(loop, center_xy, ray)
        s_pos = float(loop.project(Point(wp_xy)))
        stations.append(
            {
                "s": s_pos % loop_length,
                "wp_xy": (float(wp_xy[0]), float(wp_xy[1])),
                "sweep_start": sweep_start,
                "sweep_end": sweep_end,
                "sweep_len": float(sweep_len),
            }
        )
    if len(stations) < 3:
        raise ValueError(f"aircraft {aircraft_id}: not enough sweep stations ({len(stations)})")

    # Order along the travel direction starting at the phase offset.
    start_s = (float(phase_fraction) % 1.0) * loop_length

    def travel(s: float) -> float:
        return ((s - start_s) % loop_length) if ccw else ((start_s - s) % loop_length)

    stations.sort(key=lambda row: travel(row["s"]))

    # Route nodes: only feature points (lane corners within the turn
    # tolerance) plus straight-leg fillers at the route WP spacing. The
    # stations of each leg are packed into the departure waypoint's
    # lineSearch as one zigzag polyline (production 0303 grouping).
    spacing_max = _route_wp_spacing(cfg)
    coord_cap = _max_linesearch_coords(cfg)
    interp = max(2, int(profile.interpolation_points))
    extra_ps: list[float] = []
    for _split_pass in range(8):
        nodes = _route_nodes(
            loop,
            start_s=start_s,
            ccw=ccw,
            spacing_max_m=spacing_max,
            min_separation_m=float(cfg.min_wp_separation_m),
            turn_eps_deg=float(cfg.corner_turn_eps_deg),
            extra_ps=extra_ps,
        )
        node_ps = [node["p"] for node in nodes]
        n_legs = len(nodes)
        leg_stations: list[list[dict[str, Any]]] = [[] for _ in range(n_legs)]
        for station in stations:
            p = travel(float(station["s"]))
            # stations before the first node wrap onto the closing leg
            leg_stations[(_bisect_right(node_ps, p) - 1) % n_legs].append(station)
        over = [idx for idx in range(n_legs) if len(leg_stations[idx]) * interp > coord_cap]
        if not over:
            break
        # split every over-full leg at its median station so the zigzags fit
        for idx in over:
            members = leg_stations[idx]
            median = members[len(members) // 2]
            extra_ps.append(travel(float(median["s"])))

    # Per-station sweep records; zigzag parity alternates in travel order so
    # the packed polyline snakes across the band without long back-hops.
    sweep_id_base = int(aircraft_id) * 100_000
    sweep_lines: list[SweepLine] = []
    for order, station in enumerate(stations):
        station["flip"] = bool(order % 2)
        sweep = SweepLine(
            sweep_line_id=sweep_id_base + order + 1,
            start_x=float(station["sweep_start"][0]),
            start_y=float(station["sweep_start"][1]),
            end_x=float(station["sweep_end"][0]),
            end_y=float(station["sweep_end"][1]),
            start_coordinate=mission.frame.to_latlon(*station["sweep_start"], altitude=altitude_m),
            end_coordinate=mission.frame.to_latlon(*station["sweep_end"], altitude=altitude_m),
            sweep_length_m=float(station["sweep_len"]),
        )
        station["sweep"] = sweep
        sweep_lines.append(sweep)

    # Zigzag polyline geometry per leg.
    leg_geometry: list[dict[str, Any] | None] = []
    for idx in range(n_legs):
        members = leg_stations[idx]
        if not members:
            leg_geometry.append(None)
            continue
        coords_xy: list[tuple[float, float]] = []
        coordinate_list: list[dict[str, Any]] = []
        sweep_ids: list[int] = []
        member_last_index: list[int] = []
        for station in members:
            points = _interpolated_xy_points(
                station["sweep_start"], station["sweep_end"], points=interp
            )
            if station["flip"]:
                points.reverse()
            coords_xy.extend(points)
            member_last_index.append(len(coords_xy) - 1)
            sweep_ids.append(int(station["sweep"].sweep_line_id))
        coordinate_list = [
            mission.frame.to_latlon(x, y, altitude=altitude_m) for x, y in coords_xy
        ]
        cumulative_m = [0.0]
        for left, right in zip(coords_xy, coords_xy[1:]):
            cumulative_m.append(cumulative_m[-1] + _distance(left, right))
        leg_geometry.append(
            {
                "coords_xy": coords_xy,
                "coordinate_list": coordinate_list,
                "cumulative_m": cumulative_m,
                "length_m": cumulative_m[-1],
                "sweep_ids": sweep_ids,
                "members": members,
                "member_last_index": member_last_index,
            }
        )

    leg_chords = [
        max(_distance(nodes[idx]["xy"], nodes[(idx + 1) % n_legs]["xy"]), 1.0)
        for idx in range(n_legs)
    ]
    route_min_leg = min(leg_chords) if leg_chords else 0.0
    route_max_turn = 0.0
    for idx in range(n_legs):
        h_in = _heading_deg(nodes[(idx - 1) % n_legs]["xy"], nodes[idx]["xy"])
        h_out = _heading_deg(nodes[idx]["xy"], nodes[(idx + 1) % n_legs]["xy"])
        turn = abs((h_out - h_in + 180.0) % 360.0 - 180.0)
        route_max_turn = max(route_max_turn, turn)

    # Aircraft speed: capture-law plan speed, clamped so every leg's zigzag
    # finishes inside the leg transit window at <= the camera speed cap.
    cam_cap = max(1.0, float(profile.camera_speed_cap_mps))
    v_plan = float(spec["plan_speed_mps"])
    leg_caps = [
        leg_chords[idx] * cam_cap / max(float(leg_geometry[idx]["length_m"]), 1e-6)
        for idx in range(n_legs)
        if leg_geometry[idx] is not None and float(leg_geometry[idx]["length_m"]) > 0.0
    ]
    v_cam_cap = min(leg_caps, default=v_plan)
    camera_limited = bool(spec["plan_camera_limited"])
    if float(cfg.speed_mps or 0.0) > 0.0:
        speed = float(cfg.speed_mps)
        camera_limited = camera_limited or speed > v_cam_cap + 1e-6
    else:
        speed = min(v_plan, v_cam_cap)
        if speed < float(profile.min_aircraft_speed_mps):
            speed = float(profile.min_aircraft_speed_mps)
            camera_limited = True
    speed = max(speed, 1.0)

    # Per-leg camera speed (ground speed of the packed polyline).
    weight = float(profile.search_speed_weight)
    leg_search_speed: list[float] = []
    for idx in range(n_legs):
        geometry = leg_geometry[idx]
        if geometry is None:
            leg_search_speed.append(0.0)
            continue
        transit_s = leg_chords[idx] / speed
        raw = float(geometry["length_m"]) / max(transit_s, 1e-6)
        if raw > cam_cap * (1.0 + 1e-9):
            camera_limited = True
        leg_search_speed.append(round(min(raw * max(weight, 1e-6), cam_cap), 2))

    # Waypoints: nodes per lap + a closing FLYOVER back at the start.
    waypoint_id_base = int(aircraft_id) * 100_000
    group_id_base = int(aircraft_id) * 100_000
    laps = max(1, int(cfg.laps))
    waypoints: list[Waypoint] = []
    cumulative = 0.0
    group_count = 0
    for lap in range(1, laps + 1):
        for idx, node in enumerate(nodes):
            if not (lap == 1 and idx == 0):
                cumulative += leg_chords[idx - 1] if idx > 0 else leg_chords[-1]
            coord = mission.frame.to_latlon(node["xy"][0], node["xy"][1], altitude=altitude_m)
            wp = Waypoint(
                waypoint_id=waypoint_id_base + len(waypoints) + 1,
                lap=int(lap),
                x=float(node["xy"][0]),
                y=float(node["xy"][1]),
                latitude=float(coord["latitude"]),
                longitude=float(coord["longitude"]),
                altitude=int(altitude_m),
                speed_mps=float(speed),
                heading_deg=0.0,
                eta_s=float(cumulative / speed),
                cumulative_m=float(cumulative),
                fov_deg=float(profile.fov_deg),
                sensor_type=int(cfg.sensor_type),
            )
            geometry = leg_geometry[idx]
            if geometry is not None:
                group_count += 1
                wp.line_search = LineSearchGroup(
                    group_id=group_id_base + group_count,
                    coords_xy=list(geometry["coords_xy"]),
                    coordinate_list=[dict(row) for row in geometry["coordinate_list"]],
                    cumulative_m=list(geometry["cumulative_m"]),
                    search_speed_mps=float(leg_search_speed[idx]),
                    length_m=float(geometry["length_m"]),
                    sweep_line_ids=list(geometry["sweep_ids"]),
                    interpolation_points=int(interp),
                )
            waypoints.append(wp)
    cumulative += leg_chords[-1]
    closing = mission.frame.to_latlon(nodes[0]["xy"][0], nodes[0]["xy"][1], altitude=altitude_m)
    waypoints.append(
        Waypoint(
            waypoint_id=waypoint_id_base + len(waypoints) + 1,
            lap=int(laps),
            x=float(nodes[0]["xy"][0]),
            y=float(nodes[0]["xy"][1]),
            latitude=float(closing["latitude"]),
            longitude=float(closing["longitude"]),
            altitude=int(altitude_m),
            speed_mps=float(speed),
            heading_deg=0.0,
            eta_s=float(cumulative / speed),
            cumulative_m=float(cumulative),
            waypoint_pass_type=PASS_FLYOVER,
            fov_deg=float(profile.fov_deg),
            sensor_type=int(cfg.sensor_type),
        )
    )
    _fill_headings_and_links(waypoints)

    # Lap-1 reveal times for the coverage timeline / animation: the camera
    # runs each leg's polyline once per leg transit.
    lap_length = sum(leg_chords)
    for idx in range(n_legs):
        geometry = leg_geometry[idx]
        if geometry is None:
            continue
        leg_start_m = sum(leg_chords[:idx])
        t0 = leg_start_m / speed
        t1 = (leg_start_m + leg_chords[idx]) / speed
        total = max(float(geometry["length_m"]), 1e-6)
        for station, last_idx in zip(geometry["members"], geometry["member_last_index"]):
            fraction = float(geometry["cumulative_m"][last_idx]) / total
            station["sweep"].reveal_eta_s = t0 + (t1 - t0) * fraction

    footprint_union = _build_strips(
        mission,
        sweep_lines,
        coverage_width_m=float(spec["coverage_width_m"]),
        end_extension_m=float(spec["footprint_h_m"]) * 0.5,
    )
    donut_area = max(float(mission.polygon_xy.area), 1e-6)
    coverage_ratio = float(footprint_union.area) / donut_area if not footprint_union.is_empty else 0.0

    max_sweep_len = max((row["sweep_len"] for row in stations), default=0.0)
    camera_speed_used = max(leg_search_speed, default=0.0)
    capture_spec = CaptureSpec(
        altitude_m=int(altitude_m),
        footprint_horizontal_m=float(spec["footprint_h_m"]),
        footprint_vertical_m=float(spec["footprint_v_m"]),
        sweep_spacing_m=float(spec["sweep_spacing_m"]),
        coverage_width_m=float(spec["coverage_width_m"]),
        aircraft_speed_mps=float(speed),
        aircraft_speed_kmh=float(speed) * 3.6,
        scan_interval_mean_s=float((lap_length / speed) / max(len(stations), 1)),
        max_sweep_length_m=float(max_sweep_len),
        camera_speed_max_used_mps=float(camera_speed_used),
        camera_limited=bool(camera_limited),
    )
    return AircraftPlan(
        aircraft_id=int(aircraft_id),
        direction="ccw" if ccw else "cw",
        strategy=str(profile.strategy),
        band_inner_fraction=float(f_in),
        band_outer_fraction=float(f_out),
        lane_offset_m=float(lane_offset_m),
        turn_radius_m=float(turn_radius_m),
        loop_length_m=float(loop_length),
        altitude_m=int(altitude_m),
        phase_fraction=float(phase_fraction),
        min_hole_boundary_gap_m=float(gap_min_m),
        turn_radius_violated=bool(turn_radius_violated),
        route_nodes_per_lap=int(n_legs),
        route_wp_spacing_m=float(spacing_max),
        min_wp_separation_m=float(cfg.min_wp_separation_m),
        route_min_leg_m=float(route_min_leg),
        route_max_turn_deg=float(route_max_turn),
        max_linesearch_coords=int(coord_cap),
        capture_spec=capture_spec,
        planning_profile=profile,
        waypoints=waypoints,
        sweep_lines=sweep_lines,
        footprint_union=footprint_union,
        coverage_ratio=float(coverage_ratio),
    )


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------


def simulate_patrol(
    plans: list[AircraftPlan],
    *,
    dt_s: float = 5.0,
) -> list[dict[str, Any]]:
    duration = max((plan.waypoints[-1].eta_s for plan in plans if plan.waypoints), default=0.0)
    if duration <= 0.0:
        return []
    frames: list[dict[str, Any]] = []
    steps = max(1, int(math.ceil(duration / max(float(dt_s), 0.1))))
    sweep_timelines = {plan.aircraft_id: _sweep_timeline(plan) for plan in plans}
    for step in range(steps + 1):
        t = min(duration, float(step) * float(dt_s))
        aircraft = []
        for plan in plans:
            state = _interpolate_plan(plan, t)
            if state is None:
                continue
            camera = _interpolate_camera(sweep_timelines[plan.aircraft_id], t)
            if camera is not None:
                state["camera"] = camera
            aircraft.append(state)
        frames.append({"timeS": round(t, 3), "aircraft": aircraft})
    return frames


def build_coverage_timeline(
    mission: DonutMission,
    plans: list[AircraftPlan],
) -> list[dict[str, Any]]:
    """First-lap sweep reveals with the cumulative covered ratio at each one."""
    events: list[tuple[float, int, SweepLine]] = []
    for plan in plans:
        for sweep in plan.sweep_lines:
            events.append((float(sweep.reveal_eta_s), int(plan.aircraft_id), sweep))
    events.sort(key=lambda row: row[0])
    donut_area = max(float(mission.polygon_xy.area), 1e-6)
    covered: BaseGeometry = Polygon()
    out: list[dict[str, Any]] = []
    for eta, aircraft_id, sweep in events:
        if sweep.strip is not None and not sweep.strip.is_empty:
            covered = covered.union(sweep.strip)
        out.append(
            {
                "timeS": round(float(eta), 3),
                "aircraftID": int(aircraft_id),
                "sweepLineID": int(sweep.sweep_line_id),
                "cumulativeCoverageRatio": round(float(covered.area) / donut_area, 5),
            }
        )
    return out


def plans_to_jsonable(mission: DonutMission, plans: list[AircraftPlan]) -> dict[str, Any]:
    profile = plans[0].planning_profile if plans and plans[0].planning_profile is not None else None
    donut_area = max(float(mission.polygon_xy.area), 1e-6)
    unions = [
        plan.footprint_union
        for plan in plans
        if plan.footprint_union is not None and not plan.footprint_union.is_empty
    ]
    combined = unary_union(unions) if unions else Polygon()
    uncovered_area = max(0.0, donut_area - float(combined.area))
    largest_gap = 0.0
    if not combined.is_empty:
        residual = mission.polygon_xy.difference(combined)
        if not residual.is_empty:
            pieces = getattr(residual, "geoms", [residual])
            largest_gap = max(float(piece.area) for piece in pieces)
    row = {
        "sourceInputMissionID": int(mission.input_mission_id),
        "sourceInputMissionType": int(mission.input_mission_type),
        "sourceRegionType": int(mission.region_type),
        "outerPointCount": len(mission.outer_latlon),
        "holeCount": len(mission.holes_latlon),
        "donutAreaM2": round(donut_area, 3),
        "coverage": {
            "combinedRatio": round(float(combined.area) / donut_area, 5),
            "uncoveredAreaM2": round(uncovered_area, 3),
            "largestGapM2": round(largest_gap, 3),
            "perAircraftRatio": {
                str(plan.aircraft_id): round(float(plan.coverage_ratio), 5) for plan in plans
            },
        },
        "cameraFeasible": all(
            plan.capture_spec is None or not plan.capture_spec.camera_limited for plan in plans
        ),
        "aircraftPlanList": [plan.as_dict() for plan in plans],
    }
    if profile is not None:
        row["planningProfile"] = profile.as_dict()
    return row


# ---------------------------------------------------------------------------
# Capture-law / runtime bridges
# ---------------------------------------------------------------------------


def _resolve_capture_law(
    profile: MissionPlanningProfile,
    cfg: PatrolConfig,
    *,
    altitude_m: int,
) -> dict[str, Any]:
    """FOV -> footprint/spacing/speed at one altitude layer.

    Falls back to the legacy separation formula when capture_geometry is not
    importable (offline runs), keeping the prototype usable stand-alone.
    """
    fov = float(profile.fov_deg)
    footprint_h = 0.0
    footprint_v = 0.0
    spacing = float(cfg.sweep_spacing_m or 0.0)
    plan_speed = float(profile.min_aircraft_speed_mps)
    plan_camera_limited = False
    if _capture is not None:
        footprint = _capture.nadir_footprint_m(fov, altitude_m=float(altitude_m))
        if footprint is not None:
            footprint_h = float(footprint["horizontalM"])
            footprint_v = float(footprint["verticalM"])
        if spacing <= 0.0:
            law_spacing = _capture.area_vertical_sweep_spacing_m(fov, altitude_m=float(altitude_m))
            if law_spacing is not None and law_spacing > 0.0:
                spacing = float(law_spacing)
        plan = _capture.capture_speed_plan(fov, altitude_m=float(altitude_m))
        if plan is not None:
            plan_speed = float(plan["aircraftSpeedMps"])
            plan_camera_limited = bool(plan.get("cameraLimited", False))
    if spacing <= 0.0:
        spacing = 2.0 * max(float(profile.separation_m), 1.0) * math.tan(
            max(math.radians(fov) / 2.0, 1e-6)
        )
    spacing = max(float(spacing), 1.0)
    if footprint_v <= 0.0:
        # No footprint available: assume the 20% overlap law backwards.
        footprint_v = spacing / 0.8
    if footprint_h <= 0.0:
        footprint_h = footprint_v * (16.0 / 9.0)
    return {
        "sweep_spacing_m": float(spacing),
        "footprint_h_m": float(footprint_h),
        "footprint_v_m": float(footprint_v),
        # physical strip width; a spacing forced wider than the footprint
        # must show up as measured coverage loss, not get papered over
        "coverage_width_m": float(footprint_v),
        "plan_speed_mps": float(plan_speed),
        "plan_camera_limited": bool(plan_camera_limited),
    }


def _capture_speed_window() -> tuple[float, float]:
    if _capture is not None:
        try:
            params = _capture.capture_params()
            return (
                float(params["min_aircraft_speed_mps"]),
                float(params["max_aircraft_speed_mps"]),
            )
        except Exception:
            pass
    return (40.0, 55.0)


def _aircraft_altitude_m(aircraft_id: int, explicit_altitude_m: int) -> int:
    if explicit_altitude_m > 0:
        return int(explicit_altitude_m)
    if _d0303 is not None and hasattr(_d0303, "_aircraft_alt_offset_m"):
        try:
            return int(round(float(_d0303._aircraft_alt_offset_m(int(aircraft_id)))))
        except Exception:
            pass
    return int(1000 + ((int(aircraft_id) - 4) % 3) * 10)


def _d0303_default(name: str, default: float) -> float:
    if _d0303 is None:
        return float(default)
    try:
        return float(getattr(_d0303, name))
    except Exception:
        return float(default)


def _runtime_float(name: str, default: float) -> float:
    if _d0303 is not None and hasattr(_d0303, "_runtime_float_setting"):
        try:
            return float(_d0303._runtime_float_setting(name, float(default)))
        except Exception:
            pass
    return float(default)


def _runtime_bool(name: str, default: bool) -> bool:
    if _d0303 is not None and hasattr(_d0303, "_runtime_bool_setting"):
        try:
            return bool(_d0303._runtime_bool_setting(name, bool(default)))
        except Exception:
            pass
    return bool(default)


def _runtime_manual_fov(name: str, default: float) -> float:
    if _d0303 is not None and hasattr(_d0303, "_runtime_manual_fov_deg"):
        try:
            return float(_d0303._runtime_manual_fov_deg(name, float(default)))
        except Exception:
            pass
    return float(default)


def _apply_adjusted_db_fov(raw_fov_deg: float) -> float:
    if _d0303 is not None and hasattr(_d0303, "_apply_runtime_adjusted_db_fov"):
        try:
            return float(_d0303._apply_runtime_adjusted_db_fov(float(raw_fov_deg)))
        except Exception:
            pass
    return float(raw_fov_deg)


# ---------------------------------------------------------------------------
# Partition helpers
# ---------------------------------------------------------------------------


def _band_fractions(strategy: str, count: int, overlap_fraction: float) -> list[tuple[float, float]]:
    if strategy != "band" or count <= 1:
        return [(0.0, 1.0) for _ in range(count)]
    seam = max(0.0, min(0.2, float(overlap_fraction)))
    out: list[tuple[float, float]] = []
    for idx in range(count):
        f_in = max(0.0, float(idx) / count - seam)
        f_out = 1.0 if idx == count - 1 else float(idx + 1) / count
        out.append((f_in, f_out))
    return out


def _resolve_lane_offsets(
    cfg: PatrolConfig,
    profile: MissionPlanningProfile,
    *,
    gap_min: float,
    count: int,
    strategy: str,
) -> tuple[list[float], float, bool]:
    """Lane ring offsets from the hole, inside [turn radius, gap - margin]."""
    margin = max(0.0, float(cfg.edge_margin_m))
    requested_turn_radius = max(1.0, float(profile.turn_radius_m))
    hi = max(1.0, gap_min - margin)
    turn_violated = requested_turn_radius > hi
    effective_turn_radius = min(requested_turn_radius, hi)
    lo = effective_turn_radius

    if cfg.lane_offsets_m:
        offsets = [
            max(lo, min(hi, float(_value_at(cfg.lane_offsets_m, idx, lo))))
            for idx in range(count)
        ]
        return offsets, effective_turn_radius, turn_violated

    if strategy == "windmill" or count == 1:
        mid = lo + (hi - lo) * 0.5
        return [mid for _ in range(count)], effective_turn_radius, turn_violated

    # band: aim at the band mid, clamp into the feasible span, then push apart
    offsets = []
    for idx in range(count):
        mid_fraction = (idx + 0.5) / count
        offsets.append(max(lo, min(hi, gap_min * mid_fraction)))
    min_sep = min(20.0, (hi - lo) / max(count - 1, 1)) if hi > lo else 0.0
    for idx in range(1, count):
        offsets[idx] = max(offsets[idx], offsets[idx - 1] + min_sep)
        offsets[idx] = min(offsets[idx], hi)
    return offsets, effective_turn_radius, turn_violated


def _resolve_directions(cfg: PatrolConfig, strategy: str, count: int) -> list[str]:
    if cfg.directions:
        return [str(_value_at(cfg.directions, idx, "ccw")).lower() for idx in range(count)]
    if strategy == "band":
        # counter-rotating adjacent rings; altitude layers separate them
        return ["ccw" if idx % 2 == 0 else "cw" for idx in range(count)]
    return ["ccw" for _ in range(count)]


def _resolve_phases(cfg: PatrolConfig, count: int) -> list[float]:
    if cfg.phase_fractions:
        return [float(_value_at(cfg.phase_fractions, idx, 0.0)) for idx in range(count)]
    return [float(idx) / max(count, 1) for idx in range(count)]


# ---------------------------------------------------------------------------
# Radial ray tracing
# ---------------------------------------------------------------------------


def _trace_station_rays(
    mission: DonutMission,
    center_xy: tuple[float, float],
    *,
    station_fraction: float,
    spacing_m: float,
) -> list[RadialRay]:
    """Radial rays so adjacent sweep LINES stay <= spacing apart in the band.

    The perpendicular separation between two rays with angular gap dtheta is
    at most dtheta * r, where r is the largest station radius of the pair.
    Bounding that (instead of the chord along the possibly oblique station
    curve) keeps the ray count minimal: an outer edge running oblique to the
    rays would otherwise force needlessly dense stations, collapsing the
    aircraft transit windows and making the camera infeasible.

    Construction: rays are seeded at every boundary/hole vertex angle (along
    a straight edge the station radius is convex in theta, so pair-endpoint
    checks are exact between vertex rays), then every vertex-to-vertex
    segment is bisected to measure its separation integral and re-divided
    into EQUAL separation steps just under the spacing bound. The equal
    division matters: plain bisection converges to gaps anywhere in
    (spacing/2, spacing], and the short ones would starve the camera of
    transit time.
    """
    spacing = max(1.0, float(spacing_m))
    target = spacing * 0.96
    minx, miny, maxx, maxy = mission.polygon_xy.bounds
    ray_len = max(maxx - minx, maxy - miny, 1.0) * 3.0
    two_pi = 2.0 * math.pi

    def cast(theta: float) -> RadialRay | None:
        return _cast_ray(mission, center_xy, theta % two_pi, ray_len)

    def radius(ray: RadialRay) -> float:
        return max(1.0, _distance(center_xy, ray.point_at(station_fraction)))

    # Seed with vertex rays. Two vertices almost collinear with the center
    # would pin two rays a sliver apart, starving the transit window between
    # their sweeps, so close seed pairs are merged into their angular
    # midpoint (the 20% overlap + end extension absorb the vertex offset).
    seeds: list[RadialRay] = []
    for theta in _critical_angles(mission, center_xy):
        ray = cast(theta)
        if ray is not None:
            seeds.append(ray)
    if len(seeds) < 2:
        seeds = [ray for ray in (cast(k * two_pi / 8.0) for k in range(8)) if ray is not None]
    if len(seeds) < 2:
        raise ValueError("failed to trace radial rays across the donut")
    seeds = _merge_close_seeds(seeds, cast, radius, merge_below_m=0.5 * spacing)

    rays: list[RadialRay] = []
    for idx, ray_a in enumerate(seeds):
        ray_b = seeds[(idx + 1) % len(seeds)]
        theta_b = ray_b.theta_rad if idx + 1 < len(seeds) else ray_b.theta_rad + two_pi
        rays.append(ray_a)
        rays.extend(
            _equal_separation_rays(cast, radius, ray_a, ray_b, theta_b, target)
        )

    # Wrap-segment mids normalize to small angles at the tail of the list, so
    # restore cyclic order before pairwise checks (an unsorted tail would fake
    # a near-360-degree gap and inject a phantom ray at its midpoint).
    rays = sorted(rays, key=lambda row: row.theta_rad)

    # Safety pass: the equal division interpolates the separation integral,
    # so re-check the final pairs and bisect the rare violator.
    for _pass in range(4):
        inserted = False
        repaired: list[RadialRay] = []
        for idx, ray in enumerate(rays):
            repaired.append(ray)
            nxt = rays[(idx + 1) % len(rays)]
            theta_next = nxt.theta_rad if idx + 1 < len(rays) else nxt.theta_rad + two_pi
            gap_theta = theta_next - ray.theta_rad
            if gap_theta <= math.radians(0.02):
                continue
            if gap_theta * max(radius(ray), radius(nxt)) > spacing * 1.001:
                mid = cast(ray.theta_rad + gap_theta * 0.5)
                if mid is not None:
                    repaired.append(mid)
                    inserted = True
        rays = sorted(repaired, key=lambda row: row.theta_rad)
        if not inserted:
            break
    if len(rays) < 4:
        raise ValueError("failed to trace radial rays across the donut")
    return rays


def _equal_separation_rays(
    cast,
    radius,
    ray_a: RadialRay,
    ray_b: RadialRay,
    theta_b_unwrapped: float,
    target_m: float,
) -> list[RadialRay]:
    """Intermediate rays dividing [a, b] into equal separation steps <= target.

    The separation integral (sum of dtheta * max-pair-radius) is measured by
    bisecting the segment until every quadrature interval is below target,
    then the interval is re-divided at equal fractions of that integral.
    """
    two_pi = 2.0 * math.pi
    nodes: list[tuple[float, RadialRay]] = [
        (float(ray_a.theta_rad), ray_a),
        (float(theta_b_unwrapped), ray_b),
    ]
    for _pass in range(24):
        refined: list[tuple[float, RadialRay]] = []
        inserted = False
        for (t0, r0), (t1, r1) in zip(nodes, nodes[1:]):
            refined.append((t0, r0))
            if t1 - t0 <= math.radians(0.02):
                continue
            if (t1 - t0) * max(radius(r0), radius(r1)) > target_m:
                mid = cast((t0 + t1) * 0.5)
                if mid is not None:
                    refined.append(((t0 + t1) * 0.5, mid))
                    inserted = True
        refined.append(nodes[-1])
        nodes = refined
        if not inserted or len(nodes) > 4096:
            break

    separations = [
        (t1 - t0) * max(radius(r0), radius(r1))
        for (t0, r0), (t1, r1) in zip(nodes, nodes[1:])
    ]
    total = sum(separations)
    count = int(math.ceil(total / max(target_m, 1.0)))
    if count <= 1:
        return []
    out: list[RadialRay] = []
    cumulative = 0.0
    node_idx = 0
    for k in range(1, count):
        goal = total * float(k) / float(count)
        while node_idx < len(separations) and cumulative + separations[node_idx] < goal:
            cumulative += separations[node_idx]
            node_idx += 1
        if node_idx >= len(separations):
            break
        t0 = nodes[node_idx][0]
        t1 = nodes[node_idx + 1][0]
        seg = separations[node_idx]
        frac = (goal - cumulative) / max(seg, 1e-9)
        ray = cast((t0 + (t1 - t0) * frac) % two_pi)
        if ray is not None:
            out.append(ray)
    return out


def _merge_close_seeds(
    seeds: list[RadialRay],
    cast,
    radius,
    *,
    merge_below_m: float,
) -> list[RadialRay]:
    """Replace seed pairs closer than merge_below_m with their midpoint ray."""
    two_pi = 2.0 * math.pi
    seeds = sorted(seeds, key=lambda row: row.theta_rad)
    for _pass in range(6):
        if len(seeds) <= 2:
            break
        changed = False
        out: list[RadialRay] = []
        skip_next = False
        for idx, ray in enumerate(seeds):
            if skip_next:
                skip_next = False
                continue
            if idx + 1 < len(seeds):
                nxt = seeds[idx + 1]
                gap = nxt.theta_rad - ray.theta_rad
                if gap * max(radius(ray), radius(nxt)) < merge_below_m:
                    mid = cast(ray.theta_rad + gap * 0.5)
                    out.append(mid if mid is not None else ray)
                    skip_next = True
                    changed = True
                    continue
            out.append(ray)
        if len(out) > 2:
            wrap_gap = (out[0].theta_rad + two_pi) - out[-1].theta_rad
            if wrap_gap * max(radius(out[0]), radius(out[-1])) < merge_below_m:
                mid = cast(out[-1].theta_rad + wrap_gap * 0.5)
                last = out.pop()
                out.pop(0)
                out.append(mid if mid is not None else last)
                changed = True
        seeds = sorted(out, key=lambda row: row.theta_rad)
        if not changed:
            break
    return seeds


def _critical_angles(mission: DonutMission, center_xy: tuple[float, float]) -> list[float]:
    """Angles of every outer/hole vertex as seen from the ray origin."""
    cx, cy = center_xy
    angles: list[float] = []
    rings = [mission.polygon_xy.exterior, *mission.polygon_xy.interiors]
    for ring in rings:
        for x, y in ring.coords:
            dx, dy = float(x) - cx, float(y) - cy
            if math.hypot(dx, dy) < 1.0:
                continue
            angles.append(math.atan2(dy, dx) % (2.0 * math.pi))
    return sorted(set(angles))


def _cast_ray(
    mission: DonutMission,
    center_xy: tuple[float, float],
    theta_rad: float,
    ray_len: float,
) -> RadialRay | None:
    cx, cy = center_xy
    ux, uy = math.cos(theta_rad), math.sin(theta_rad)
    ray = LineString([(cx, cy), (cx + ux * ray_len, cy + uy * ray_len)])
    pieces = _line_candidates(ray.intersection(mission.polygon_xy))
    if not pieces:
        return None
    origin = Point(cx, cy)
    piece = min(pieces, key=lambda line: line.distance(origin))
    projections = [
        ((float(px) - cx) * ux + (float(py) - cy) * uy, (float(px), float(py)))
        for px, py in piece.coords
    ]
    projections.sort(key=lambda row: row[0])
    inner = projections[0][1]
    outer = projections[-1][1]
    if projections[-1][0] - projections[0][0] < 0.5:
        return None
    return RadialRay(theta_rad=float(theta_rad % (2.0 * math.pi)), inner_xy=inner, outer_xy=outer)


# ---------------------------------------------------------------------------
# Lane loop / route geometry
# ---------------------------------------------------------------------------


def _build_lane_loop(
    mission: DonutMission,
    hole_union: BaseGeometry,
    lane_offset_m: float,
    *,
    turn_step_deg: float,
) -> LineString:
    arc_resolution = max(4, int(math.ceil(90.0 / max(float(turn_step_deg), 1.0))))
    lane_area = hole_union.buffer(max(float(lane_offset_m), 1.0), resolution=arc_resolution, join_style=1)
    lane_area = orient(_largest_polygon(lane_area), sign=1.0)  # exterior CCW
    line = LineString(lane_area.exterior.coords)
    if mission.polygon_xy.buffer(1.0).contains(line):
        return line
    clipped = line.intersection(mission.polygon_xy.buffer(-1.0))
    if clipped.is_empty:
        clipped = line.intersection(mission.polygon_xy)
    loop = _as_longest_line(clipped)
    if loop.length <= 1.0:
        raise ValueError("failed to build patrol lane inside donut polygon")
    return loop


def _ray_lane_point(
    loop: LineString,
    center_xy: tuple[float, float],
    ray: RadialRay,
) -> tuple[float, float]:
    """Intersection of the radial ray with the lane loop (nearest to center).

    Falls back to the nearest-point projection of the ray's inner end when the
    clipped loop does not cross this ray (non-star-shaped edge cases).
    """
    cx, cy = center_xy
    far_x = cx + (ray.outer_xy[0] - cx) * 2.0
    far_y = cy + (ray.outer_xy[1] - cy) * 2.0
    ray_line = LineString([(cx, cy), (far_x, far_y)])
    hits: list[tuple[float, float]] = []
    intersection = loop.intersection(ray_line)
    stack = [intersection]
    while stack:
        geom = stack.pop()
        if geom.is_empty:
            continue
        geoms = getattr(geom, "geoms", None)
        if geoms is not None:
            stack.extend(geoms)
            continue
        if isinstance(geom, Point):
            hits.append((float(geom.x), float(geom.y)))
        elif isinstance(geom, LineString):
            coords = list(geom.coords)
            hits.append((float(coords[0][0]), float(coords[0][1])))
            hits.append((float(coords[-1][0]), float(coords[-1][1])))
    if hits:
        return min(hits, key=lambda pt: _distance(center_xy, pt))
    fallback = loop.interpolate(loop.project(Point(ray.inner_xy)))
    return (float(fallback.x), float(fallback.y))


def _largest_polygon(geometry: BaseGeometry) -> Polygon:
    if isinstance(geometry, Polygon):
        return geometry
    geoms = [geom for geom in getattr(geometry, "geoms", []) if isinstance(geom, Polygon)]
    if not geoms:
        raise ValueError("buffer produced no polygon")
    return max(geoms, key=lambda row: row.area)


def _route_wp_spacing(cfg: PatrolConfig) -> float:
    """Max distance between route waypoints on straight legs."""
    if float(cfg.route_wp_spacing_m or 0.0) > 0.0:
        return max(float(cfg.route_wp_spacing_m), 100.0)
    return max(
        _runtime_float("uav_wp_interval_m", _d0303_default("SWEEP_ROUTE_WP_SPACING_M", 2000.0)),
        100.0,
    )


def _max_linesearch_coords(cfg: PatrolConfig) -> int:
    if int(cfg.max_linesearch_coords or 0) > 0:
        return max(int(cfg.max_linesearch_coords), 6)
    value = _runtime_float(
        "max_linesearch_coords_per_waypoint",
        _d0303_default("MAX_LINESEARCH_COORDS_PER_WAYPOINT", 2000.0),
    )
    return max(int(round(value)), 6)


def _route_nodes(
    loop: LineString,
    *,
    start_s: float,
    ccw: bool,
    spacing_max_m: float,
    min_separation_m: float,
    turn_eps_deg: float = 3.0,
    extra_ps: Iterable[float] = (),
) -> list[dict[str, Any]]:
    """Route waypoint nodes on the lane loop: corner entry/exit + fillers.

    Each rounded corner of the lane loop contributes exactly TWO waypoints
    (arc entry and arc exit; the arc itself is left to the aircraft's own
    turn logic). A shallow corner whose entry-exit chord is shorter than
    min_separation_m collapses into a single apex waypoint. Straight legs
    longer than spacing_max_m get evenly spaced fillers, and every kept node
    respects min_separation_m — the real aircraft cannot track waypoints
    packed tighter than that.

    Nodes are returned in travel order (p = travel distance from the phase
    start along the loop).
    """
    length = max(float(loop.length), 1.0)
    min_sep = max(1.0, float(min_separation_m))

    def travel(s: float) -> float:
        return ((s - start_s) % length) if ccw else ((start_s - s) % length)

    coords = list(loop.coords)
    if len(coords) > 1 and coords[0] == coords[-1]:
        coords = coords[:-1]
    n = len(coords)

    # Turning vertices. A buffer-generated loop has NO interior vertices on
    # its straight edges (every vertex sits on a corner arc), so corners are
    # detected as runs of turning vertices whose along-loop gaps stay short;
    # a long edge (>= min separation) is a straight and breaks the run.
    turning_idx: list[int] = []
    for idx in range(n):
        h_in = _heading_deg(coords[(idx - 1) % n], coords[idx])
        h_out = _heading_deg(coords[idx], coords[(idx + 1) % n])
        diff = abs((h_out - h_in + 180.0) % 360.0 - 180.0)
        if diff >= float(turn_eps_deg):
            turning_idx.append(idx)

    def along_gap(a: int, b: int) -> float:
        total = 0.0
        idx = a
        while idx != b:
            total += _distance(coords[idx], coords[(idx + 1) % n])
            idx = (idx + 1) % n
            if total > length:
                break
        return total

    # (p, priority) candidates; lower priority number wins a merge.
    PRIO_CORNER, PRIO_START, PRIO_SPLIT = 0, 1, 2
    candidates: list[tuple[float, int]] = [(0.0, PRIO_START)]
    candidates.extend((float(p) % length, PRIO_SPLIT) for p in extra_ps)

    regions: list[tuple[int, int]] = []
    if turning_idx:
        gaps = [
            along_gap(turning_idx[k], turning_idx[(k + 1) % len(turning_idx)])
            for k in range(len(turning_idx))
        ]
        breaks = [k for k in range(len(turning_idx)) if gaps[k] >= min_sep]
        if breaks:
            for pos, brk in enumerate(breaks):
                first = turning_idx[(brk + 1) % len(turning_idx)]
                next_brk = breaks[(pos + 1) % len(breaks)]
                last = turning_idx[next_brk]
                regions.append((first, last))
        # no breaks -> near-circular lane, fall through to uniform fillers

    for first, last in regions:
        if first == last:
            s_pos = float(loop.project(Point(coords[first])))
            candidates.append((travel(s_pos), PRIO_CORNER))
            continue
        chord = _distance(coords[first], coords[last])
        if chord < min_sep:
            span = (last - first) % n
            apex = coords[(first + span // 2) % n]
            s_pos = float(loop.project(Point(apex)))
            candidates.append((travel(s_pos), PRIO_CORNER))
        else:
            for vertex in (coords[first], coords[last]):
                s_pos = float(loop.project(Point(vertex)))
                candidates.append((travel(s_pos), PRIO_CORNER))

    candidates.sort(key=lambda row: row[0])
    merged: list[tuple[float, int]] = []
    for p, prio in candidates:
        if merged and p - merged[-1][0] < min_sep:
            if prio < merged[-1][1]:
                merged[-1] = (p, prio)
            continue
        merged.append((p, prio))
    while len(merged) > 1 and (merged[0][0] + length) - merged[-1][0] < min_sep:
        if merged[-1][1] < merged[0][1]:
            merged.pop(0)
        else:
            merged.pop()

    filled: list[float] = []
    spacing = max(float(spacing_max_m), min_sep)
    for idx, (p, _prio) in enumerate(merged):
        filled.append(p)
        nxt = merged[idx + 1][0] if idx + 1 < len(merged) else merged[0][0] + length
        gap = nxt - p
        if gap > spacing:
            count = int(math.ceil(gap / spacing))
            count = min(count, max(1, int(gap // min_sep)))
            filled.extend(p + gap * k / count for k in range(1, count))

    nodes: list[dict[str, Any]] = []
    for p in filled:
        s = (start_s + p) % length if ccw else (start_s - p) % length
        point = loop.interpolate(s)
        nodes.append({"p": float(p), "s": float(s), "xy": (float(point.x), float(point.y))})
    return nodes


def _interpolated_xy_points(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    *,
    points: int,
) -> list[tuple[float, float]]:
    count = max(2, int(points))
    out: list[tuple[float, float]] = []
    for idx in range(count):
        ratio = float(idx) / float(count - 1)
        out.append(
            (
                float(start_xy[0]) + (float(end_xy[0]) - float(start_xy[0])) * ratio,
                float(start_xy[1]) + (float(end_xy[1]) - float(start_xy[1])) * ratio,
            )
        )
    return out


def _fill_headings_and_links(waypoints: list[Waypoint]) -> None:
    for idx, wp in enumerate(waypoints):
        if idx + 1 < len(waypoints):
            nxt = waypoints[idx + 1]
            wp.next_waypoint_id = int(nxt.waypoint_id)
            wp.heading_deg = _heading_deg((wp.x, wp.y), (nxt.x, nxt.y))
        elif len(waypoints) > 1:
            wp.heading_deg = waypoints[idx - 1].heading_deg


# ---------------------------------------------------------------------------
# Coverage strips
# ---------------------------------------------------------------------------


def _build_strips(
    mission: DonutMission,
    sweep_lines: list[SweepLine],
    *,
    coverage_width_m: float,
    end_extension_m: float = 0.0,
) -> BaseGeometry:
    """Coverage strip per sweep: width = vertical footprint across the line.

    The line is extended by end_extension_m (half the horizontal footprint:
    the first/last camera frame reaches that far beyond the line ends) and
    clipped back to the donut, which also covers the boundary bulges between
    adjacent strip end caps at convex outer corners.
    """
    pieces = []
    half_width = max(0.5, float(coverage_width_m) * 0.5)
    extension = max(0.0, float(end_extension_m))
    seen: set[tuple[float, float, float, float]] = set()
    for sweep in sweep_lines:
        key = (
            round(sweep.start_x, 1),
            round(sweep.start_y, 1),
            round(sweep.end_x, 1),
            round(sweep.end_y, 1),
        )
        sx, sy = float(sweep.start_x), float(sweep.start_y)
        ex, ey = float(sweep.end_x), float(sweep.end_y)
        length = math.hypot(ex - sx, ey - sy)
        if extension > 0.0 and length > 1e-6:
            ux, uy = (ex - sx) / length, (ey - sy) / length
            sx, sy = sx - ux * extension, sy - uy * extension
            ex, ey = ex + ux * extension, ey + uy * extension
        line = LineString([(sx, sy), (ex, ey)])
        raw = line.buffer(half_width, cap_style=2, join_style=2)
        clipped = raw.intersection(mission.polygon_xy)
        sweep.strip = clipped
        if key in seen:
            continue  # later laps repeat the same strip geometry
        seen.add(key)
        if not clipped.is_empty:
            pieces.append(clipped)
    return unary_union(pieces) if pieces else Polygon()


# ---------------------------------------------------------------------------
# Simulation interpolation
# ---------------------------------------------------------------------------


def _sweep_timeline(plan: AircraftPlan) -> list[tuple[float, float, LineSearchGroup]]:
    """(leg_start, leg_end, group) windows; the camera runs the group polyline
    across each window."""
    wps = plan.waypoints
    out: list[tuple[float, float, LineSearchGroup]] = []
    for idx, wp in enumerate(wps):
        if wp.line_search is None:
            continue
        start = float(wp.eta_s)
        end = float(wps[idx + 1].eta_s) if idx + 1 < len(wps) else start + 1e-3
        if end <= start:
            end = start + 1e-3
        out.append((start, end, wp.line_search))
    return out


def _interpolate_camera(
    timeline: list[tuple[float, float, LineSearchGroup]],
    time_s: float,
) -> dict[str, Any] | None:
    if not timeline:
        return None
    starts = [row[0] for row in timeline]
    idx = _bisect_right(starts, time_s) - 1
    if idx < 0:
        return None
    start, end, group = timeline[idx]
    if time_s > end or not group.coords_xy:
        return None
    ratio = max(0.0, min(1.0, (time_s - start) / max(end - start, 1e-6)))
    target = ratio * float(group.length_m)
    seg = _bisect_right(group.cumulative_m, target) - 1
    seg = min(max(seg, 0), len(group.coords_xy) - 2) if len(group.coords_xy) > 1 else 0
    x0, y0 = group.coords_xy[seg]
    if len(group.coords_xy) > 1:
        x1, y1 = group.coords_xy[seg + 1]
        seg_len = max(float(group.cumulative_m[seg + 1] - group.cumulative_m[seg]), 1e-9)
        frac = max(0.0, min(1.0, (target - float(group.cumulative_m[seg])) / seg_len))
        x = x0 + (x1 - x0) * frac
        y = y0 + (y1 - y0) * frac
    else:
        x, y = x0, y0
    return {
        "lineSearchGroupID": int(group.group_id),
        "x": round(float(x), 3),
        "y": round(float(y), 3),
        "progress": round(ratio, 4),
    }


def _interpolate_plan(plan: AircraftPlan, time_s: float) -> dict[str, Any] | None:
    wps = plan.waypoints
    if not wps:
        return None
    if time_s <= wps[0].eta_s:
        wp = wps[0]
        return _state_from_xy(plan.aircraft_id, wp.x, wp.y, wp.heading_deg, time_s, wp.lap)
    lo, hi = 0, len(wps) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if wps[mid].eta_s < time_s:
            lo = mid + 1
        else:
            hi = mid
    if lo == 0:
        wp = wps[0]
        return _state_from_xy(plan.aircraft_id, wp.x, wp.y, wp.heading_deg, time_s, wp.lap)
    prev, curr = wps[lo - 1], wps[lo]
    denom = max(curr.eta_s - prev.eta_s, 1e-6)
    ratio = max(0.0, min(1.0, (time_s - prev.eta_s) / denom))
    x = prev.x + (curr.x - prev.x) * ratio
    y = prev.y + (curr.y - prev.y) * ratio
    heading = _heading_deg((prev.x, prev.y), (curr.x, curr.y))
    return _state_from_xy(plan.aircraft_id, x, y, heading, time_s, curr.lap)


def _state_from_xy(
    aircraft_id: int,
    x: float,
    y: float,
    heading_deg: float,
    time_s: float,
    lap: int,
) -> dict[str, Any]:
    return {
        "aircraftID": int(aircraft_id),
        "timeS": round(float(time_s), 3),
        "x": round(float(x), 3),
        "y": round(float(y), 3),
        "headingDeg": round(float(heading_deg), 2),
        "lap": int(lap),
    }


# ---------------------------------------------------------------------------
# Small utilities
# ---------------------------------------------------------------------------


def _normalize_coord_rows(value: Any) -> list[dict[str, Any]]:
    rows = value if isinstance(value, list) else []
    out: list[dict[str, Any]] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        try:
            lat = float(row["latitude"])
            lon = float(row["longitude"])
        except Exception:
            continue
        altitude = 0
        try:
            altitude = int(round(float(row.get("altitude", 0) or 0)))
        except Exception:
            pass
        out.append({"latitude": lat, "longitude": lon, "altitude": altitude})
    return out


def _value_at(values: tuple[Any, ...], index: int, default: Any) -> Any:
    if not values:
        return default
    if index < len(values):
        return values[index]
    return values[-1]


def _resolve_aircraft_ids(cfg: PatrolConfig) -> list[int]:
    ids = [int(row) for row in (cfg.aircraft_ids or ()) if int(row) > 0]
    if ids:
        return ids[: max(0, int(cfg.aircraft_count))]
    return list(range(4, 4 + max(0, int(cfg.aircraft_count))))


def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except Exception:
        return None
    return result if math.isfinite(result) else None


def _as_longest_line(geometry: BaseGeometry) -> LineString:
    if isinstance(geometry, LineString):
        return geometry
    lines: list[LineString] = []
    geoms = getattr(geometry, "geoms", None)
    if geoms is not None:
        for geom in geoms:
            if isinstance(geom, LineString):
                lines.append(geom)
            else:
                sub = getattr(geom, "geoms", None)
                if sub is not None:
                    lines.extend(g for g in sub if isinstance(g, LineString))
    if not lines:
        raise ValueError("geometry does not contain a line")
    return max(lines, key=lambda row: row.length)


def _line_candidates(geometry: BaseGeometry) -> list[LineString]:
    if geometry.is_empty:
        return []
    if isinstance(geometry, LineString):
        return [geometry] if geometry.length > 0.5 else []
    out: list[LineString] = []
    geoms = getattr(geometry, "geoms", None)
    if geoms is None:
        return []
    for geom in geoms:
        if isinstance(geom, LineString) and geom.length > 0.5:
            out.append(geom)
        else:
            out.extend(_line_candidates(geom))
    return out


def _distance(left: tuple[float, float], right: tuple[float, float]) -> float:
    return math.hypot(float(right[0]) - float(left[0]), float(right[1]) - float(left[1]))


def _heading_deg(start: tuple[float, float], end: tuple[float, float]) -> float:
    dx = float(end[0]) - float(start[0])
    dy = float(end[1]) - float(start[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
