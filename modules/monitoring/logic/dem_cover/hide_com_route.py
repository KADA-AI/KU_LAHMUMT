
"""Terrain-certified ingress routes to native hide/communication endpoints.

The endpoint solver answers *where* a manned aircraft can remain concealed
while retaining UAV LOS.  This module answers the separate question of whether
the aircraft can reach one of those endpoints without crossing terrain or
breaking the conservative LAH motion envelope.

Safety properties
-----------------
* Direct and several left/right cubic-Bezier horizontal routes are considered.
* Every native raster cell touched by every route segment is visited with a
  supercover DDA.  Nodata is a hard failure here (unlike optical LOS, where it
  is deliberately non-blocking).
* A dense altitude envelope is propagated in both time directions.  It clears
  terrain, respects climb/descent rates, preserves the exact current altitude,
  and terminates inside the supplied endpoint altitude interval.
* Speed is propagated with acceleration/deceleration limits and local curve
  caps from both turn-rate and bank-angle constraints.
* Returned 3-D waypoints are checked again against the dense terrain profile.

``endpoint_interval_validated`` means that the chosen arrival altitude is
inside a ``HideEndpointCandidate`` interval supplied by endpoint analysis.
Native-refined candidates additionally carry a canonical LOS validator; such
routes are not selectable until their actual arrival altitude passes enemy
masking and the required UAV-link count. Legacy/test endpoints without that
explicit requirement retain the standalone terrain/dynamics behaviour.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from typing import Any, Iterable, Sequence

import numpy as np

from .hide_com import HideEndpointCandidate


_G_MPS2 = 9.80665
_EPS = 1.0e-9


@dataclass(frozen=True)
class RouteDynamics:
    """Conservative LAH limits used by the route feasibility pass.

    Heading follows the aviation convention: 0 deg is north (+Y), 90 deg is
    east (+X), increasing clockwise.
    """

    max_speed_mps: float = 73.611
    acceleration_mps2: float = 4.0
    deceleration_mps2: float = 4.0
    climb_rate_mps: float = 6.675
    descent_rate_mps: float = 5.25
    max_turn_rate_deg_s: float = 20.0
    max_bank_deg: float = 30.0
    max_altitude_m: float = 2_000.0
    clearance_m: float = 50.0

    # Bounded search/detail controls.  They are part of the public settings so
    # headless tests and future GUI controls can reproduce a result exactly.
    bezier_offset_fractions: tuple[float, ...] = (0.12, 0.25)
    curve_segment_m: float = 80.0
    max_waypoint_spacing_m: float = 250.0
    max_waypoints: int = 256
    max_endpoint_candidates: int = 16


@dataclass(frozen=True)
class RouteWaypoint:
    """One timestamped native-coordinate 3-D route waypoint."""

    x: float
    y: float
    alt_m: float
    timestamp_s: float
    speed_mps: float
    ground_m: float
    distance_m: float
    lat: float
    lon: float

    @property
    def altitude_m(self) -> float:
        return float(self.alt_m)

    @property
    def eta_s(self) -> float:
        return float(self.timestamp_s)

    def as_dict(self) -> dict[str, float]:
        return {
            "x": round(float(self.x), 3),
            "y": round(float(self.y), 3),
            "lat": round(float(self.lat), 7),
            "lon": round(float(self.lon), 7),
            "altitudeMslM": round(float(self.alt_m), 3),
            "groundM": round(float(self.ground_m), 3),
            "timestampS": round(float(self.timestamp_s), 4),
            "speedMps": round(float(self.speed_mps), 4),
            "distanceM": round(float(self.distance_m), 3),
        }


@dataclass
class RouteCandidate:
    """Evaluated route to one endpoint through one horizontal geometry."""

    endpoint_index: int
    kind: str
    lateral_offset_m: float
    endpoint: HideEndpointCandidate
    waypoints: list[RouteWaypoint] = field(default_factory=list)

    terrain_safe: bool = False
    dynamics_feasible: bool = False
    deadline_met: bool = False
    endpoint_interval_validated: bool = False
    endpoint_canonical_los_required: bool = False
    endpoint_canonical_los_validated: bool = False
    canonical_enemy_visible_count: int | None = None
    canonical_uav_link_count: int | None = None
    canonical_required_uav_links: int | None = None
    failure_codes: list[str] = field(default_factory=list)

    eta_s: float = float("inf")
    distance_m: float = 0.0
    min_clearance_m: float = float("nan")
    max_altitude_m: float = float("nan")
    max_speed_mps: float = float("nan")
    arrival_altitude_m: float = float("nan")
    initial_turn_time_s: float = 0.0
    max_turn_rate_deg_s: float = 0.0
    max_bank_deg: float = 0.0

    profile_distance_m: list[float] = field(default_factory=list, repr=False)
    profile_x: list[float] = field(default_factory=list, repr=False)

    profile_y: list[float] = field(default_factory=list, repr=False)
    profile_terrain_m: list[float] = field(default_factory=list, repr=False)
    profile_altitude_m: list[float] = field(default_factory=list, repr=False)
    profile_time_s: list[float] = field(default_factory=list, repr=False)
    profile_speed_mps: list[float] = field(default_factory=list, repr=False)

    @property
    def route_constraints_valid(self) -> bool:
        """Terrain, dynamics and analytic endpoint constraints before LOS."""

        return bool(
            self.terrain_safe
            and self.dynamics_feasible
            and self.endpoint_interval_validated
            and self.waypoints
        )

    @property
    def valid(self) -> bool:
        return bool(
            self.route_constraints_valid
            and (
                not self.endpoint_canonical_los_required
                or self.endpoint_canonical_los_validated
            )
        )

    @property
    def green_valid(self) -> bool:
        return bool(self.valid and self.deadline_met)

    @property
    def best_effort(self) -> bool:
        return bool(self.valid and not self.deadline_met)

    @property
    def status(self) -> str:
        if self.green_valid:
            return "green_valid"
        if self.best_effort:
            return "best_effort"
        return "invalid"

    @property
    def recommended_speed_mps(self) -> float:
        return float(self.max_speed_mps)

    def as_dict(self, *, include_profile: bool = False) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "endpointIndex": int(self.endpoint_index),
            "kind": self.kind,
            "lateralOffsetM": round(float(self.lateral_offset_m), 2),
            "status": self.status,
            "valid": self.valid,
            "greenValid": self.green_valid,
            "bestEffort": self.best_effort,
            "terrainSafe": bool(self.terrain_safe),
            "dynamicsFeasible": bool(self.dynamics_feasible),
            "deadlineMet": bool(self.deadline_met),
            "endpointIntervalValidated": bool(self.endpoint_interval_validated),
            "endpointCanonicalLosRequired": bool(
                self.endpoint_canonical_los_required
            ),
            "endpointCanonicalLosValidated": bool(
                self.endpoint_canonical_los_validated
            ),
            "canonicalEnemyVisibleCount": self.canonical_enemy_visible_count,
            "canonicalUavLinkCount": self.canonical_uav_link_count,
            "canonicalRequiredUavLinks": self.canonical_required_uav_links,
            "failureCodes": list(self.failure_codes),
            "etaS": None if not math.isfinite(self.eta_s) else round(self.eta_s, 4),
            "distanceM": round(float(self.distance_m), 3),
            "minClearanceM": (
                None
                if not math.isfinite(self.min_clearance_m)
                else round(self.min_clearance_m, 3)
            ),
            "maxAltitudeM": (
                None
                if not math.isfinite(self.max_altitude_m)
                else round(self.max_altitude_m, 3)
            ),
            "maxSpeedMps": (
                None
                if not math.isfinite(self.max_speed_mps)
                else round(self.max_speed_mps, 4)
            ),
            "arrivalAltitudeM": (
                None
                if not math.isfinite(self.arrival_altitude_m)
                else round(self.arrival_altitude_m, 3)
            ),
            "waypoints": [waypoint.as_dict() for waypoint in self.waypoints],
        }
        if include_profile:
            payload["profile"] = {
                "distanceM": list(self.profile_distance_m),
                "x": list(self.profile_x),
                "y": list(self.profile_y),
                "terrainM": list(self.profile_terrain_m),
                "altitudeM": list(self.profile_altitude_m),
                "timeS": list(self.profile_time_s),
                "speedMps": list(self.profile_speed_mps),
            }
        return payload


@dataclass
class HideRouteResult:
    """All evaluated routes and the selected deadline/best-effort route."""

    selected: RouteCandidate | None
    candidates: list[RouteCandidate]
    green_valid: bool
    best_effort: bool
    deadline_s: float
    failure_codes: list[str]
    planning_elapsed_s: float

    @property
    def selected_route(self) -> RouteCandidate | None:
        return self.selected

    @property
    def candidate_routes(self) -> list[RouteCandidate]:
        return self.candidates

    @property
    def waypoints(self) -> list[RouteWaypoint]:
        return [] if self.selected is None else self.selected.waypoints

    @property
    def eta_s(self) -> float:
        return float("inf") if self.selected is None else float(self.selected.eta_s)

    @property
    def recommended_speed_mps(self) -> float:
        return (
            float("nan")
            if self.selected is None

            else float(self.selected.max_speed_mps)
        )

    @property
    def terrain_safe(self) -> bool:
        return bool(self.selected is not None and self.selected.terrain_safe)

    @property
    def dynamics_feasible(self) -> bool:
        return bool(self.selected is not None and self.selected.dynamics_feasible)

    @property
    def endpoint_interval_validated(self) -> bool:
        return bool(
            self.selected is not None
            and self.selected.endpoint_interval_validated
        )

    @property
    def valid(self) -> bool:
        return bool(self.selected is not None and self.selected.valid)

    @property
    def deadline_met(self) -> bool:
        return bool(self.selected is not None and self.selected.deadline_met)

    def as_dict(self, *, include_profiles: bool = False) -> dict[str, Any]:
        return {
            "greenValid": bool(self.green_valid),
            "bestEffort": bool(self.best_effort),
            "deadlineS": round(float(self.deadline_s), 3),
            "failureCodes": list(self.failure_codes),
            "planningElapsedS": round(float(self.planning_elapsed_s), 4),
            "selected": (
                None
                if self.selected is None
                else self.selected.as_dict(include_profile=include_profiles)
            ),
            "candidates": [
                candidate.as_dict(include_profile=include_profiles)
                for candidate in self.candidates
            ],
        }


@dataclass(frozen=True)
class _TerrainTrace:
    distance_m: np.ndarray
    x: np.ndarray
    y: np.ndarray
    terrain_m: np.ndarray
    anchor_indices: tuple[int, ...]
    failure_code: str | None = None


def _finite_positive(value: float, fallback: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return float(fallback)
    return parsed if math.isfinite(parsed) and parsed > 0.0 else float(fallback)


def _wrap_delta_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def _bearing_deg(dx: float, dy: float) -> float:
    return math.degrees(math.atan2(float(dx), float(dy))) % 360.0


def _polyline_distances(points: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.zeros(0, dtype=np.float64)
    if len(points) == 1:
        return np.zeros(1, dtype=np.float64)
    legs = np.hypot(np.diff(points[:, 0]), np.diff(points[:, 1]))
    return np.concatenate((np.asarray([0.0]), np.cumsum(legs))).astype(np.float64)


def _deduplicate_points(points: Iterable[tuple[float, float]]) -> np.ndarray:
    kept: list[tuple[float, float]] = []
    for x, y in points:
        point = (float(x), float(y))
        if kept and math.hypot(point[0] - kept[-1][0], point[1] - kept[-1][1]) <= 1e-6:
            continue
        kept.append(point)
    return np.asarray(kept, dtype=np.float64)


def _turn_then_tangent_geometry(
    start: np.ndarray,
    goal: np.ndarray,
    heading_u: np.ndarray,
    *,
    radius_m: float,
    turn_side: int,
    segment_m: float,
) -> np.ndarray | None:
    """Build a constant-radius initial turn followed by a tangent straight."""

    radius = max(1.0, float(radius_m))
    side = 1 if int(turn_side) >= 0 else -1
    left_u = np.asarray((-heading_u[1], heading_u[0]), dtype=np.float64)
    centre = start + left_u * radius * float(side)
    goal_vector = goal - centre
    centre_to_goal_m = float(np.hypot(goal_vector[0], goal_vector[1]))
    if centre_to_goal_m <= radius + 1.0e-6:
        return None
    alpha = math.atan2(float(goal_vector[1]), float(goal_vector[0]))
    tangent_offset = math.acos(min(1.0, radius / centre_to_goal_m))
    start_radial = math.atan2(float(start[1] - centre[1]), float(start[0] - centre[0]))
    options: list[tuple[float, float, np.ndarray]] = []
    for tangent_angle in (alpha - tangent_offset, alpha + tangent_offset):
        tangent = centre + radius * np.asarray(
            (math.cos(tangent_angle), math.sin(tangent_angle)), dtype=np.float64
        )
        line = goal - tangent
        line_length = float(np.hypot(line[0], line[1]))
        if line_length <= 1.0e-6:
            continue
        line_u = line / line_length
        tangent_u = (
            np.asarray((-math.sin(tangent_angle), math.cos(tangent_angle)))
            if side > 0
            else np.asarray((math.sin(tangent_angle), -math.cos(tangent_angle)))
        )
        alignment = float(np.dot(tangent_u, line_u))
        if alignment < 0.995:
            continue
        arc_angle = (
            (tangent_angle - start_radial) % (2.0 * math.pi)
            if side > 0
            else (start_radial - tangent_angle) % (2.0 * math.pi)
        )
        options.append((arc_angle, line_length, tangent))
    if not options:
        return None
    arc_angle, line_length, tangent = min(
        options,
        key=lambda item: (item[0] * radius + item[1], item[0]),
    )
    arc_count = max(2, int(math.ceil((arc_angle * radius) / max(10.0, segment_m))))
    line_count = max(1, int(math.ceil(line_length / max(10.0, segment_m))))
    points: list[tuple[float, float]] = []
    for index in range(arc_count + 1):
        fraction = index / arc_count
        angle = start_radial + float(side) * arc_angle * fraction
        point = centre + radius * np.asarray((math.cos(angle), math.sin(angle)))
        points.append((float(point[0]), float(point[1])))
    for index in range(1, line_count + 1):
        fraction = index / line_count
        point = tangent + (goal - tangent) * fraction
        points.append((float(point[0]), float(point[1])))
    result = _deduplicate_points(points)
    return result if len(result) >= 2 else None


def _horizontal_geometries(
    start: tuple[float, float],
    goal: tuple[float, float],
    heading_deg: float,
    dynamics: RouteDynamics,
    current_speed_mps: float = 0.0,
) -> list[tuple[str, float, np.ndarray]]:
    """Return direct plus heading-tangent left/right cubic Bezier routes."""

    p0 = np.asarray(start, dtype=np.float64)
    p3 = np.asarray(goal, dtype=np.float64)
    delta = p3 - p0
    direct_m = float(np.hypot(delta[0], delta[1]))
    if direct_m <= 1e-6:
        return [("direct", 0.0, np.asarray((p0, p3), dtype=np.float64))]

    u = delta / direct_m
    left = np.asarray((-u[1], u[0]), dtype=np.float64)
    heading_rad = math.radians(float(heading_deg) % 360.0)
    heading_u = np.asarray((math.sin(heading_rad), math.cos(heading_rad)))
    segment_m = max(10.0, float(dynamics.curve_segment_m))
    direct_count = max(1, int(math.ceil(direct_m / segment_m)))
    direct_points = [
        tuple(p0 + delta * (index / direct_count))
        for index in range(direct_count + 1)
    ]
    geometries: list[tuple[str, float, np.ndarray]] = [
        ("direct", 0.0, _deduplicate_points(direct_points))
    ]

    speed = max(0.0, min(float(current_speed_mps), float(dynamics.max_speed_mps)))
    max_turn_rad_s = math.radians(
        _finite_positive(dynamics.max_turn_rate_deg_s, 20.0)
    )
    bank_limit_rad = math.radians(
        min(89.0, _finite_positive(dynamics.max_bank_deg, 30.0))
    )
    turn_radius_m = max(
        20.0,
        speed / max(max_turn_rad_s, 1.0e-6),
        speed * speed / max(_G_MPS2 * math.tan(bank_limit_rad), 1.0e-6),
    )
    # These paths let an already-moving aircraft keep its current tangent and
    # execute a physically bounded turn before the straight intercept. They
    # specifically cover targets behind the aircraft, where a short cubic can
    # require impossible instantaneous deceleration.
    for radius_rank, radius_scale in enumerate((1.25, 1.75), start=1):
        radius = turn_radius_m * radius_scale
        for side, side_name in ((1, "left"), (-1, "right")):
            points = _turn_then_tangent_geometry(
                p0,
                p3,
                heading_u,
                radius_m=radius,
                turn_side=side,
                segment_m=segment_m,
            )
            if points is not None:
                geometries.append(
                    (f"arc_{side_name}_{radius_rank}", float(side) * radius, points)
                )

    handle_m = min(max(30.0, direct_m * 0.28), 500.0, direct_m * 0.48)
    offsets: list[tuple[str, float]] = [("bezier_heading", 0.0)]
    for rank, fraction in enumerate(dynamics.bezier_offset_fractions, start=1):
        offset_m = min(750.0, max(15.0, direct_m * abs(float(fraction))))
        offsets.extend(
            (
                (f"bezier_left_{rank}", offset_m),
                (f"bezier_right_{rank}", -offset_m),
            )
        )

    for kind, offset_m in offsets:
        p1 = p0 + heading_u * handle_m
        p2 = p3 - u * handle_m + left * float(offset_m)
        control_length = (
            float(np.linalg.norm(p1 - p0))
            + float(np.linalg.norm(p2 - p1))
            + float(np.linalg.norm(p3 - p2))
        )
        count = max(2, min(4096, int(math.ceil(control_length / segment_m))))
        samples: list[tuple[float, float]] = []

        for index in range(count + 1):
            t = index / count
            q = (
                (1.0 - t) ** 3 * p0
                + 3.0 * (1.0 - t) ** 2 * t * p1
                + 3.0 * (1.0 - t) * t * t * p2
                + t**3 * p3
            )
            samples.append((float(q[0]), float(q[1])))
        points = _deduplicate_points(samples)
        if len(points) >= 2:
            geometries.append((kind, float(offset_m), points))
    return geometries


def _segment_supercover(
    dem: Any,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
) -> list[tuple[int, int, float, float]]:
    """Return every raster cell touched by a segment with parametric intervals.

    The affine inverse makes the traversal valid for north-up and rotated
    rasters.  Zero-length side-cell intervals at exact corner crossings are
    intentional: a conservative supercover treats a terrain spike touching
    the centreline at one point as relevant.
    """

    c0, r0 = dem.inverse_transform * (float(x0), float(y0))
    c1, r1 = dem.inverse_transform * (float(x1), float(y1))
    dc = float(c1 - c0)
    dr = float(r1 - r0)
    if abs(dc) <= _EPS and abs(dr) <= _EPS:
        return [(int(math.floor(r0)), int(math.floor(c0)), 0.0, 1.0)]

    col = int(math.floor(c0))
    row = int(math.floor(r0))
    step_c = 1 if dc > 0.0 else (-1 if dc < 0.0 else 0)
    step_r = 1 if dr > 0.0 else (-1 if dr < 0.0 else 0)
    if step_c:
        next_c = float(col + 1 if step_c > 0 else col)
        t_max_c = (next_c - float(c0)) / dc
        t_delta_c = abs(1.0 / dc)
    else:
        t_max_c = float("inf")
        t_delta_c = float("inf")
    if step_r:
        next_r = float(row + 1 if step_r > 0 else row)
        t_max_r = (next_r - float(r0)) / dr
        t_delta_r = abs(1.0 / dr)
    else:
        t_max_r = float("inf")
        t_delta_r = float("inf")

    vertical_boundary = step_c == 0 and abs(c0 - round(c0)) <= 1e-10
    horizontal_boundary = step_r == 0 and abs(r0 - round(r0)) <= 1e-10
    cells: list[tuple[int, int, float, float]] = []
    seen: set[tuple[int, int, int, int]] = set()

    def add(rr: int, cc: int, enter: float, leave: float) -> None:
        enter = float(np.clip(enter, 0.0, 1.0))
        leave = float(np.clip(leave, enter, 1.0))
        key = (int(rr), int(cc), int(round(enter * 1e12)), int(round(leave * 1e12)))
        if key not in seen:
            seen.add(key)
            cells.append((int(rr), int(cc), enter, leave))

    current_t = 0.0
    max_steps = int(abs(math.floor(c1) - col) + abs(math.floor(r1) - row) + 16)
    for _ in range(max(16, max_steps * 3)):
        next_t = min(1.0, t_max_c, t_max_r)
        add(row, col, current_t, next_t)
        if vertical_boundary:
            add(row, col - 1, current_t, next_t)
        if horizontal_boundary:
            add(row - 1, col, current_t, next_t)
        if next_t >= 1.0 - 1e-12:
            break

        cross_c = t_max_c <= next_t + 1e-12
        cross_r = t_max_r <= next_t + 1e-12
        if cross_c and cross_r:
            # At a corner, include both side cells as zero-width contacts.
            add(row, col + step_c, next_t, next_t)
            add(row + step_r, col, next_t, next_t)
        if cross_c:
            col += step_c
            t_max_c += t_delta_c
        if cross_r:
            row += step_r
            t_max_r += t_delta_r
        if not cross_c and not cross_r:  # numerical guard
            break
        current_t = next_t
    else:
        raise RuntimeError("Raster supercover traversal exceeded its bounded step count.")
    return cells


def _trace_terrain(dem: Any, points: np.ndarray) -> _TerrainTrace:
    """Build a compact cell-boundary terrain profile for one horizontal route."""

    cumulative = _polyline_distances(points)
    if len(points) < 2 or cumulative[-1] <= 1e-6:
        return _TerrainTrace(
            np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), (), "ZERO_LENGTH_ROUTE"
        )
    raw: list[tuple[float, float, float, float]] = []
    anchor_distances = [float(value) for value in cumulative]
    for index, (left, right) in enumerate(zip(points, points[1:])):
        x0, y0 = map(float, left)
        x1, y1 = map(float, right)
        if not dem.contains_native(x0, y0) or not dem.contains_native(x1, y1):
            return _TerrainTrace(
                np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), (), "OUTSIDE_DEM"
            )
        leg_m = float(cumulative[index + 1] - cumulative[index])
        for row, col, enter, leave in _segment_supercover(dem, x0, y0, x1, y1):
            if row < 0 or col < 0 or row >= int(dem.height) or col >= int(dem.width):
                return _TerrainTrace(
                    np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), (), "OUTSIDE_DEM"
                )
            ground_m = float(dem.block_elev[row, col])
            if not math.isfinite(ground_m) or ground_m < -1000.0:
                return _TerrainTrace(
                    np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), (), "NODATA_CROSSED"
                )
            for fraction in (enter, leave):
                raw.append(
                    (
                        float(cumulative[index]) + float(fraction) * leg_m,
                        x0 + (x1 - x0) * float(fraction),
                        y0 + (y1 - y0) * float(fraction),
                        ground_m,
                    )
                )

    raw.sort(key=lambda item: item[0])

    merged: list[list[float]] = []
    for distance_m, x, y, ground_m in raw:
        if merged and abs(distance_m - merged[-1][0]) <= 1e-7:
            if ground_m > merged[-1][3]:
                merged[-1][1] = x
                merged[-1][2] = y
                merged[-1][3] = ground_m
        else:
            merged.append([distance_m, x, y, ground_m])
    if len(merged) < 2:
        return _TerrainTrace(
            np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0), (), "EMPTY_TERRAIN_PROFILE"
        )
    distances = np.asarray([item[0] for item in merged], dtype=np.float64)
    xs = np.asarray([item[1] for item in merged], dtype=np.float64)
    ys = np.asarray([item[2] for item in merged], dtype=np.float64)
    terrain = np.asarray([item[3] for item in merged], dtype=np.float64)
    anchors = tuple(
        sorted(
            {
                int(np.argmin(np.abs(distances - anchor_m)))
                for anchor_m in anchor_distances
            }
        )
    )
    return _TerrainTrace(distances, xs, ys, terrain, anchors, None)


def _speed_and_time_profile(
    points: np.ndarray,
    current_speed_mps: float,
    heading_deg: float,
    dynamics: RouteDynamics,
) -> tuple[np.ndarray, np.ndarray, float, float, float, str | None]:
    """Minimum-time speed profile under curve, acceleration and turn limits."""

    distances = _polyline_distances(points)
    if len(points) < 2 or distances[-1] <= 1e-6:
        return np.zeros(0), np.zeros(0), 0.0, 0.0, 0.0, "ZERO_LENGTH_ROUTE"
    speed0 = float(current_speed_mps)
    max_speed = _finite_positive(dynamics.max_speed_mps, 73.611)
    if not math.isfinite(speed0) or speed0 < 0.0:
        return np.zeros(0), np.zeros(0), 0.0, 0.0, 0.0, "INVALID_CURRENT_SPEED"
    if speed0 > max_speed + 1e-6:
        return np.zeros(0), np.zeros(0), 0.0, 0.0, 0.0, "CURRENT_SPEED_ABOVE_LIMIT"

    legs = np.diff(points, axis=0)
    leg_lengths = np.hypot(legs[:, 0], legs[:, 1])
    headings = np.asarray(
        [_bearing_deg(float(dx), float(dy)) for dx, dy in legs], dtype=np.float64
    )
    caps = np.full(len(points), max_speed, dtype=np.float64)
    curvatures = np.zeros(len(points), dtype=np.float64)
    max_turn_rad_s = math.radians(_finite_positive(dynamics.max_turn_rate_deg_s, 20.0))
    bank_limit_rad = math.radians(
        min(89.0, _finite_positive(dynamics.max_bank_deg, 30.0))
    )

    # Initial heading change is distributed conservatively over the first
    # sampled curve segment.  A stopped helicopter may yaw in place first.
    initial_delta_deg = abs(_wrap_delta_deg(float(headings[0]) - float(heading_deg)))
    initial_turn_time_s = 0.0
    if speed0 <= 0.5 and initial_delta_deg > 1e-6:
        initial_turn_time_s = initial_delta_deg / math.degrees(max_turn_rad_s)
        initial_delta_deg = 0.0

    for index in range(len(points) - 1):
        if index == 0:
            angle_rad = math.radians(initial_delta_deg)
            span_m = max(1.0, float(leg_lengths[0]))
        else:
            angle_rad = math.radians(
                abs(_wrap_delta_deg(float(headings[index]) - float(headings[index - 1])))
            )
            span_m = max(
                1.0,
                0.5 * (float(leg_lengths[index - 1]) + float(leg_lengths[index])),
            )
        curvature = angle_rad / span_m
        if curvature <= 1e-12:
            continue
        curvatures[index] = curvature
        turn_cap = max_turn_rad_s / curvature
        bank_cap = math.sqrt(max(0.0, _G_MPS2 * math.tan(bank_limit_rad) / curvature))
        caps[index] = min(caps[index], turn_cap, bank_cap)

    if speed0 > caps[0] + 1e-6:
        return (
            np.zeros(0),
            np.zeros(0),
            initial_turn_time_s,
            0.0,
            0.0,
            "INITIAL_SPEED_EXCEEDS_TURN_LIMIT",
        )

    acceleration = _finite_positive(dynamics.acceleration_mps2, 4.0)
    deceleration = _finite_positive(dynamics.deceleration_mps2, 4.0)
    speeds = np.zeros(len(points), dtype=np.float64)
    speeds[0] = speed0
    for index in range(1, len(points)):
        ds = float(distances[index] - distances[index - 1])
        reachable = math.sqrt(max(0.0, speeds[index - 1] ** 2 + 2.0 * acceleration * ds))
        speeds[index] = min(float(caps[index]), max_speed, reachable)
    for index in range(len(points) - 2, -1, -1):
        ds = float(distances[index + 1] - distances[index])
        allowed = math.sqrt(max(0.0, speeds[index + 1] ** 2 + 2.0 * deceleration * ds))
        speeds[index] = min(speeds[index], allowed)
    if speeds[0] < speed0 - 1e-6:
        return (
            np.zeros(0),
            np.zeros(0),
            initial_turn_time_s,
            0.0,
            0.0,
            "INSUFFICIENT_DISTANCE_TO_DECELERATE",
        )
    speeds[0] = speed0

    for index in range(len(points) - 1):
        ds = max(1e-9, float(distances[index + 1] - distances[index]))
        acceleration_value = (
            float(speeds[index + 1] ** 2 - speeds[index] ** 2) / (2.0 * ds)
        )
        if acceleration_value > acceleration + 1e-6:
            return speeds, np.zeros(0), initial_turn_time_s, 0.0, 0.0, "ACCELERATION_LIMIT"
        if -acceleration_value > deceleration + 1e-6:
            return speeds, np.zeros(0), initial_turn_time_s, 0.0, 0.0, "DECELERATION_LIMIT"

    timestamps = np.zeros(len(points), dtype=np.float64)
    for index in range(len(points) - 1):
        ds = float(distances[index + 1] - distances[index])
        denom = float(speeds[index] + speeds[index + 1])
        if denom <= 1e-9:
            return (
                np.zeros(0),
                np.zeros(0),
                initial_turn_time_s,
                0.0,
                0.0,

                "ZERO_SPEED_SEGMENT",
            )
        timestamps[index + 1] = timestamps[index] + 2.0 * ds / denom
    turn_rates = np.degrees(speeds * curvatures)
    bank_angles = np.degrees(
        np.arctan2(speeds * speeds * curvatures, np.float64(_G_MPS2))
    )
    max_turn = float(np.max(turn_rates)) if turn_rates.size else 0.0
    max_bank = float(np.max(bank_angles)) if bank_angles.size else 0.0
    if max_turn > float(dynamics.max_turn_rate_deg_s) + 1e-5:
        return speeds, timestamps, initial_turn_time_s, max_turn, max_bank, "TURN_RATE_LIMIT"
    if max_bank > float(dynamics.max_bank_deg) + 1e-5:
        return speeds, timestamps, initial_turn_time_s, max_turn, max_bank, "BANK_LIMIT"
    return speeds, timestamps, initial_turn_time_s, max_turn, max_bank, None


def _interpolate_kinematics(
    query_distance_m: np.ndarray,
    node_distance_m: np.ndarray,
    node_speed_mps: np.ndarray,
    node_time_s: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Evaluate exact constant-acceleration time/speed inside every route leg."""

    query = np.asarray(query_distance_m, dtype=np.float64)
    result_time = np.zeros(query.shape, dtype=np.float64)
    result_speed = np.zeros(query.shape, dtype=np.float64)
    if query.size == 0:
        return result_time, result_speed
    leg_indices = np.searchsorted(node_distance_m, query, side="right") - 1
    leg_indices = np.clip(leg_indices, 0, len(node_distance_m) - 2)
    for output_index, leg_index_value in enumerate(leg_indices):
        leg_index = int(leg_index_value)
        leg_start_m = float(node_distance_m[leg_index])
        leg_length_m = max(
            1e-12,
            float(node_distance_m[leg_index + 1] - node_distance_m[leg_index]),
        )
        partial_m = float(np.clip(query[output_index] - leg_start_m, 0.0, leg_length_m))
        v0 = float(node_speed_mps[leg_index])
        v1 = float(node_speed_mps[leg_index + 1])
        acceleration = (v1 * v1 - v0 * v0) / (2.0 * leg_length_m)
        speed = math.sqrt(max(0.0, v0 * v0 + 2.0 * acceleration * partial_m))
        denominator = v0 + speed
        if denominator <= 1e-12:
            partial_time_s = 0.0
        else:
            # 2*s/(v0+v) is stable for both acceleration and deceleration.
            partial_time_s = 2.0 * partial_m / denominator
        result_time[output_index] = float(node_time_s[leg_index]) + partial_time_s
        result_speed[output_index] = speed
    final = query >= float(node_distance_m[-1]) - 1e-9
    result_time[final] = float(node_time_s[-1])
    result_speed[final] = float(node_speed_mps[-1])
    return result_time, result_speed


def _propagate_altitudes(
    terrain_m: np.ndarray,
    timestamps_s: np.ndarray,
    own_alt_m: float,
    endpoint: HideEndpointCandidate,
    dynamics: RouteDynamics,
) -> tuple[np.ndarray, float, str | None]:
    """Solve the minimum asymmetric-rate altitude envelope and arrival altitude."""

    if terrain_m.size < 2 or timestamps_s.size != terrain_m.size:
        return np.zeros(0), float("nan"), "INVALID_VERTICAL_PROFILE"
    clearance = _finite_positive(dynamics.clearance_m, 50.0)
    floor_m = terrain_m.astype(np.float64, copy=True) + clearance
    own_alt = float(own_alt_m)
    if not math.isfinite(own_alt):
        return np.zeros(0), float("nan"), "INVALID_OWN_ALTITUDE"
    if own_alt < float(floor_m[0]) - 1e-6:
        return np.zeros(0), float("nan"), "START_BELOW_CLEARANCE"

    endpoint_low = max(float(endpoint.min_altitude_m), float(floor_m[-1]))
    endpoint_high = min(float(endpoint.max_altitude_m), float(dynamics.max_altitude_m))
    if not (math.isfinite(endpoint_low) and math.isfinite(endpoint_high)):
        return np.zeros(0), float("nan"), "INVALID_ENDPOINT_INTERVAL"
    if endpoint_low > endpoint_high + 1e-6:
        return np.zeros(0), float("nan"), "ENDPOINT_INTERVAL_BELOW_CLEARANCE"

    climb = _finite_positive(dynamics.climb_rate_mps, 6.675)
    descent = _finite_positive(dynamics.descent_rate_mps, 5.25)

    def closure(base: np.ndarray) -> np.ndarray:
        past = base.copy()
        for index in range(1, len(base)):
            dt = max(0.0, float(timestamps_s[index] - timestamps_s[index - 1]))
            past[index] = max(past[index], past[index - 1] - descent * dt)
        future = base.copy()
        for index in range(len(base) - 2, -1, -1):
            dt = max(0.0, float(timestamps_s[index + 1] - timestamps_s[index]))
            future[index] = max(future[index], future[index + 1] - climb * dt)
        return np.maximum(past, future)

    base = floor_m.copy()
    base[0] = max(base[0], own_alt)
    required = closure(base)
    if required[0] > own_alt + 1e-6:
        return np.zeros(0), float("nan"), "CLIMB_RATE_UNREACHABLE"
    total_time_s = max(0.0, float(timestamps_s[-1] - timestamps_s[0]))
    required_goal = max(
        endpoint_low,
        float(required[-1]),
        own_alt - descent * total_time_s,
    )
    reachable_goal_high = min(
        endpoint_high,
        own_alt + climb * total_time_s,
    )
    if required_goal > reachable_goal_high + 1e-6:
        return np.zeros(0), float("nan"), "ENDPOINT_ALTITUDE_INTERVAL_UNREACHABLE"
    preferred = float(endpoint.preferred_altitude_m)
    if not math.isfinite(preferred):
        preferred = required_goal
    arrival_alt = float(np.clip(preferred, required_goal, reachable_goal_high))

    base[0] = own_alt
    base[-1] = max(base[-1], arrival_alt)
    altitude_m = closure(base)
    if altitude_m[0] > own_alt + 1e-6:
        return np.zeros(0), float("nan"), "CLIMB_RATE_UNREACHABLE"
    altitude_m[0] = own_alt
    altitude_m[-1] = arrival_alt
    if float(np.max(altitude_m)) > float(dynamics.max_altitude_m) + 1e-6:
        return np.zeros(0), float("nan"), "MAX_ALTITUDE_EXCEEDED"

    for index in range(len(altitude_m) - 1):
        dt = max(0.0, float(timestamps_s[index + 1] - timestamps_s[index]))
        delta = float(altitude_m[index + 1] - altitude_m[index])
        if delta > climb * dt + 1e-5:
            return np.zeros(0), float("nan"), "CLIMB_RATE_LIMIT"
        if -delta > descent * dt + 1e-5:
            return np.zeros(0), float("nan"), "DESCENT_RATE_LIMIT"
    return altitude_m, arrival_alt, None


def _select_waypoint_indices(

    trace: _TerrainTrace,
    altitude_m: np.ndarray,
    dynamics: RouteDynamics,
) -> list[int]:
    """Simplify the dense safe envelope without letting a 3-D chord penetrate it."""

    selected = set(int(index) for index in trace.anchor_indices)
    selected.update((0, len(trace.distance_m) - 1))
    pending = [
        (left, right)
        for left, right in zip(sorted(selected), sorted(selected)[1:])
    ]
    clearance = float(dynamics.clearance_m)
    max_spacing = max(10.0, float(dynamics.max_waypoint_spacing_m))
    while pending:
        left, right = pending.pop()
        if right - left <= 1:
            continue
        span_m = float(trace.distance_m[right] - trace.distance_m[left])
        if span_m <= 1e-9:
            continue
        interior = np.arange(left + 1, right, dtype=np.int32)
        fractions = (
            trace.distance_m[interior] - float(trace.distance_m[left])
        ) / span_m
        chord = float(altitude_m[left]) + fractions * (
            float(altitude_m[right]) - float(altitude_m[left])
        )
        required = trace.terrain_m[interior] + clearance
        deficits = required - chord
        split: int | None = None
        if deficits.size and float(np.max(deficits)) > 1e-7:
            split = int(interior[int(np.argmax(deficits))])
        elif span_m > max_spacing + 1e-6:
            midpoint = 0.5 * (
                float(trace.distance_m[left]) + float(trace.distance_m[right])
            )
            split = int(
                interior[int(np.argmin(np.abs(trace.distance_m[interior] - midpoint)))]
            )
        if split is None or split <= left or split >= right:
            continue
        selected.add(split)
        pending.extend(((left, split), (split, right)))
    return sorted(selected)


def validate_route_waypoints(
    dem: Any,
    waypoints: Sequence[RouteWaypoint],
    dynamics: RouteDynamics | None = None,
) -> dict[str, Any]:
    """Independently validate every emitted 3-D waypoint chord on native DEM.

    This deliberately rebuilds each segment's supercover instead of trusting
    the profile used to create the waypoints.  It is therefore suitable as the
    final serialization/integration guard as well as a focused regression API.
    """

    settings = dynamics or RouteDynamics()
    failures: list[str] = []
    minimum_clearance_m = float("inf")
    maximum_climb_mps = 0.0
    maximum_descent_mps = 0.0
    if not waypoints:
        return {
            "valid": False,
            "failure_codes": ["NO_WAYPOINTS"],
            "min_clearance_m": float("nan"),
            "max_climb_rate_mps": 0.0,
            "max_descent_rate_mps": 0.0,
        }

    def add_failure(code: str) -> None:
        if code not in failures:
            failures.append(code)

    if len(waypoints) == 1:
        waypoint = waypoints[0]
        if not dem.contains_native(float(waypoint.x), float(waypoint.y)):
            add_failure("OUTSIDE_DEM")
        else:
            row, col = dem.nearest_index(float(waypoint.x), float(waypoint.y))
            terrain_m = float(dem.block_elev[int(row), int(col)])
            if not math.isfinite(terrain_m) or terrain_m < -1000.0:
                add_failure("NODATA_CROSSED")
            else:
                minimum_clearance_m = float(waypoint.alt_m) - terrain_m

    for left, right in zip(waypoints, waypoints[1:]):
        x0, y0, z0 = float(left.x), float(left.y), float(left.alt_m)
        x1, y1, z1 = float(right.x), float(right.y), float(right.alt_m)
        dt = float(right.timestamp_s) - float(left.timestamp_s)
        if dt < -1e-9:
            add_failure("NON_MONOTONIC_TIME")
            continue
        if max(z0, z1) > float(settings.max_altitude_m) + 1e-6:
            add_failure("MAX_ALTITUDE_EXCEEDED")
        delta_z = z1 - z0
        if dt <= 1e-12:
            if abs(delta_z) > 1e-7:
                add_failure("ZERO_TIME_VERTICAL_CHANGE")
        elif delta_z >= 0.0:
            maximum_climb_mps = max(maximum_climb_mps, delta_z / dt)
            if delta_z / dt > float(settings.climb_rate_mps) + 1e-6:
                add_failure("CLIMB_RATE_LIMIT")
        else:
            maximum_descent_mps = max(maximum_descent_mps, -delta_z / dt)
            if -delta_z / dt > float(settings.descent_rate_mps) + 1e-6:
                add_failure("DESCENT_RATE_LIMIT")

        leg_m = math.hypot(x1 - x0, y1 - y0)
        if leg_m <= 1e-9:
            if not dem.contains_native(x0, y0):
                add_failure("OUTSIDE_DEM")
                continue
            row, col = dem.nearest_index(x0, y0)
            terrain_m = float(dem.block_elev[int(row), int(col)])
            if not math.isfinite(terrain_m) or terrain_m < -1000.0:
                add_failure("NODATA_CROSSED")
                continue
            minimum_clearance_m = min(
                minimum_clearance_m, min(z0, z1) - terrain_m
            )
            continue

        trace = _trace_terrain(
            dem,
            np.asarray(((x0, y0), (x1, y1)), dtype=np.float64),
        )
        if trace.failure_code is not None:
            add_failure(trace.failure_code)
            continue
        fractions = np.clip(trace.distance_m / leg_m, 0.0, 1.0)
        chord_altitude_m = z0 + fractions * (z1 - z0)
        clearances_m = chord_altitude_m - trace.terrain_m
        minimum_clearance_m = min(
            minimum_clearance_m, float(np.min(clearances_m))
        )
        if np.any(clearances_m < float(settings.clearance_m) - 1e-6):

            add_failure("TERRAIN_CLEARANCE_VIOLATION")

    if float(waypoints[0].alt_m) > float(settings.max_altitude_m) + 1e-6:
        add_failure("MAX_ALTITUDE_EXCEEDED")
    if minimum_clearance_m < float(settings.clearance_m) - 1e-6:
        add_failure("TERRAIN_CLEARANCE_VIOLATION")
    if not math.isfinite(minimum_clearance_m):
        minimum_clearance_m = float("nan")
    return {
        "valid": not failures,
        "failure_codes": failures,
        "min_clearance_m": minimum_clearance_m,
        "max_climb_rate_mps": maximum_climb_mps,
        "max_descent_rate_mps": maximum_descent_mps,
    }


def _validate_endpoint_canonical_los(candidate: RouteCandidate) -> None:
    """Apply a native endpoint's canonical LOS hook at the arrival altitude."""

    required = bool(
        getattr(candidate.endpoint, "requires_canonical_los_validation", False)
    )
    candidate.endpoint_canonical_los_required = required
    if not required:
        return

    validator = getattr(candidate.endpoint, "canonical_los_validator", None)
    try:
        required_links = max(
            1,
            int(getattr(candidate.endpoint, "required_uav_links", 0)),
        )
    except (TypeError, ValueError, OverflowError):
        required_links = 1
    candidate.canonical_required_uav_links = required_links
    if not callable(validator):
        candidate.failure_codes.append("ENDPOINT_CANONICAL_LOS_VALIDATOR_MISSING")
        return

    try:
        status = validator(float(candidate.arrival_altitude_m))
        enemy_visible = int(status[0])
        uav_links = int(status[3])
        if enemy_visible < 0 or uav_links < 0:
            raise ValueError("negative canonical LOS count")
    except Exception:  # noqa: BLE001 - candidate must fail closed at integration edge
        candidate.failure_codes.append("ENDPOINT_CANONICAL_LOS_VALIDATION_ERROR")
        return

    candidate.canonical_enemy_visible_count = enemy_visible
    candidate.canonical_uav_link_count = uav_links
    if enemy_visible != 0:
        candidate.failure_codes.append("ENDPOINT_CANONICAL_ENEMY_LOS_FAILED")
    if uav_links < required_links:
        candidate.failure_codes.append("ENDPOINT_CANONICAL_UAV_LOS_FAILED")
    candidate.endpoint_canonical_los_validated = bool(
        enemy_visible == 0 and uav_links >= required_links
    )


def _evaluate_geometry(
    dem: Any,
    *,
    own_alt_m: float,
    current_speed_mps: float,
    heading_deg: float,
    endpoint: HideEndpointCandidate,
    endpoint_index: int,
    kind: str,
    lateral_offset_m: float,
    points: np.ndarray,
    deadline_s: float,
    dynamics: RouteDynamics,
) -> RouteCandidate:
    candidate = RouteCandidate(
        endpoint_index=int(endpoint_index),
        kind=str(kind),
        lateral_offset_m=float(lateral_offset_m),
        endpoint=endpoint,
        endpoint_canonical_los_required=bool(
            getattr(endpoint, "requires_canonical_los_validation", False)
        ),
    )
    if kind == "direct" and float(current_speed_mps) > 0.5 and len(points) >= 2:
        first_dx = float(points[1, 0] - points[0, 0])
        first_dy = float(points[1, 1] - points[0, 1])
        mismatch_deg = abs(
            _wrap_delta_deg(_bearing_deg(first_dx, first_dy) - float(heading_deg))
        )
        if mismatch_deg > 1.0:
            candidate.failure_codes.append("MOVING_DIRECT_HEADING_MISMATCH")
            return candidate
    trace = _trace_terrain(dem, points)
    candidate.distance_m = (
        float(trace.distance_m[-1]) if trace.distance_m.size else 0.0
    )
    if trace.failure_code is not None:
        candidate.failure_codes.append(trace.failure_code)
        return candidate

    (
        route_speeds,
        route_times,
        initial_turn_time_s,
        max_turn_rate,
        max_bank,
        speed_failure,
    ) = _speed_and_time_profile(
        points,
        current_speed_mps,
        heading_deg,
        dynamics,
    )
    candidate.initial_turn_time_s = float(initial_turn_time_s)
    candidate.max_turn_rate_deg_s = float(max_turn_rate)
    candidate.max_bank_deg = float(max_bank)
    if speed_failure is not None:
        candidate.failure_codes.append(speed_failure)
        return candidate

    route_distances = _polyline_distances(points)
    dense_motion_time, dense_speed = _interpolate_kinematics(
        trace.distance_m,
        route_distances,
        route_speeds,
        route_times,
    )
    altitude_m, arrival_alt, altitude_failure = _propagate_altitudes(
        trace.terrain_m,
        dense_motion_time,
        own_alt_m,
        endpoint,
        dynamics,
    )
    if altitude_failure is not None:
        candidate.failure_codes.append(altitude_failure)
        return candidate

    candidate.terrain_safe = bool(

        np.all(altitude_m + 1e-7 >= trace.terrain_m + float(dynamics.clearance_m))
    )
    candidate.dynamics_feasible = True
    candidate.endpoint_interval_validated = bool(
        float(endpoint.min_altitude_m) - 1e-6
        <= float(arrival_alt)
        <= float(endpoint.max_altitude_m) + 1e-6
    )
    if not candidate.terrain_safe:
        candidate.failure_codes.append("TERRAIN_CLEARANCE_VIOLATION")
    if not candidate.endpoint_interval_validated:
        candidate.failure_codes.append("ENDPOINT_INTERVAL_VIOLATION")

    dense_time = dense_motion_time + float(initial_turn_time_s)
    candidate.eta_s = float(dense_time[-1])
    candidate.deadline_met = candidate.eta_s <= float(deadline_s) + 1e-9
    if not candidate.deadline_met:
        candidate.failure_codes.append("DEADLINE_EXCEEDED")
    candidate.min_clearance_m = float(np.min(altitude_m - trace.terrain_m))
    candidate.max_altitude_m = float(np.max(altitude_m))
    candidate.max_speed_mps = float(np.max(route_speeds))
    candidate.arrival_altitude_m = float(arrival_alt)

    candidate.profile_distance_m = trace.distance_m.astype(float).tolist()
    candidate.profile_x = trace.x.astype(float).tolist()
    candidate.profile_y = trace.y.astype(float).tolist()
    candidate.profile_terrain_m = trace.terrain_m.astype(float).tolist()
    candidate.profile_altitude_m = altitude_m.astype(float).tolist()
    candidate.profile_time_s = dense_time.astype(float).tolist()
    candidate.profile_speed_mps = dense_speed.astype(float).tolist()

    selected_indices = _select_waypoint_indices(trace, altitude_m, dynamics)
    if len(selected_indices) > max(2, int(dynamics.max_waypoints)):
        candidate.terrain_safe = False
        candidate.dynamics_feasible = False
        candidate.failure_codes.append("WAYPOINT_LIMIT_EXCEEDED")
        return candidate

    waypoints: list[RouteWaypoint] = []
    # Preserve the actual current state at t=0.  When stopped, a duplicate XY
    # waypoint records the bounded in-place yaw before translation begins.
    start_lat, start_lon = dem.native_to_latlon(float(trace.x[0]), float(trace.y[0]))
    if initial_turn_time_s > 1e-9:
        waypoints.append(
            RouteWaypoint(
                x=float(trace.x[0]),
                y=float(trace.y[0]),
                alt_m=float(altitude_m[0]),
                timestamp_s=0.0,
                speed_mps=float(current_speed_mps),
                ground_m=float(trace.terrain_m[0]),
                distance_m=0.0,
                lat=float(start_lat),
                lon=float(start_lon),
            )
        )
    for index in selected_indices:
        lat, lon = dem.native_to_latlon(float(trace.x[index]), float(trace.y[index]))
        waypoints.append(
            RouteWaypoint(
                x=float(trace.x[index]),
                y=float(trace.y[index]),
                alt_m=float(altitude_m[index]),
                timestamp_s=float(dense_time[index]),
                speed_mps=float(dense_speed[index]),
                ground_m=float(trace.terrain_m[index]),
                distance_m=float(trace.distance_m[index]),
                lat=float(lat),
                lon=float(lon),
            )
        )
    candidate.waypoints = waypoints
    validation = validate_route_waypoints(dem, candidate.waypoints, dynamics)
    if not bool(validation["valid"]):
        for code in validation["failure_codes"]:
            if code not in candidate.failure_codes:
                candidate.failure_codes.append(str(code))
        terrain_codes = {
            "OUTSIDE_DEM",
            "NODATA_CROSSED",
            "TERRAIN_CLEARANCE_VIOLATION",
        }
        if any(code in terrain_codes for code in validation["failure_codes"]):
            candidate.terrain_safe = False
        if any(code not in terrain_codes for code in validation["failure_codes"]):
            candidate.dynamics_feasible = False
    elif math.isfinite(float(validation["min_clearance_m"])):
        candidate.min_clearance_m = min(
            candidate.min_clearance_m,
            float(validation["min_clearance_m"]),
        )
    return candidate


def _evaluate_vertical_hover(
    dem: Any,
    *,
    own_x: float,
    own_y: float,
    own_alt_m: float,
    current_speed_mps: float,
    endpoint: HideEndpointCandidate,
    endpoint_index: int,
    deadline_s: float,
    dynamics: RouteDynamics,
) -> RouteCandidate:
    """Evaluate an exact-own endpoint as a bounded vertical hover manoeuvre."""

    candidate = RouteCandidate(
        endpoint_index=int(endpoint_index),
        kind="vertical_hover",
        lateral_offset_m=0.0,
        endpoint=endpoint,
        distance_m=0.0,
        endpoint_canonical_los_required=bool(
            getattr(endpoint, "requires_canonical_los_validation", False)
        ),
    )
    if float(current_speed_mps) > 0.5:
        candidate.failure_codes.append("ZERO_LENGTH_ROUTE_WHILE_MOVING")
        return candidate
    row, col = dem.nearest_index(float(own_x), float(own_y))
    if row < 0 or col < 0 or row >= int(dem.height) or col >= int(dem.width):
        candidate.failure_codes.append("OWN_OUTSIDE_DEM")
        return candidate
    ground_m = max(
        float(dem.block_elev[int(row), int(col)]),
        float(endpoint.ground_m),
    )
    if not math.isfinite(ground_m) or ground_m < -1000.0:
        candidate.failure_codes.append("NODATA_CROSSED")
        return candidate
    clearance_floor_m = ground_m + float(dynamics.clearance_m)
    own_altitude_m = float(own_alt_m)
    if own_altitude_m < clearance_floor_m - 1e-6:
        candidate.failure_codes.append("START_BELOW_CLEARANCE")
        return candidate
    if own_altitude_m > float(dynamics.max_altitude_m) + 1e-6:
        candidate.failure_codes.append("MAX_ALTITUDE_EXCEEDED")
        return candidate


    interval_low_m = max(float(endpoint.min_altitude_m), clearance_floor_m)
    interval_high_m = min(
        float(endpoint.max_altitude_m), float(dynamics.max_altitude_m)
    )
    if interval_low_m > interval_high_m + 1e-6:
        candidate.failure_codes.append("ENDPOINT_INTERVAL_BELOW_CLEARANCE")
        return candidate
    # Nearest altitude in the strict interval minimizes concealment time.  If
    # already inside it, the endpoint is reached at t=0 exactly as expected.
    arrival_alt_m = float(np.clip(own_altitude_m, interval_low_m, interval_high_m))
    altitude_delta_m = arrival_alt_m - own_altitude_m
    if altitude_delta_m >= 0.0:
        eta_s = altitude_delta_m / _finite_positive(dynamics.climb_rate_mps, 6.675)
    else:
        eta_s = -altitude_delta_m / _finite_positive(dynamics.descent_rate_mps, 5.25)

    candidate.terrain_safe = True
    candidate.dynamics_feasible = True
    candidate.endpoint_interval_validated = True
    candidate.eta_s = float(eta_s)
    candidate.deadline_met = candidate.eta_s <= float(deadline_s) + 1e-9
    if not candidate.deadline_met:
        candidate.failure_codes.append("DEADLINE_EXCEEDED")
    candidate.min_clearance_m = min(own_altitude_m, arrival_alt_m) - ground_m
    candidate.max_altitude_m = max(own_altitude_m, arrival_alt_m)
    candidate.max_speed_mps = 0.0
    candidate.arrival_altitude_m = arrival_alt_m

    lat, lon = dem.native_to_latlon(float(own_x), float(own_y))
    initial = RouteWaypoint(
        x=float(own_x),
        y=float(own_y),
        alt_m=own_altitude_m,
        timestamp_s=0.0,
        speed_mps=0.0,
        ground_m=ground_m,
        distance_m=0.0,
        lat=float(lat),
        lon=float(lon),
    )
    candidate.waypoints = [initial]
    if eta_s > 1e-9:
        candidate.waypoints.append(
            RouteWaypoint(
                x=float(own_x),
                y=float(own_y),
                alt_m=arrival_alt_m,
                timestamp_s=float(eta_s),
                speed_mps=0.0,
                ground_m=ground_m,
                distance_m=0.0,
                lat=float(lat),
                lon=float(lon),
            )
        )
    candidate.profile_distance_m = [0.0] if eta_s <= 1e-9 else [0.0, 0.0]
    candidate.profile_x = [float(own_x)] * len(candidate.profile_distance_m)
    candidate.profile_y = [float(own_y)] * len(candidate.profile_distance_m)
    candidate.profile_terrain_m = [ground_m] * len(candidate.profile_distance_m)
    candidate.profile_altitude_m = (
        [own_altitude_m]
        if eta_s <= 1e-9
        else [own_altitude_m, arrival_alt_m]
    )
    candidate.profile_time_s = [0.0] if eta_s <= 1e-9 else [0.0, float(eta_s)]
    candidate.profile_speed_mps = [0.0] * len(candidate.profile_distance_m)
    validation = validate_route_waypoints(dem, candidate.waypoints, dynamics)
    if not bool(validation["valid"]):
        for code in validation["failure_codes"]:
            if code not in candidate.failure_codes:
                candidate.failure_codes.append(str(code))
        candidate.terrain_safe = False
        candidate.dynamics_feasible = False
    return candidate


def _invalid_result(
    deadline_s: float,
    started_at: float,
    code: str,
) -> HideRouteResult:
    return HideRouteResult(
        selected=None,
        candidates=[],
        green_valid=False,
        best_effort=False,
        deadline_s=float(deadline_s),
        failure_codes=[str(code)],
        planning_elapsed_s=time.perf_counter() - started_at,
    )


def plan_hide_communication_routes(
    dem: Any,
    own_x: float,
    own_y: float,
    own_alt_m: float,
    current_speed_mps: float,
    heading_deg: float,
    endpoint_candidates: Sequence[HideEndpointCandidate],
    *,
    deadline_s: float = 10.0,
    dynamics: RouteDynamics | None = None,
) -> HideRouteResult:
    """Plan a native-terrain-safe route to a hide/communication endpoint.

    The fastest fully valid route at or below ``deadline_s`` is returned as a
    green-valid selection.  If none meets the deadline, the fastest route that
    still satisfies *all* terrain, endpoint and LAH limits is returned as a
    best effort.  Limits are never relaxed to manufacture a 10-second result.

    Parameters are native projected metres and MSL metres.  ``dem`` is normally
    :class:`NativeDemRaster`; a compatible test double may expose the same
    ``block_elev``, affine/coordinate and CRS helper API.
    """

    started_at = time.perf_counter()
    settings = dynamics or RouteDynamics()
    deadline = float(deadline_s)
    scalar_values = (own_x, own_y, own_alt_m, current_speed_mps, heading_deg, deadline)
    if not all(math.isfinite(float(value)) for value in scalar_values):
        return _invalid_result(deadline if math.isfinite(deadline) else 10.0, started_at, "INVALID_INPUT")
    if deadline <= 0.0:
        return _invalid_result(deadline, started_at, "INVALID_DEADLINE")
    if float(current_speed_mps) < 0.0:
        return _invalid_result(deadline, started_at, "INVALID_CURRENT_SPEED")
    if not dem.contains_native(float(own_x), float(own_y)):
        return _invalid_result(deadline, started_at, "OWN_OUTSIDE_DEM")
    own_row, own_col = dem.nearest_index(float(own_x), float(own_y))
    own_ground = float(dem.block_elev[int(own_row), int(own_col)])
    if not math.isfinite(own_ground) or own_ground < -1000.0:
        return _invalid_result(deadline, started_at, "OWN_ON_NODATA")

    endpoints = list(endpoint_candidates or [])
    if not endpoints:
        return _invalid_result(deadline, started_at, "NO_ENDPOINT_CANDIDATES")
    endpoints = endpoints[: max(1, int(settings.max_endpoint_candidates))]

    evaluated: list[RouteCandidate] = []

    for endpoint_index, endpoint in enumerate(endpoints):
        try:
            goal_x = float(endpoint.x)
            goal_y = float(endpoint.y)
            interval_ok = (
                math.isfinite(float(endpoint.min_altitude_m))
                and math.isfinite(float(endpoint.max_altitude_m))
                and float(endpoint.min_altitude_m) <= float(endpoint.max_altitude_m)
            )
        except (TypeError, ValueError, OverflowError, AttributeError):
            continue
        if not interval_ok or not dem.contains_native(goal_x, goal_y):
            continue
        if math.hypot(goal_x - float(own_x), goal_y - float(own_y)) <= 1e-6:
            evaluated.append(
                _evaluate_vertical_hover(
                    dem,
                    own_x=float(own_x),
                    own_y=float(own_y),
                    own_alt_m=float(own_alt_m),
                    current_speed_mps=float(current_speed_mps),
                    endpoint=endpoint,
                    endpoint_index=endpoint_index,
                    deadline_s=deadline,
                    dynamics=settings,
                )
            )
            continue
        for kind, lateral_offset_m, points in _horizontal_geometries(
            (float(own_x), float(own_y)),
            (goal_x, goal_y),
            float(heading_deg),
            settings,
            float(current_speed_mps),
        ):
            evaluated.append(
                _evaluate_geometry(
                    dem,
                    own_alt_m=float(own_alt_m),
                    current_speed_mps=float(current_speed_mps),
                    heading_deg=float(heading_deg),
                    endpoint=endpoint,
                    endpoint_index=endpoint_index,
                    kind=kind,
                    lateral_offset_m=lateral_offset_m,
                    points=points,
                    deadline_s=deadline,
                    dynamics=settings,
                )
            )

    def selection_key(candidate: RouteCandidate) -> tuple[float, float, float, int, str]:
        return (
            float(candidate.eta_s),
            float(candidate.max_altitude_m),
            float(candidate.distance_m),
            int(candidate.endpoint_index),
            "0" if candidate.kind == "direct" else str(candidate.kind),
        )

    route_feasible = [
        candidate for candidate in evaluated if candidate.route_constraints_valid
    ]
    ordered_candidates = sorted(
        (candidate for candidate in route_feasible if candidate.deadline_met),
        key=selection_key,
    ) + sorted(
        (candidate for candidate in route_feasible if not candidate.deadline_met),
        key=selection_key,
    )
    selected: RouteCandidate | None = None
    for candidate in ordered_candidates:
        if candidate.endpoint_canonical_los_required:
            _validate_endpoint_canonical_los(candidate)
        if candidate.valid:
            selected = candidate
            break
    failure_codes: list[str] = []
    if selected is None:
        failure_codes.append("NO_LIMIT_COMPLIANT_TERRAIN_SAFE_ROUTE")
        for candidate in evaluated:
            for code in candidate.failure_codes:
                if code not in failure_codes:
                    failure_codes.append(code)
    elif not selected.deadline_met:
        failure_codes.append("DEADLINE_EXCEEDED")

    return HideRouteResult(
        selected=selected,
        candidates=evaluated,
        green_valid=bool(selected is not None and selected.green_valid),
        best_effort=bool(selected is not None and selected.best_effort),
        deadline_s=deadline,
        failure_codes=failure_codes,
        planning_elapsed_s=time.perf_counter() - started_at,
    )


__all__ = [
    "RouteDynamics",
    "RouteWaypoint",
    "RouteCandidate",
    "HideRouteResult",
    "plan_hide_communication_routes",
    "validate_route_waypoints",
]
