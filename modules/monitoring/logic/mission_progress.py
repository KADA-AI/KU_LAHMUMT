# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

from shapely.geometry import GeometryCollection, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from modules.monitoring.logic.area_search_interpolation import (
    build_frame_interpolated_footprint_geometry,
    build_path_frame_interpolated_footprint_geometry,
    build_sweep_endpoint_fill_geometry,
    resolve_frame_sample_fractions,
)
from modules.monitoring.logic.capture_gate import evaluate_capture_gate
from modules.monitoring.logic.boundary_guard_progress import (
    BoundaryGuardProgressGate,
)
from modules.monitoring.logic.mission_coverage import (
    MissionCoverageDefinition,
    build_footprint_geometry,
    build_mission_coverage_definition,
    build_projected_sweep_path,
    merge_coverage_geometry,
    project_coordinate_to_sweep_path,
)
from modules.monitoring.logic.coverage_settings import (
    load_coverage_settings,
)
from modules.monitoring.logic.mission_line_progress import (
    LineSweepDefinition,
    LineSweepState,
    _width_hint_m,
    build_line_sweep_definition,
    force_complete_line_sweep_state,
    line_sweep_metrics,
    reset_line_sweep_state,
    update_line_sweep_state,
)
from modules.monitoring.logic.spatial_coverage_depth import (
    SpatialCoverageDepthLedger,
    stable_capture_source_id,
)
from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float

try:
    from modules.mission_planning.runtime.attack_tracking_state import (
        list_active_tracking_assignments,
    )
except Exception:
    def list_active_tracking_assignments() -> list[dict[str, Any]]:
        return []

_ON_MISSION_STARTUP_GUARD_MS = 10000
_ON_MISSION_BLOCK_FLIGHT_MODES = {1, 2, 3}
_PRECISE_SWEEP_MAX_SENSOR_DISTANCE_M = 1500.0
_COVERAGE_COMPLETE_ABSOLUTE_TOLERANCE_M2 = 0.05
_COVERAGE_COMPLETE_RELATIVE_TOLERANCE = 1e-6


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_float(value: object) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _coverage_pass_contract_rows(mission: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rows_by_pass: dict[str, dict[str, Any]] = {}
    for key in (
        "coverage_pass_details",
        "coveragePassDetails",
        "coverage_pass_obligations",
        "coveragePassObligations",
    ):
        rows = mission.get(key)
        if not isinstance(rows, list):
            continue
        for raw_row in rows:
            if not isinstance(raw_row, dict):
                continue
            pass_name = str(
                raw_row.get("coveragePass", raw_row.get("coverage_pass")) or ""
            ).strip().lower()
            if pass_name not in {"forward", "reverse"}:
                continue
            merged = dict(rows_by_pass.get(pass_name) or {})
            merged.update(raw_row)
            rows_by_pass[pass_name] = merged
    return rows_by_pass


def _coverage_pass_remaining_geometry(
    row: dict[str, Any] | None,
    coverage_def: MissionCoverageDefinition,
) -> BaseGeometry | None:
    detail = None
    if isinstance(row, dict):
        detail = row.get("remainingDetail", row.get("remaining_detail"))
    if not isinstance(detail, dict):
        return None

    def _polygon(coords: object) -> BaseGeometry | None:
        points: list[tuple[float, float]] = []
        for item in coords or []:
            if not isinstance(item, dict):
                continue
            lat_value = item.get("latitude")
            if lat_value is None:
                lat_value = item.get("Latitude")
            lon_value = item.get("longitude")
            if lon_value is None:
                lon_value = item.get("Longitude")
            lat = _coerce_float(lat_value)
            lon = _coerce_float(lon_value)
            if lat is None or lon is None:
                continue
            try:
                x_val, y_val = coverage_def.transformer.transform(float(lon), float(lat))
            except Exception:
                continue
            points.append((float(x_val), float(y_val)))
        if len(points) < 3:
            return None
        try:
            polygon = Polygon(points)
            if not polygon.is_valid:
                polygon = polygon.buffer(0)
        except Exception:
            return None
        return polygon if polygon is not None and not polygon.is_empty else None

    segment_parts = [
        geometry
        for geometry in (
            _polygon(item.get("coordinateList"))
            for item in (detail.get("areaSegmentList") or [])
            if isinstance(item, dict)
        )
        if geometry is not None and not geometry.is_empty
    ]
    geometry: BaseGeometry | None = None
    if segment_parts:
        try:
            geometry = unary_union(segment_parts)
        except Exception:
            geometry = None
    else:
        outer_parts: list[BaseGeometry] = []
        hole_parts: list[BaseGeometry] = []
        for item in detail.get("areaList") or []:
            if not isinstance(item, dict):
                continue
            polygon = _polygon(item.get("coordinateList"))
            if polygon is None or polygon.is_empty:
                continue
            (hole_parts if bool(item.get("isHole")) else outer_parts).append(polygon)
        if not outer_parts:
            legacy = _polygon(detail.get("coordinateList"))
            if legacy is not None:
                outer_parts.append(legacy)
        if outer_parts:
            try:
                geometry = unary_union(outer_parts)
                if hole_parts:
                    geometry = geometry.difference(unary_union(hole_parts))
            except Exception:
                geometry = None
    if geometry is None or geometry.is_empty:
        return None
    try:
        geometry = coverage_def.assignment_geometry.intersection(geometry)
        if not geometry.is_valid:
            geometry = geometry.buffer(0)
    except Exception:
        return None
    return geometry if geometry is not None and not geometry.is_empty else None


def _coverage_detail_geometry(
    detail: object,
    coverage_def: MissionCoverageDefinition,
) -> BaseGeometry | None:
    if not isinstance(detail, dict):
        return None
    return _coverage_pass_remaining_geometry(
        {"remainingDetail": detail},
        coverage_def,
    )


def _seed_depth_contract_sources(
    mission: dict[str, Any],
    coverage_def: MissionCoverageDefinition,
) -> tuple[dict[str, BaseGeometry], dict[str, dict[str, Any]]]:
    sources: dict[str, BaseGeometry] = {}
    attribution: dict[str, dict[str, Any]] = {}
    for index, row in enumerate(mission.get("coverage_observation_details") or []):
        if not isinstance(row, dict):
            continue
        source_id = str(
            row.get("acquisitionID")
            or row.get("acquisitionId")
            or row.get("acquisition_id")
            or row.get("coverageAcquisitionID")
            or row.get("coverageAcquisitionId")
            or row.get("observationID")
            or row.get("observationId")
            or f"portable:observation:{index + 1}"
        ).strip()
        detail = (
            row.get("coveredDetail")
            or row.get("observationDetail")
            or row.get("coverageDetail")
            or row.get("remainingDetail")
        )
        geometry = _coverage_detail_geometry(detail, coverage_def)
        if not source_id or geometry is None or geometry.is_empty:
            continue
        sources[source_id] = merge_coverage_geometry(sources.get(source_id), geometry)
        attribution[source_id] = {
            "aircraftID": _coerce_int(row.get("aircraftID", row.get("aircraft_id"))),
            "coveragePass": row.get("coveragePass", row.get("coverage_pass")),
            "acquisitionID": source_id,
            "source": "portable_depth_observation",
        }
    if sources:
        return sources, attribution

    # Backward-compatible reconstruction when a portable contract contains
    # only exact depth bands.  depth=2 is placed in two synthetic layers;
    # depth=1 is placed in only the first, preserving the measured depth.
    first_layer: BaseGeometry | None = None
    second_layer: BaseGeometry | None = None
    for row in mission.get("coverage_depth_details") or []:
        if not isinstance(row, dict):
            continue
        depth = _coerce_int(row.get("coverageDepth", row.get("coverage_depth")))
        if depth is None or depth <= 0:
            continue
        detail = row.get("remainingDetail") or row.get("coverageDetail")
        geometry = _coverage_detail_geometry(detail, coverage_def)
        if geometry is None or geometry.is_empty:
            continue
        first_layer = merge_coverage_geometry(first_layer, geometry)
        if int(depth) >= 2:
            second_layer = merge_coverage_geometry(second_layer, geometry)
    if first_layer is not None and not first_layer.is_empty:
        sources["portable:depth-layer:1"] = first_layer
        attribution["portable:depth-layer:1"] = {"source": "portable_depth_band"}
    if second_layer is not None and not second_layer.is_empty:
        sources["portable:depth-layer:2"] = second_layer
        attribution["portable:depth-layer:2"] = {"source": "portable_depth_band"}
    return sources, attribution


def _derive_cumulative_etas(raw_etas: list[float]) -> tuple[list[float], bool]:
    """Return cumulative ETAs and whether the raw values looked cumulative."""
    if not raw_etas:
        return [], False
    values = [max(0.0, float(v)) for v in raw_etas]
    eps = 1e-6
    is_non_decreasing = all(values[idx] + eps >= values[idx - 1] for idx in range(1, len(values)))
    starts_at_zero = values[0] <= eps
    if starts_at_zero and is_non_decreasing:
        return values, True
    cumulative: list[float] = []
    total = 0.0
    for v in values:
        total += v
        cumulative.append(total)
    return cumulative, False


def _coord_distance_m(a: object, b: object) -> float | None:
    if not isinstance(a, dict) or not isinstance(b, dict):
        return None
    lat1 = _coerce_float(a.get("latitude") or a.get("Latitude"))
    lon1 = _coerce_float(a.get("longitude") or a.get("Longitude"))
    lat2 = _coerce_float(b.get("latitude") or b.get("Latitude"))
    lon2 = _coerce_float(b.get("longitude") or b.get("Longitude"))
    if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
        return None
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    d_phi = math.radians(float(lat2) - float(lat1))
    d_lambda = math.radians(float(lon2) - float(lon1))
    hav = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
    )
    return 6_371_000.0 * 2.0 * math.atan2(math.sqrt(hav), math.sqrt(max(0.0, 1.0 - hav)))


def _nearest_coord_index(
    coords: list[dict[str, Any]],
    target: object,
) -> tuple[int | None, float | None]:
    best_idx: int | None = None
    best_dist: float | None = None
    for idx, coord in enumerate(coords or []):
        dist_m = _coord_distance_m(coord, target)
        if dist_m is None:
            continue
        if best_dist is None or float(dist_m) < float(best_dist):
            best_idx = int(idx)
            best_dist = float(dist_m)
    return best_idx, best_dist


def _coverage_completion_metrics(
    covered_area_m2: float,
    required_area_m2: float,
) -> tuple[float, float, bool]:
    """Return remaining area, numeric tolerance, and actual requirement status.

    Completion deliberately follows measured footprint area, not waypoint/path
    completion.  The tiny tolerance only absorbs geometry floating-point noise.
    """
    required = max(0.0, float(required_area_m2))
    covered = max(0.0, min(required, float(covered_area_m2)))
    remaining = max(0.0, required - covered)
    tolerance = max(
        float(_COVERAGE_COMPLETE_ABSOLUTE_TOLERANCE_M2),
        required * float(_COVERAGE_COMPLETE_RELATIVE_TOLERANCE),
    )
    return remaining, tolerance, bool(required > 0.0 and remaining <= tolerance)


def _coverage_sensor_offset_bypass_allowed(
    coverage_def: MissionCoverageDefinition,
    meta: "MissionMeta | None",
    *,
    current_waypoint_id: int | None,
    footprint_geometry: BaseGeometry | None,
) -> bool:
    """Permit a long sensor look only on a planned sweep over its assignment."""
    if meta is None or current_waypoint_id is None:
        return False
    sweep_coordinates = (meta.waypoint_sweep_coords or {}).get(int(current_waypoint_id))
    if not isinstance(sweep_coordinates, list) or len(sweep_coordinates) < 2:
        return False
    if footprint_geometry is None or footprint_geometry.is_empty:
        return False
    try:
        return float(
            footprint_geometry.intersection(coverage_def.assignment_geometry).area or 0.0
        ) > 1e-6
    except Exception:
        return False


@dataclass
class MissionMeta:
    mission_id: int
    aircraft_id: int
    input_id: int | None
    package_id: int | None
    path_id: int | None
    planned_seconds: float
    waypoint_ids: list[int]
    waypoint_eta_cumulative: dict[int, float]
    waypoint_index: dict[int, int]
    sweep_point_count: int = 0
    waypoint_sweep_start_index: dict[int, int] | None = None
    waypoint_sweep_point_count: dict[int, int] | None = None
    waypoint_sweep_coords: dict[int, list[dict[str, Any]]] | None = None
    has_filming: bool = False
    requires_filming_completion: bool = False
    post_attack_boundary_hold: bool = False
    width_hint_m: float | None = None
    coverage_pass_by_waypoint_id: dict[int, str] | None = None
    coverage_pass_order: tuple[str, ...] = ()
    coverage_acquisition_id_by_waypoint_id: dict[int, str] | None = None
    coverage_acquisition_id: str | None = None
    coverage_generation_token: object | None = None
    coverage_required_depth: int = 1
    input_mission_type: int | None = None
    region_type: int | None = None
    boundary_guard_loop: bool = False
    boundary_guard_set_id: str | None = None


@dataclass
class MissionProgressState:
    completed_seconds: float = 0.0
    current_waypoint_id: int | None = None
    segment_start_ms: int | None = None
    done: bool = False
    paused: bool = False
    awaiting_execute: bool = False
    elapsed_seconds: float = 0.0
    last_update_ms: int | None = None
    path_done: bool = False
    sweep_done: bool = False
    flying_status: int | None = None
    filming_status: int | None = None
    sweep_progress_points: int = 0


@dataclass
class MissionCoverageState:
    covered_geometry: BaseGeometry | None = None
    covered_area_m2: float = 0.0
    last_footprint_geometry: BaseGeometry | None = None
    last_footprint_timestamp_ms: int | None = None
    covered_geometry_by_pass: dict[str, BaseGeometry | None] = field(default_factory=dict)
    covered_area_m2_by_pass: dict[str, float] = field(default_factory=dict)
    last_footprint_geometry_by_pass: dict[str, BaseGeometry | None] = field(default_factory=dict)
    last_footprint_timestamp_ms_by_pass: dict[str, int | None] = field(default_factory=dict)
    last_sweep_waypoint_id: int | None = None
    last_sweep_chainage_m: float | None = None
    last_sweep_waypoint_id_by_pass: dict[str, int | None] = field(default_factory=dict)
    last_sweep_chainage_m_by_pass: dict[str, float | None] = field(default_factory=dict)
    covered_geometry_by_source: dict[str, BaseGeometry | None] = field(default_factory=dict)
    coverage_source_attribution: dict[str, dict[str, Any]] = field(default_factory=dict)


class MissionProgressTracker:
    def __init__(self) -> None:
        self._system_mode_code: int | None = None
        self._on_mission_startup_guard_requested: bool = False
        self._on_mission_startup_guard_pending: set[int] = set()
        self._on_mission_startup_guard_first_wp: dict[int, int | None] = {}
        self._on_mission_startup_guard_baselined: set[int] = set()
        self._on_mission_startup_guard_start_ms: dict[int, int] = {}
        self._boundary_guard_gate = BoundaryGuardProgressGate(
            default_duration_s=max(
                0.001,
                float(get_runtime_float("type2_boundary_guard_duration_s", 600.0)),
            )
        )
        self.reset({})

    def set_system_mode(self, mode_code: int | None) -> None:
        mode = _coerce_int(mode_code)
        prev = self._system_mode_code
        self._system_mode_code = mode
        if mode == 3:
            if prev != 3:
                self._on_mission_startup_guard_requested = True
                self._arm_on_mission_startup_guard()
                if self._on_mission_startup_guard_pending:
                    self._on_mission_startup_guard_requested = False
            return
        self._on_mission_startup_guard_requested = False
        self._clear_on_mission_startup_guard()

    def reset(self, view: dict[str, Any] | None) -> None:
        previous_actual_by_input_pass: dict[tuple[int, str], BaseGeometry] = {}
        previous_actual_by_input_source: dict[tuple[int, str], BaseGeometry] = {}
        previous_source_attribution: dict[tuple[int, str], dict[str, Any]] = {}
        for previous_mission_id, previous_meta in getattr(self, "_mission_meta", {}).items():
            if previous_meta.input_id is None:
                continue
            previous_state = getattr(self, "_mission_coverage_state", {}).get(previous_mission_id)
            if previous_state is None:
                continue
            for pass_name in tuple(previous_meta.coverage_pass_order or ()):
                geometry = previous_state.covered_geometry_by_pass.get(pass_name)
                if geometry is None or geometry.is_empty:
                    continue
                key = (int(previous_meta.input_id), str(pass_name))
                previous_actual_by_input_pass[key] = merge_coverage_geometry(
                    previous_actual_by_input_pass.get(key),
                    geometry,
                )
            for source_id, geometry in previous_state.covered_geometry_by_source.items():
                if geometry is None or geometry.is_empty:
                    continue
                source_key = (int(previous_meta.input_id), str(source_id))
                previous_actual_by_input_source[source_key] = merge_coverage_geometry(
                    previous_actual_by_input_source.get(source_key),
                    geometry,
                )
                attribution = previous_state.coverage_source_attribution.get(str(source_id))
                if isinstance(attribution, dict):
                    previous_source_attribution[source_key] = dict(attribution)
        self._mission_meta: dict[int, MissionMeta] = {}
        self._progress_state: dict[int, MissionProgressState] = {}
        self._aircraft_missions: dict[int, list[int]] = {}
        self._waypoint_to_mission: dict[int, dict[int, int]] = {}
        self._input_to_missions: dict[int, list[int]] = {}
        self._input_mission_ids: list[int] = []
        self._aircraft_current_mission: dict[int, int | None] = {}
        self._mission_to_package: dict[int, int | None] = {}
        self._last_completed_idx: dict[int, int] = {}
        self._completed_mission_ids: set[int] = set()
        self._completed_input_ids: set[int] = set()
        self._last_timestamp_ms: int | None = None
        self._paused_aircraft: set[int] = set()
        self._aircraft_hold_mission: dict[int, int] = {}
        self._formation_followers: dict[int, dict[str, int | None]] = {}
        self._formation_followers_map: dict[int, int | None] = {}
        self._leader_mission_by_aircraft_input: dict[tuple[int, int], int] = {}
        self._waypoint_state: dict[int, dict[int, str]] = {}
        self._waypoint_actual_seconds: dict[int, dict[int, float]] = {}
        self._waypoint_actual_real_seconds: dict[int, dict[int, float]] = {}
        self._waypoint_completion_ts_ms: dict[int, dict[int, int | None]] = {}
        self._mission_coverage_defs: dict[int, MissionCoverageDefinition] = {}
        self._mission_coverage_state: dict[int, MissionCoverageState] = {}
        self._mission_sweep_paths: dict[tuple[int, int], Any] = {}
        self._mission_line_defs: dict[int, LineSweepDefinition] = {}
        self._mission_line_state: dict[int, LineSweepState] = {}
        self._forced_active_input_id: int | None = None
        self._declared_current_input_id: int | None = (
            _coerce_int(view.get("current_input_mission_id"))
            if isinstance(view, dict)
            else None
        )
        self._fallback_baseline_ms: dict[int, int] = {}
        self._fallback_baseline_monotonic: dict[int, float] = {}
        self._boundary_guard_gate.configure(view)

        if not view:
            return

        input_missions = view.get("input_missions") or []
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            input_id = _coerce_int(item.get("input_mission_id"))
            if input_id is None:
                continue
            self._input_mission_ids.append(input_id)
            if item.get("is_done"):
                self._completed_input_ids.add(input_id)

        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = _coerce_int(entry.get("aircraft_id"))
            if aircraft_id is None:
                continue
            package_id = _coerce_int(entry.get("individual_mission_package_id"))
            self._aircraft_missions.setdefault(aircraft_id, [])
            self._aircraft_current_mission[aircraft_id] = _coerce_int(
                entry.get("current_individual_mission_id")
            )
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                if bool(mission.get("execution_blocked_until_next_collab")):
                    # Retained future artifacts are not part of the executable
                    # controller sequence yet.  Keeping them in the progress
                    # fallback list lets WP=0/flying=2 falsely finish work that
                    # SIM never loaded.
                    continue
                mission_id = _coerce_int(mission.get("individual_mission_id"))
                if mission_id is None:
                    continue
                input_id = _coerce_int(mission.get("input_id"))
                formation_leader_id = _coerce_int(mission.get("formation_leader_id"))
                is_formation_follower = bool(mission.get("skip_progress"))
                if not is_formation_follower:
                    if formation_leader_id is not None and int(formation_leader_id) != int(aircraft_id):
                        is_formation_follower = True
                path_id = _coerce_int(mission.get("path_id"))
                planned_seconds = _coerce_float(mission.get("eta_seconds")) or 0.0
                waypoint_ids: list[int] = []
                raw_etas: list[float] = []
                provided_cumulative: list[float | None] = []
                sweep_point_count = _coerce_int(mission.get("sweep_point_count")) or 0
                has_filming = bool(sweep_point_count > 0)
                requires_filming_completion = False
                for wp in mission.get("waypoints") or []:
                    if not isinstance(wp, dict):
                        continue
                    wid = _coerce_int(wp.get("waypoint_id") or wp.get("waypointID"))
                    if wid is None:
                        continue
                    waypoint_ids.append(wid)
                    eta = _coerce_float(wp.get("eta"))
                    raw_etas.append(float(eta) if eta is not None else 0.0)
                    cum_eta = _coerce_float(wp.get("eta_cumulative"))
                    provided_cumulative.append(float(cum_eta) if cum_eta is not None else None)
                    operation_mode = _coerce_int(wp.get("operation_mode"))
                    if operation_mode is None:
                        operation_mode = _coerce_int(wp.get("operationMode"))
                    if operation_mode is None:
                        operation_mode = _coerce_int(wp.get("OperationMode"))
                    filming_property = (
                        wp.get("filmingProperty")
                        or wp.get("FilmingProperty")
                        or wp.get("filming_property")
                    )
                    if operation_mode is None and isinstance(filming_property, dict):
                        operation_mode = _coerce_int(filming_property.get("operation_mode"))
                        if operation_mode is None:
                            operation_mode = _coerce_int(filming_property.get("operationMode"))
                        if operation_mode is None:
                            operation_mode = _coerce_int(filming_property.get("OperationMode"))
                    if operation_mode == 2:
                        requires_filming_completion = True
                        has_filming = True
                    if (
                        wp.get("has_filming_property")
                        or wp.get("hasFilmingProperty")
                        or wp.get("has_line_search")
                        or wp.get("hasLineSearch")
                        or (_coerce_int(wp.get("line_search_point_count")) or 0) > 0
                    ):
                        has_filming = True
                if not waypoint_ids:
                    for wid in mission.get("waypoint_ids") or []:
                        wid_int = _coerce_int(wid)
                        if wid_int is None:
                            continue
                        waypoint_ids.append(wid_int)
                        raw_etas.append(0.0)
                        provided_cumulative.append(None)

                cumulative_etas: list[float] = []
                if waypoint_ids:
                    if provided_cumulative and all(val is not None for val in provided_cumulative):
                        cumulative_etas = [max(0.0, float(val)) for val in provided_cumulative]  # type: ignore[arg-type]
                    else:
                        derived, _used_cumulative = _derive_cumulative_etas(raw_etas)
                        cumulative_etas = list(derived) if derived else [0.0] * len(waypoint_ids)
                        if len(cumulative_etas) < len(waypoint_ids):
                            pad = cumulative_etas[-1] if cumulative_etas else 0.0
                            cumulative_etas.extend([pad] * (len(waypoint_ids) - len(cumulative_etas)))
                        for idx, val in enumerate(provided_cumulative):
                            if val is None:
                                continue
                            if idx < len(cumulative_etas):
                                cumulative_etas[idx] = max(cumulative_etas[idx], max(0.0, float(val)))

                    for idx in range(1, len(cumulative_etas)):
                        if cumulative_etas[idx] < cumulative_etas[idx - 1]:
                            cumulative_etas[idx] = cumulative_etas[idx - 1]

                last_cumulative = cumulative_etas[-1] if cumulative_etas else 0.0
                total_seconds = max(planned_seconds, last_cumulative, 0.0)
                if waypoint_ids and total_seconds > 0:
                    if not cumulative_etas:
                        denom = max(1, len(waypoint_ids) - 1)
                        cumulative_etas = [
                            float(total_seconds) * (idx / denom) for idx in range(len(waypoint_ids))
                        ]
                    else:
                        last_cumulative = cumulative_etas[-1]
                        if last_cumulative <= 0:
                            denom = max(1, len(waypoint_ids) - 1)
                            cumulative_etas = [
                                float(total_seconds) * (idx / denom) for idx in range(len(waypoint_ids))
                            ]
                        elif abs(total_seconds - last_cumulative) / max(total_seconds, 1.0) > 0.01:
                            scale = float(total_seconds) / float(last_cumulative)
                            cumulative_etas = [float(val) * scale for val in cumulative_etas]
                    if cumulative_etas:
                        cumulative_etas[-1] = float(total_seconds)
                planned_seconds = float(total_seconds)

                waypoint_eta_cumulative: dict[int, float] = {}
                waypoint_index: dict[int, int] = {}
                waypoint_sweep_start_index: dict[int, int] = {}
                waypoint_sweep_point_count: dict[int, int] = {}
                waypoint_sweep_coords: dict[int, list[dict[str, Any]]] = {}
                sweep_lists = [
                    [dict(coord) for coord in coords if isinstance(coord, dict)]
                    for coords in (mission.get("sweep_line_coordinate_lists") or [])
                    if isinstance(coords, list)
                ]
                sweep_list_idx = 0
                sweep_point_offset = 0
                for idx, wid in enumerate(waypoint_ids):
                    cum_val = cumulative_etas[idx] if idx < len(cumulative_etas) else 0.0
                    waypoint_eta_cumulative[int(wid)] = float(max(0.0, cum_val))
                    waypoint_index[int(wid)] = int(idx)
                    waypoint_def = (
                        mission.get("waypoints")[idx]
                        if isinstance(mission.get("waypoints"), list) and idx < len(mission.get("waypoints"))
                        else {}
                    )
                    point_count = (
                        _coerce_int((waypoint_def or {}).get("line_search_point_count"))
                        if isinstance(waypoint_def, dict)
                        else None
                    ) or 0
                    if point_count > 0:
                        coords_for_wp: list[dict[str, Any]] = []
                        if sweep_list_idx < len(sweep_lists):
                            coords_for_wp = sweep_lists[sweep_list_idx]
                            sweep_list_idx += 1
                        if coords_for_wp:
                            point_count = len(coords_for_wp)
                        waypoint_sweep_start_index[int(wid)] = int(sweep_point_offset)
                        waypoint_sweep_point_count[int(wid)] = int(point_count)
                        waypoint_sweep_coords[int(wid)] = coords_for_wp
                        sweep_point_offset += int(point_count)
                coverage_pass_by_waypoint_id: dict[int, str] = {}
                coverage_pass_order: list[str] = []
                coverage_acquisition_id_by_waypoint_id: dict[int, str] = {}
                for waypoint_def in mission.get("waypoints") or []:
                    if not isinstance(waypoint_def, dict):
                        continue
                    pass_name = str(
                        waypoint_def.get("area_coverage_pass") or ""
                    ).strip().lower()
                    waypoint_id = _coerce_int(waypoint_def.get("waypoint_id"))
                    if pass_name not in {"forward", "reverse"} or waypoint_id is None:
                        continue
                    if int(waypoint_def.get("line_search_point_count") or 0) <= 0:
                        continue
                    coverage_pass_by_waypoint_id[int(waypoint_id)] = pass_name
                    acquisition_id = str(
                        waypoint_def.get("coverage_acquisition_id")
                        or waypoint_def.get("coverageAcquisitionID")
                        or waypoint_def.get("coverageAcquisitionId")
                        or ""
                    ).strip()
                    if acquisition_id:
                        coverage_acquisition_id_by_waypoint_id[int(waypoint_id)] = acquisition_id
                    if pass_name not in coverage_pass_order:
                        coverage_pass_order.append(pass_name)
                explicit_pass_contract = bool(
                    _coerce_int(mission.get("area_coverage_pass_contract_version"))
                    or mission.get("remaining_coverage_passes")
                    or mission.get("coverage_pass_policy") == "all_passes_required"
                )
                explicit_depth_contract = bool(
                    _coerce_int(mission.get("area_coverage_depth_contract_version"))
                    or str(mission.get("coverage_depth_policy") or "").strip().lower()
                    == "spatial_capture_depth"
                )
                explicit_remaining_passes = [
                    pass_name
                    for pass_name in (
                        str(value or "").strip().lower()
                        for value in (mission.get("remaining_coverage_passes") or [])
                    )
                    if pass_name in {"forward", "reverse"}
                ]
                contract_rows = _coverage_pass_contract_rows(mission)
                explicit_completed_passes = [
                    pass_name
                    for pass_name in (
                        str(value or "").strip().lower()
                        for value in (mission.get("completed_coverage_passes") or [])
                    )
                    if pass_name in {"forward", "reverse"}
                ]
                completed_contract_passes = set(explicit_completed_passes)
                for pass_name, pass_row in contract_rows.items():
                    if bool(pass_row.get("isDone", pass_row.get("is_done", False))):
                        completed_contract_passes.add(str(pass_name))
                declared_contract_order = [
                    pass_name
                    for pass_name in (
                        str(value or "").strip().lower()
                        for value in (mission.get("coverage_pass_order") or [])
                    )
                    if pass_name in {"forward", "reverse"}
                ]
                if not declared_contract_order and contract_rows:
                    declared_contract_order = [
                        str(pass_name)
                        for pass_name, _row in sorted(
                            contract_rows.items(),
                            key=lambda item: (
                                int(
                                    _coerce_int(
                                        item[1].get("passIndex", item[1].get("pass_index"))
                                    )
                                    or 999
                                ),
                                str(item[0]),
                            ),
                        )
                    ]
                if not coverage_pass_order and len(explicit_remaining_passes) == 1:
                    sole_pass = str(explicit_remaining_passes[0])
                    for waypoint_id, point_count in waypoint_sweep_point_count.items():
                        if int(point_count or 0) > 0:
                            coverage_pass_by_waypoint_id[int(waypoint_id)] = sole_pass
                    if coverage_pass_by_waypoint_id:
                        coverage_pass_order = [sole_pass]
                waypoint_pass_order = list(coverage_pass_order)
                if explicit_remaining_passes:
                    waypoint_pass_order = [
                        pass_name
                        for pass_name in explicit_remaining_passes
                        if pass_name in coverage_pass_order
                    ]
                coverage_pass_order = []
                for pass_name in [
                    *declared_contract_order,
                    *waypoint_pass_order,
                    *explicit_completed_passes,
                ]:
                    if pass_name in coverage_pass_order:
                        continue
                    if (
                        pass_name in coverage_pass_by_waypoint_id.values()
                        or pass_name in completed_contract_passes
                    ):
                        coverage_pass_order.append(str(pass_name))
                coverage_pass_by_waypoint_id = {
                    waypoint_id: pass_name
                    for waypoint_id, pass_name in coverage_pass_by_waypoint_id.items()
                    if pass_name in coverage_pass_order
                }
                if not coverage_pass_order or (
                    len(coverage_pass_order) < 2 and not explicit_pass_contract
                ):
                    coverage_pass_by_waypoint_id = {}
                    coverage_pass_order = []

                mission_acquisition_id = str(
                    mission.get("coverage_acquisition_id")
                    or mission.get("coverageAcquisitionID")
                    or mission.get("coverageAcquisitionId")
                    or ""
                ).strip() or None
                required_depth = _coerce_int(
                    mission.get("required_coverage_depth")
                    or mission.get("requiredCoverageDepth")
                )
                if required_depth is None:
                    required_depth = (
                        2
                        if coverage_pass_order
                        or explicit_pass_contract
                        or explicit_depth_contract
                        else 1
                    )
                required_depth = max(1, int(required_depth))

                meta = MissionMeta(
                    mission_id=mission_id,
                    aircraft_id=aircraft_id,
                    input_id=input_id,
                    package_id=package_id,
                    path_id=path_id,
                    planned_seconds=planned_seconds,
                    waypoint_ids=waypoint_ids,
                    waypoint_eta_cumulative=waypoint_eta_cumulative,
                    waypoint_index=waypoint_index,
                    sweep_point_count=int(max(0, sweep_point_count)),
                    waypoint_sweep_start_index=waypoint_sweep_start_index,
                    waypoint_sweep_point_count=waypoint_sweep_point_count,
                    waypoint_sweep_coords=waypoint_sweep_coords,
                    has_filming=bool(has_filming),
                    requires_filming_completion=bool(requires_filming_completion),
                    post_attack_boundary_hold=bool(mission.get("post_attack_boundary_hold")),
                    width_hint_m=float(_width_hint_m(mission)),
                    coverage_pass_by_waypoint_id=coverage_pass_by_waypoint_id,
                    coverage_pass_order=tuple(coverage_pass_order),
                    coverage_acquisition_id_by_waypoint_id=coverage_acquisition_id_by_waypoint_id,
                    coverage_acquisition_id=mission_acquisition_id,
                    coverage_generation_token=(
                        mission.get("coverage_generation_token")
                        if mission.get("coverage_generation_token") is not None
                        else mission.get("flight_path_timestamp_ms")
                    ),
                    coverage_required_depth=int(required_depth),
                    input_mission_type=_coerce_int(
                        mission.get("input_mission_type")
                    ),
                    region_type=_coerce_int(mission.get("region_type")),
                    boundary_guard_loop=bool(
                        mission.get("boundary_guard_loop")
                    ),
                    boundary_guard_set_id=(
                        str(mission.get("boundary_guard_set_id")).strip()
                        if mission.get("boundary_guard_set_id") is not None
                        else None
                    ),
                )
                self._mission_meta[mission_id] = meta
                coverage_def = build_mission_coverage_definition(mission)
                if coverage_def is not None:
                    self._mission_coverage_defs[mission_id] = coverage_def
                    if input_id is not None and (
                        coverage_pass_order or explicit_depth_contract
                    ):
                        seeded_state = MissionCoverageState()
                        aggregate_geometry: BaseGeometry | None = coverage_def.assignment_geometry
                        covered_work_m2 = 0.0
                        if explicit_depth_contract:
                            contract_sources, contract_attribution = _seed_depth_contract_sources(
                                mission,
                                coverage_def,
                            )
                            seeded_state.covered_geometry_by_source.update(contract_sources)
                            seeded_state.coverage_source_attribution.update(contract_attribution)
                        for (seed_input_id, source_id), source_geometry in previous_actual_by_input_source.items():
                            if int(seed_input_id) != int(input_id):
                                continue
                            try:
                                clipped_source = coverage_def.assignment_geometry.intersection(source_geometry)
                            except Exception:
                                continue
                            if clipped_source is None or clipped_source.is_empty:
                                continue
                            seeded_state.covered_geometry_by_source[str(source_id)] = clipped_source
                            attribution = previous_source_attribution.get(
                                (int(seed_input_id), str(source_id))
                            )
                            if isinstance(attribution, dict):
                                seeded_state.coverage_source_attribution[str(source_id)] = dict(attribution)
                        for pass_name in coverage_pass_order:
                            seed = previous_actual_by_input_pass.get((int(input_id), str(pass_name)))
                            clipped_seed: BaseGeometry | None = None
                            if seed is not None and not seed.is_empty:
                                try:
                                    clipped_seed = coverage_def.assignment_geometry.intersection(seed)
                                except Exception:
                                    clipped_seed = None
                            pass_remaining = (
                                None
                                if explicit_depth_contract
                                else _coverage_pass_remaining_geometry(
                                    contract_rows.get(str(pass_name)),
                                    coverage_def,
                                )
                            )
                            if pass_remaining is not None and not pass_remaining.is_empty:
                                try:
                                    contract_seed = coverage_def.assignment_geometry.difference(
                                        pass_remaining
                                    )
                                except Exception:
                                    contract_seed = None
                                if contract_seed is not None and not contract_seed.is_empty:
                                    clipped_seed = merge_coverage_geometry(
                                        clipped_seed,
                                        contract_seed,
                                    )
                            if pass_name in completed_contract_passes and not explicit_depth_contract:
                                clipped_seed = coverage_def.assignment_geometry
                            seeded_state.covered_geometry_by_pass[str(pass_name)] = clipped_seed
                            pass_area = float(clipped_seed.area or 0.0) if clipped_seed is not None else 0.0
                            seeded_state.covered_area_m2_by_pass[str(pass_name)] = pass_area
                            covered_work_m2 += pass_area
                            if aggregate_geometry is not None:
                                try:
                                    aggregate_geometry = aggregate_geometry.intersection(
                                        clipped_seed if clipped_seed is not None else GeometryCollection()
                                    )
                                except Exception:
                                    aggregate_geometry = GeometryCollection()
                            if (
                                clipped_seed is not None
                                and not clipped_seed.is_empty
                                and not explicit_depth_contract
                            ):
                                has_preserved_source = any(
                                    str(
                                        (
                                            seeded_state.coverage_source_attribution.get(source_id)
                                            or {}
                                        ).get("coveragePass")
                                        or ""
                                    ) == str(pass_name)
                                    or str(source_id).endswith(f":pass:{pass_name}")
                                    for source_id in seeded_state.covered_geometry_by_source
                                )
                                if not has_preserved_source:
                                    legacy_source_id = f"legacy:pass:{pass_name}"
                                    seeded_state.covered_geometry_by_source[legacy_source_id] = merge_coverage_geometry(
                                        seeded_state.covered_geometry_by_source.get(legacy_source_id),
                                        clipped_seed,
                                    )
                                    seeded_state.coverage_source_attribution[legacy_source_id] = {
                                        "coveragePass": str(pass_name),
                                        "source": "portable_pass_contract",
                                    }
                        if int(required_depth) > 1:
                            depth_ledger = SpatialCoverageDepthLedger(
                                required_depth=int(required_depth),
                                observations_by_source={
                                    str(key): value
                                    for key, value in seeded_state.covered_geometry_by_source.items()
                                    if value is not None and not value.is_empty
                                },
                                attribution_by_source=dict(seeded_state.coverage_source_attribution),
                            )
                            depth_metrics = depth_ledger.metrics(coverage_def.assignment_geometry)
                            seeded_state.covered_geometry = depth_metrics.completed_geometry
                            seeded_state.covered_area_m2 = float(depth_metrics.work_covered_m2)
                        else:
                            seeded_state.covered_geometry = aggregate_geometry
                            seeded_state.covered_area_m2 = float(covered_work_m2)
                        self._mission_coverage_state[mission_id] = seeded_state
                    for sweep_waypoint_id, sweep_coordinates in waypoint_sweep_coords.items():
                        projected_path = build_projected_sweep_path(
                            sweep_coordinates,
                            coverage_def.transformer,
                        )
                        if projected_path is not None:
                            self._mission_sweep_paths[
                                (int(mission_id), int(sweep_waypoint_id))
                            ] = projected_path
                line_def = build_line_sweep_definition(mission, coverage_def=coverage_def)
                if line_def is not None:
                    self._mission_line_defs[mission_id] = line_def
                self._mission_to_package[mission_id] = package_id
                self._last_completed_idx.setdefault(mission_id, -1)
                self._waypoint_state[mission_id] = {
                    int(wid): "pending" for wid in waypoint_ids
                }
                self._waypoint_actual_seconds[mission_id] = {}
                self._waypoint_actual_real_seconds[mission_id] = {}
                self._waypoint_completion_ts_ms[mission_id] = {}
                self._aircraft_missions[aircraft_id].append(mission_id)
                if input_id is not None:
                    self._input_to_missions.setdefault(input_id, []).append(mission_id)
                if is_formation_follower:
                    if formation_leader_id is not None:
                        self._formation_followers[mission_id] = {
                            "leader_aircraft_id": int(formation_leader_id),
                            "input_id": int(input_id) if input_id is not None else None,
                        }
                else:
                    if input_id is not None:
                        self._leader_mission_by_aircraft_input[(aircraft_id, int(input_id))] = mission_id
                for wid in waypoint_ids:
                    self._waypoint_to_mission.setdefault(aircraft_id, {})
                    self._waypoint_to_mission[aircraft_id].setdefault(wid, mission_id)
                if mission.get("is_done"):
                    self._progress_state[mission_id] = MissionProgressState(
                        completed_seconds=float(planned_seconds),
                        done=True,
                        elapsed_seconds=float(planned_seconds),
                        path_done=True,
                        sweep_done=True,
                        flying_status=2,
                        filming_status=2,
                    )
                    if coverage_def is not None:
                        self._mission_coverage_state[mission_id] = MissionCoverageState()
                    if line_def is not None:
                        line_state = LineSweepState()
                        force_complete_line_sweep_state(line_def, line_state)
                        self._mission_line_state[mission_id] = line_state
                    self._completed_mission_ids.add(mission_id)
                    if waypoint_ids:
                        self._last_completed_idx[mission_id] = len(waypoint_ids) - 1
                        for idx, wid in enumerate(waypoint_ids):
                            planned_eta = float(cumulative_etas[idx]) if idx < len(cumulative_etas) else float(planned_seconds)
                            self._waypoint_state[mission_id][int(wid)] = "reached"
                            self._waypoint_actual_seconds[mission_id][int(wid)] = float(planned_eta)
                            self._waypoint_actual_real_seconds[mission_id][int(wid)] = float(planned_eta)
                            self._waypoint_completion_ts_ms[mission_id][int(wid)] = None
                else:
                    self._progress_state.setdefault(mission_id, MissionProgressState())
                    if coverage_def is not None:
                        self._mission_coverage_state.setdefault(
                            mission_id,
                            MissionCoverageState(),
                        )
                    if line_def is not None:
                        self._mission_line_state.setdefault(mission_id, LineSweepState())

        if self._formation_followers:
            for follower_id, info in self._formation_followers.items():
                leader_id = None
                leader_aircraft = info.get("leader_aircraft_id")
                input_id = info.get("input_id")
                if leader_aircraft is not None and input_id is not None:
                    leader_id = self._leader_mission_by_aircraft_input.get(
                        (int(leader_aircraft), int(input_id))
                    )
                self._formation_followers_map[follower_id] = leader_id

        if self._system_mode_code == 3:
            # Re-arm guard on every mission-view reset (e.g. 0702 ignore=2 plan switch)
            # so stale/transient flying=2 does not immediately trigger execute-ready flow.
            self._arm_on_mission_startup_guard()
            self._on_mission_startup_guard_requested = False
        else:
            self._on_mission_startup_guard_requested = False
            self._clear_on_mission_startup_guard()

    def update(
        self,
        timestamp_ms: int | None,
        agent_states: list[dict[str, Any]] | None,
    ) -> dict[str, Any]:
        new_completed_individual: list[dict[str, int | None]] = []
        new_completed_waypoints: list[dict[str, Any]] = []
        if timestamp_ms is not None:
            self._last_timestamp_ms = int(timestamp_ms)
        tracking_assignments: list[dict[str, Any]] = []
        try:
            tracking_assignments = [
                dict(item)
                for item in list_active_tracking_assignments()
                if isinstance(item, dict)
            ]
        except Exception:
            tracking_assignments = []
        self._boundary_guard_gate.update(
            timestamp_ms=timestamp_ms,
            agent_states=agent_states or [],
            active_tracking_assignments=tracking_assignments,
        )
        for state in agent_states or []:
            if not isinstance(state, dict):
                continue
            aircraft_id = _coerce_int(state.get("aircraft_id"))
            if aircraft_id is None:
                continue
            if aircraft_id not in self._aircraft_missions:
                continue
            try:
                self._update_single_agent_state(
                    state=state,
                    aircraft_id=int(aircraft_id),
                    timestamp_ms=timestamp_ms,
                    new_completed_individual=new_completed_individual,
                    new_completed_waypoints=new_completed_waypoints,
                )
            except Exception:
                # Per-aircraft safeguard: if one aircraft's telemetry is malformed
                # or trips an unexpected edge case (e.g., stale waypoint id from a
                # previous mission after a "next collab base mission" trigger), do
                # not let it block the remaining aircraft from being processed.
                continue

        # Boundary guard loops intentionally never publish flying/filming=2.
        # Once the monitor-observed duration/cycle/tracking contract is met,
        # complete through the normal MissionProgress path so new_completed_*
        # and the existing 0503 recommendation validation remain authoritative.
        for input_id in self._boundary_guard_gate.ready_input_ids():
            for mission_id in self._input_to_missions.get(int(input_id), []):
                self._force_complete_mission(
                    mission_id=int(mission_id),
                    timestamp_ms=timestamp_ms,
                    out_completed_individual=new_completed_individual,
                    out_waypoint_updates=new_completed_waypoints,
                )

        formation_map = self._sync_formation_followers(timestamp_ms, new_completed_individual)
        new_completed_input: list[int] = []
        snapshot = self._build_snapshot(
            timestamp_ms,
            new_completed_individual,
            new_completed_input,
            new_completed_waypoints,
            formation_map=formation_map,
        )
        return snapshot

    def _update_single_agent_state(
        self,
        *,
        state: dict[str, Any],
        aircraft_id: int,
        timestamp_ms: int | None,
        new_completed_individual: list[dict[str, int | None]],
        new_completed_waypoints: list[dict[str, Any]],
    ) -> None:
        current_wp = _coerce_int(state.get("current_waypoint_id"))
        if current_wp is not None and current_wp <= 0:
            current_wp = None
        flying_status = _coerce_int(state.get("flying"))
        filming_status = _coerce_int(state.get("filming"))
        flying_status = self._filter_startup_on_mission(
            aircraft_id=aircraft_id,
            current_wp=current_wp,
            on_mission=flying_status,
            timestamp_ms=timestamp_ms,
        )
        flight_mode = _coerce_int(state.get("flight_mode"))
        if (
            flying_status == 2
            and flight_mode is not None
            and int(flight_mode) in _ON_MISSION_BLOCK_FLIGHT_MODES
        ):
            # During auto takeoff/landing/handover-point transit, ignore transient
            # flying=2 so mission progress is not forced to 100%.
            flying_status = None
        prev_mission_id = self._aircraft_current_mission.get(aircraft_id)
        mission_id = self._resolve_forced_input_mission(aircraft_id, current_wp)
        if mission_id is None:
            resolved_mission_id = self._resolve_mission_for_waypoint(aircraft_id, current_wp)
            if resolved_mission_id is not None and not self._input_switch_allowed(
                prev_mission_id=prev_mission_id,
                candidate_mission_id=resolved_mission_id,
            ):
                mission_id = prev_mission_id
            else:
                mission_id = resolved_mission_id
        if mission_id is None:
            mission_id = self._aircraft_current_mission.get(aircraft_id)
        if mission_id is None:
            missions = self._aircraft_missions.get(aircraft_id) or []
            mission_id = missions[0] if missions else None
        if mission_id is None:
            return

        mission_meta = self._mission_meta.get(int(mission_id))
        boundary_guard_pending = bool(
            mission_meta is not None
            and mission_meta.input_id is not None
            and self._boundary_guard_gate.is_guard_input(mission_meta.input_id)
            and not self._boundary_guard_gate.is_ready(mission_meta.input_id)
        )
        if boundary_guard_pending and flying_status == 2:
            # A child path tail may still emit the legacy completion status
            # while the controller advances to the next loop child.  It is not
            # the ten-minute guard completion boundary.
            flying_status = 1

        has_direct_wp_match = (
            current_wp is not None
            and self._mission_contains_waypoint(int(mission_id), int(current_wp))
        )
        combined_on_mission = self._combined_on_mission_status(
            int(mission_id),
            flying_status=flying_status,
            filming_status=filming_status,
        )
        if (
            combined_on_mission == 2
            and self._is_post_attack_boundary_hold_mission(int(mission_id))
            and not self._terminal_mission_has_execution_evidence(
                int(mission_id),
                current_wp=current_wp,
            )
        ):
            # A simulator can keep publishing flying=2 together with the
            # previous plan's waypoint after a direct replan.  A terminal
            # post-attack hold would otherwise be completed as soon as the
            # startup grace expires, even though the new waypoint was never
            # loaded or observed.  Treat that status as stale until an exact
            # waypoint from this terminal mission has been seen.
            flying_status = None
            combined_on_mission = self._combined_on_mission_status(
                int(mission_id),
                flying_status=flying_status,
                filming_status=filming_status,
            )
        if (
            combined_on_mission != 2
            and has_direct_wp_match
            and not boundary_guard_pending
        ):
            self._complete_prior_missions_on_waypoint_jump(
                aircraft_id=aircraft_id,
                prev_mission_id=prev_mission_id,
                current_mission_id=mission_id,
                timestamp_ms=timestamp_ms,
                out_completed_individual=new_completed_individual,
                out_waypoint_updates=new_completed_waypoints,
            )
        hold_id: int | None = None
        if combined_on_mission == 2:
            prev_id = self._aircraft_current_mission.get(aircraft_id)
            hold_id = prev_id if prev_id is not None else mission_id
            self._aircraft_hold_mission[aircraft_id] = hold_id
            mission_id = hold_id
            combined_on_mission = self._combined_on_mission_status(
                int(mission_id),
                flying_status=flying_status,
                filming_status=filming_status,
            )
        else:
            self._aircraft_hold_mission.pop(aircraft_id, None)

        state_obj = self._progress_state.setdefault(mission_id, MissionProgressState())
        state_obj.flying_status = flying_status
        state_obj.filming_status = filming_status
        if flying_status == 2:
            state_obj.path_done = True
        if self._sweep_complete_from_status(
            int(mission_id),
            flying_status=flying_status,
            filming_status=filming_status,
        ):
            state_obj.sweep_done = True
            self._force_complete_line_progress(int(mission_id))
        prev_wp_id = state_obj.current_waypoint_id
        if timestamp_ms is not None:
            ts_int = int(timestamp_ms)
            if prev_mission_id is not None and prev_mission_id != mission_id:
                state_obj.last_update_ms = ts_int
            else:
                last_ms = state_obj.last_update_ms
                if last_ms is None:
                    state_obj.last_update_ms = ts_int
                else:
                    if (
                        not state_obj.done
                        and not state_obj.awaiting_execute
                        and not state_obj.paused
                ):
                        delta = (ts_int - int(last_ms)) / 1000.0
                        if delta > 0:
                            state_obj.elapsed_seconds += float(delta)
                    state_obj.last_update_ms = ts_int
        self._aircraft_current_mission[aircraft_id] = mission_id
        self._update_line_sweep_progress(mission_id, state, timestamp_ms=timestamp_ms)
        self._update_sweep_point_progress(mission_id, state, current_wp=current_wp)
        self._update_mission_coverage(mission_id, state, timestamp_ms=timestamp_ms)
        meta = self._mission_meta.get(mission_id)
        if self._update_mission_state(mission_id, current_wp, combined_on_mission, timestamp_ms):
            if mission_id not in self._completed_mission_ids:
                self._completed_mission_ids.add(mission_id)
                new_completed_individual.append(
                    {
                        "mission_id": mission_id,
                        "package_id": self._mission_to_package.get(mission_id),
                    }
                )
        actual_progress = self._progress_seconds(meta, state_obj, timestamp_ms)
        actual_real = max(0.0, float(state_obj.elapsed_seconds))
        self._record_waypoint_observation(
            mission_id=mission_id,
            waypoint_id=current_wp,
            timestamp_ms=timestamp_ms,
            actual_seconds=actual_progress,
            actual_real_seconds=actual_real,
        )
        self._record_waypoint_completion(
            mission_id,
            current_wp,
            prev_wp_id,
            flying_status,
            timestamp_ms,
            new_completed_waypoints,
        )

    def _complete_prior_missions_on_waypoint_jump(
        self,
        *,
        aircraft_id: int,
        prev_mission_id: int | None,
        current_mission_id: int | None,
        timestamp_ms: int | None,
        out_completed_individual: list[dict[str, int | None]],
        out_waypoint_updates: list[dict[str, Any]],
    ) -> None:
        if prev_mission_id is None or current_mission_id is None:
            return
        prev_id = int(prev_mission_id)
        cur_id = int(current_mission_id)
        if prev_id == cur_id:
            return
        missions = self._aircraft_missions.get(int(aircraft_id)) or []
        if not missions:
            return
        try:
            prev_idx = missions.index(prev_id)
            cur_idx = missions.index(cur_id)
        except ValueError:
            return
        prev_meta = self._mission_meta.get(prev_id)
        cur_meta = self._mission_meta.get(cur_id)
        if prev_meta is None or cur_meta is None:
            return
        if (
            prev_meta.input_id is not None
            and cur_meta.input_id is not None
            and int(prev_meta.input_id) != int(cur_meta.input_id)
            and not self._forced_input_matches(int(cur_meta.input_id))
        ):
            return
        prev_state = self._progress_state.get(prev_id)
        # Guard against false positives right after replan:
        # if previous mission has never actually started, do not force-complete it.
        if prev_state is not None and not prev_state.done:
            if prev_state.current_waypoint_id is None and float(prev_state.completed_seconds or 0.0) <= 0.0:
                return
        # If waypoint jumped forward to a later individual mission in the same aircraft sequence,
        # treat the skipped mission blocks as passed even when the inputMissionID changed.
        # This is required for next-collab / inserted-prior flows where the vehicle can advance
        # to a later input mission without explicitly visiting the intermediate mission waypoints.
        if cur_idx <= prev_idx:
            return
        for mission_id in missions[prev_idx:cur_idx]:
            self._force_complete_mission(
                mission_id=int(mission_id),
                timestamp_ms=timestamp_ms,
                out_completed_individual=out_completed_individual,
                out_waypoint_updates=out_waypoint_updates,
            )

    def _force_complete_mission(
        self,
        *,
        mission_id: int,
        timestamp_ms: int | None,
        out_completed_individual: list[dict[str, int | None]],
        out_waypoint_updates: list[dict[str, Any]],
    ) -> None:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return
        state = self._progress_state.setdefault(int(mission_id), MissionProgressState())
        if state.done:
            return
        if meta.waypoint_ids:
            last_wp = int(meta.waypoint_ids[-1])
            state.current_waypoint_id = last_wp
            self._record_waypoint_completion(
                int(mission_id),
                last_wp,
                state.current_waypoint_id,
                2,
                timestamp_ms,
                out_waypoint_updates,
            )
            self._last_completed_idx[int(mission_id)] = len(meta.waypoint_ids) - 1
        state.done = True
        state.awaiting_execute = False
        state.paused = False
        state.path_done = True
        state.sweep_done = True
        state.flying_status = 2
        state.filming_status = 2
        state.completed_seconds = float(max(state.completed_seconds, max(0.0, meta.planned_seconds)))
        self._force_complete_line_progress(int(mission_id))
        if timestamp_ms is not None:
            state.segment_start_ms = int(timestamp_ms)
            state.last_update_ms = int(timestamp_ms)
        self._completed_mission_ids.add(int(mission_id))
        out_completed_individual.append(
            {
                "mission_id": int(mission_id),
                "package_id": self._mission_to_package.get(int(mission_id)),
            }
        )

    def get_active_input_id(self) -> int | None:
        forced_input_id = self._forced_active_input_id
        if forced_input_id is not None:
            try:
                forced_int = int(forced_input_id)
            except Exception:
                forced_int = None
            if forced_int is not None and forced_int in self._input_mission_ids:
                return forced_int
        declared_input_id = self._declared_current_input_id
        if declared_input_id is not None:
            try:
                declared_int = int(declared_input_id)
            except Exception:
                declared_int = None
            if declared_int is not None and declared_int in self._input_mission_ids:
                return declared_int
        active_counts: dict[int, int] = {}
        done_counts: dict[int, int] = {}
        for mission_id in self._aircraft_current_mission.values():
            if mission_id is None:
                continue
            meta = self._mission_meta.get(mission_id)
            if meta is None or meta.input_id is None:
                continue
            state = self._progress_state.get(mission_id)
            if state is not None and state.done:
                done_counts[meta.input_id] = done_counts.get(meta.input_id, 0) + 1
            else:
                active_counts[meta.input_id] = active_counts.get(meta.input_id, 0) + 1
        if active_counts:
            return max(active_counts.items(), key=lambda item: item[1])[0]
        if done_counts:
            return max(done_counts.items(), key=lambda item: item[1])[0]
        return None

    def get_hold_mission_id(self, aircraft_id: int | None) -> int | None:
        if aircraft_id is None:
            return None
        try:
            return self._aircraft_hold_mission.get(int(aircraft_id))
        except Exception:
            return None

    def force_complete_input(self, input_id: int | None) -> list[dict[str, int | None]]:
        if input_id is None:
            return []
        mission_ids = self._input_to_missions.get(int(input_id), [])
        completed = self.force_complete_missions(mission_ids)
        self._completed_input_ids.add(int(input_id))
        return completed

    def activate_input(self, input_id: int | None) -> dict[int, int]:
        if input_id is None:
            return {}
        target_input_id = int(input_id)
        prev_forced_input_id = self._forced_active_input_id
        self._forced_active_input_id = int(target_input_id)
        activated: dict[int, int] = {}
        for aircraft_id, mission_ids in self._aircraft_missions.items():
            for mission_id in mission_ids:
                meta = self._mission_meta.get(int(mission_id))
                if meta is None or meta.input_id is None or int(meta.input_id) != target_input_id:
                    continue
                state = self._progress_state.setdefault(int(mission_id), MissionProgressState())
                if state.done:
                    continue
                # A freshly activated "next" input often sees transient flying=2
                # before the aircraft actually starts moving on the new route.
                # Clear any stale ready flag here and re-arm the startup guard below
                # so the next 0401 sample does not immediately turn this input into
                # execute-ready/100% progress.
                state.awaiting_execute = False
                self._aircraft_current_mission[int(aircraft_id)] = int(mission_id)
                activated[int(aircraft_id)] = int(mission_id)
                break
        if (
            activated
            and self._system_mode_code == 3
            and (
                prev_forced_input_id is None
                or int(prev_forced_input_id) != int(target_input_id)
            )
        ):
            self._arm_on_mission_startup_guard()
            self._on_mission_startup_guard_requested = False
        return activated

    def _resolve_mission_for_waypoint_in_input(
        self,
        aircraft_id: int,
        waypoint_id: int | None,
        input_id: int | None,
    ) -> int | None:
        if waypoint_id is None or input_id is None:
            return None
        missions = self._aircraft_missions.get(int(aircraft_id)) or []
        if not missions:
            return None
        input_int = int(input_id)
        current_id = self._aircraft_current_mission.get(int(aircraft_id))
        if current_id is not None and self._mission_contains_waypoint(int(current_id), int(waypoint_id)):
            meta = self._mission_meta.get(int(current_id))
            if meta is not None and meta.input_id is not None and int(meta.input_id) == input_int:
                return int(current_id)
        mapping = self._waypoint_to_mission.get(int(aircraft_id), {})
        mission_id = mapping.get(int(waypoint_id))
        if mission_id is not None:
            meta = self._mission_meta.get(int(mission_id))
            if meta is not None and meta.input_id is not None and int(meta.input_id) == input_int:
                return int(mission_id)
        for mission_id in missions:
            meta = self._mission_meta.get(int(mission_id))
            if meta is None or meta.input_id is None or int(meta.input_id) != input_int or not meta.waypoint_ids:
                continue
            try:
                min_wp = min(meta.waypoint_ids)
                max_wp = max(meta.waypoint_ids)
            except ValueError:
                continue
            if min_wp <= int(waypoint_id) <= max_wp:
                return int(mission_id)
        return None

    def _resolve_forced_input_mission(
        self,
        aircraft_id: int,
        waypoint_id: int | None,
    ) -> int | None:
        forced_input_id = self._forced_active_input_id
        if forced_input_id is None:
            return None
        missions = self._aircraft_missions.get(int(aircraft_id)) or []
        if not missions:
            return None
        resolved = self._resolve_mission_for_waypoint_in_input(
            int(aircraft_id),
            waypoint_id,
            int(forced_input_id),
        )
        if resolved is not None:
            return int(resolved)
        current_id = self._aircraft_current_mission.get(int(aircraft_id))
        if current_id is not None:
            meta = self._mission_meta.get(int(current_id))
            state = self._progress_state.get(int(current_id))
            current_not_done = True if state is None else (not bool(state.done))
            if (
                meta is not None
                and meta.input_id is not None
                and int(meta.input_id) == int(forced_input_id)
                and current_not_done
            ):
                return int(current_id)
        last_same_input: int | None = None
        for mission_id in missions:
            meta = self._mission_meta.get(int(mission_id))
            if meta is None or meta.input_id is None or int(meta.input_id) != int(forced_input_id):
                continue
            last_same_input = int(mission_id)
            state = self._progress_state.setdefault(int(mission_id), MissionProgressState())
            if not state.done:
                return int(mission_id)
        return last_same_input

    def _mission_contains_waypoint(self, mission_id: int | None, waypoint_id: int | None) -> bool:
        if mission_id is None or waypoint_id is None:
            return False
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return False
        return int(waypoint_id) in meta.waypoint_index

    def force_complete_missions(self, mission_ids: list[int]) -> list[dict[str, int | None]]:
        completed: list[dict[str, int | None]] = []
        for mission_id in mission_ids:
            meta = self._mission_meta.get(mission_id)
            if meta is None:
                continue
            state = self._progress_state.setdefault(mission_id, MissionProgressState())
            if not state.done:
                state.done = True
                state.awaiting_execute = False
                state.path_done = True
                state.sweep_done = True
                state.flying_status = 2
                state.filming_status = 2
                state.completed_seconds = float(max(0.0, meta.planned_seconds))
                self._force_complete_line_progress(int(mission_id))
                if meta.waypoint_ids:
                    state.current_waypoint_id = int(meta.waypoint_ids[-1])
                    waypoint_state = self._waypoint_state.setdefault(
                        int(mission_id),
                        {int(v): "pending" for v in meta.waypoint_ids},
                    )
                    for wid in meta.waypoint_ids:
                        wid_int = int(wid)
                        if waypoint_state.get(wid_int) == "reached":
                            continue
                        waypoint_state[wid_int] = "skipped"
                    self._last_completed_idx[int(mission_id)] = len(meta.waypoint_ids) - 1
                state.segment_start_ms = self._last_timestamp_ms
            self._completed_mission_ids.add(mission_id)
            completed.append({"mission_id": mission_id, "package_id": meta.package_id})
        return completed

    def reset_input_progress(self, input_id: int | None) -> list[int]:
        if input_id is None:
            return []
        mission_ids = self._input_to_missions.get(int(input_id), [])
        self.reset_missions(mission_ids)
        self._completed_input_ids.discard(int(input_id))
        self._boundary_guard_gate.reset_input(int(input_id))
        return list(mission_ids)

    def reset_missions(self, mission_ids: list[int]) -> None:
        for mission_id in mission_ids:
            state = self._progress_state.setdefault(mission_id, MissionProgressState())
            state.completed_seconds = 0.0
            state.current_waypoint_id = None
            state.segment_start_ms = None
            state.done = False
            state.paused = False
            state.awaiting_execute = False
            state.elapsed_seconds = 0.0
            state.last_update_ms = None
            state.path_done = False
            state.sweep_done = False
            state.flying_status = None
            state.filming_status = None
            self._completed_mission_ids.discard(mission_id)
            meta = self._mission_meta.get(mission_id)
            if meta is not None:
                self._waypoint_state[mission_id] = {
                    int(wid): "pending" for wid in meta.waypoint_ids
                }
            if mission_id in self._mission_coverage_defs:
                self._mission_coverage_state[mission_id] = MissionCoverageState()
            if mission_id in self._mission_line_defs:
                line_state = self._mission_line_state.setdefault(mission_id, LineSweepState())
                reset_line_sweep_state(line_state)

    def pause_aircraft(self, aircraft_id: int | None, timestamp_ms: int | None) -> None:
        if aircraft_id is None:
            return
        aid = int(aircraft_id)
        mission_ids = self._aircraft_missions.get(aid) or []
        if not mission_ids:
            return
        if timestamp_ms is not None:
            self._last_timestamp_ms = int(timestamp_ms)
        for mission_id in mission_ids:
            meta = self._mission_meta.get(mission_id)
            if meta is None:
                continue
            state = self._progress_state.setdefault(mission_id, MissionProgressState())
            if state.done:
                continue
            progress = self._progress_seconds(meta, state, timestamp_ms)
            state.completed_seconds = max(state.completed_seconds, progress)
            if timestamp_ms is not None:
                state.segment_start_ms = int(timestamp_ms)
            state.paused = True
        self._paused_aircraft.add(aid)

    def resume_aircraft(self, aircraft_id: int | None, timestamp_ms: int | None) -> None:
        if aircraft_id is None:
            return
        aid = int(aircraft_id)
        mission_ids = self._aircraft_missions.get(aid) or []
        if not mission_ids:
            return
        if timestamp_ms is not None:
            self._last_timestamp_ms = int(timestamp_ms)
        for mission_id in mission_ids:
            state = self._progress_state.setdefault(mission_id, MissionProgressState())
            if state.done:
                continue
            state.paused = False
            if timestamp_ms is not None:
                state.segment_start_ms = int(timestamp_ms)
        self._paused_aircraft.discard(aid)

    @staticmethod
    def _coverage_source_id(
        meta: MissionMeta | None,
        *,
        coverage_pass: str | None,
        waypoint_id: int | None,
    ) -> str:
        acquisition_id = None
        if meta is not None and waypoint_id is not None:
            acquisition_id = (meta.coverage_acquisition_id_by_waypoint_id or {}).get(
                int(waypoint_id)
            )
        if not acquisition_id and meta is not None:
            acquisition_id = meta.coverage_acquisition_id
        return stable_capture_source_id(
            aircraft_id=meta.aircraft_id if meta is not None else None,
            coverage_pass=coverage_pass,
            acquisition_id=acquisition_id,
            mission_id=meta.mission_id if meta is not None else None,
            generation_token=(
                meta.coverage_generation_token if meta is not None else None
            ),
        )

    @staticmethod
    def _refresh_multi_pass_coverage(
        coverage_state: MissionCoverageState,
        coverage_def: MissionCoverageDefinition,
        pass_order: tuple[str, ...],
        *,
        required_depth: int = 2,
    ) -> None:
        required_depth = max(1, int(required_depth))
        source_geometries = {
            str(source_id): geometry
            for source_id, geometry in coverage_state.covered_geometry_by_source.items()
            if geometry is not None and not geometry.is_empty
        }
        if not source_geometries:
            source_geometries = {
                f"legacy:pass:{pass_name}": geometry
                for pass_name in pass_order
                for geometry in [coverage_state.covered_geometry_by_pass.get(pass_name)]
                if geometry is not None and not geometry.is_empty
            }
        if required_depth > 1:
            metrics = SpatialCoverageDepthLedger(
                required_depth=required_depth,
                observations_by_source=source_geometries,
                attribution_by_source=dict(coverage_state.coverage_source_attribution),
            ).metrics(coverage_def.assignment_geometry)
            coverage_state.covered_geometry = metrics.completed_geometry
            coverage_state.covered_area_m2 = float(metrics.work_covered_m2)
            return
        aggregate_geometry: BaseGeometry | None = coverage_def.assignment_geometry
        covered_work_m2 = 0.0
        for pass_name in pass_order:
            pass_geometry = coverage_state.covered_geometry_by_pass.get(pass_name)
            covered_work_m2 += float(
                coverage_state.covered_area_m2_by_pass.get(pass_name, 0.0)
            )
            if aggregate_geometry is None or pass_geometry is None:
                aggregate_geometry = None
                continue
            try:
                aggregate_geometry = aggregate_geometry.intersection(pass_geometry)
            except Exception:
                aggregate_geometry = None
        coverage_state.covered_geometry = aggregate_geometry
        coverage_state.covered_area_m2 = max(
            0.0,
            min(
                float(coverage_def.planned_area_m2) * max(1, len(pass_order)),
                float(covered_work_m2),
            ),
        )

    def _finalize_completed_sweep_endpoints(
        self,
        mission_id: int,
        coverage_state: MissionCoverageState,
        coverage_def: MissionCoverageDefinition,
        meta: MissionMeta | None,
        settings: dict[str, Any],
    ) -> None:
        if not bool(settings.get("include_sweep_endpoint_coverage", True)):
            return
        pass_order = tuple(meta.coverage_pass_order or ()) if meta is not None else ()
        pass_by_waypoint = dict(meta.coverage_pass_by_waypoint_id or {}) if meta is not None else {}

        candidates: list[tuple[str | None, int | None, BaseGeometry | None, float | None]] = []
        if pass_order:
            for pass_name in pass_order:
                candidates.append(
                    (
                        pass_name,
                        coverage_state.last_sweep_waypoint_id_by_pass.get(pass_name),
                        coverage_state.last_footprint_geometry_by_pass.get(pass_name),
                        coverage_state.last_sweep_chainage_m_by_pass.get(pass_name),
                    )
                )
        else:
            candidates.append(
                (
                    None,
                    coverage_state.last_sweep_waypoint_id,
                    coverage_state.last_footprint_geometry,
                    coverage_state.last_sweep_chainage_m,
                )
            )

        for pass_name, waypoint_id, footprint, chainage_m in candidates:
            if waypoint_id is None or footprint is None or chainage_m is None:
                continue
            if pass_name is not None and pass_by_waypoint.get(int(waypoint_id)) != pass_name:
                continue
            sweep_path = self._mission_sweep_paths.get((int(mission_id), int(waypoint_id)))
            if sweep_path is None:
                continue
            endpoint_fill = build_sweep_endpoint_fill_geometry(
                footprint,
                sweep_path,
                chainage_m,
                spacing_fraction=float(settings.get("sweep_turn_spacing_fraction", 0.5)),
                minimum_spacing_m=float(settings.get("sweep_turn_min_spacing_m", 5.0)),
                max_samples=int(settings.get("max_sweep_turn_fill_samples", 32)),
                assignment_geometry=coverage_def.assignment_geometry,
            )
            if endpoint_fill.is_empty:
                continue
            if pass_name is None:
                merged = merge_coverage_geometry(
                    coverage_state.covered_geometry,
                    endpoint_fill,
                )
                coverage_state.covered_geometry = merged
                coverage_state.covered_area_m2 = float(
                    max(0.0, min(coverage_def.planned_area_m2, merged.area))
                )
                coverage_state.last_sweep_chainage_m = float(sweep_path.length)
                continue

            merged_pass = merge_coverage_geometry(
                coverage_state.covered_geometry_by_pass.get(pass_name),
                endpoint_fill,
            )
            coverage_state.covered_geometry_by_pass[pass_name] = merged_pass
            coverage_state.covered_area_m2_by_pass[pass_name] = float(
                max(0.0, min(coverage_def.planned_area_m2, merged_pass.area))
            )
            coverage_state.last_sweep_chainage_m_by_pass[pass_name] = float(
                sweep_path.length
            )
            source_id = self._coverage_source_id(
                meta,
                coverage_pass=pass_name,
                waypoint_id=waypoint_id,
            )
            coverage_state.covered_geometry_by_source[source_id] = merge_coverage_geometry(
                coverage_state.covered_geometry_by_source.get(source_id),
                endpoint_fill,
            )
            coverage_state.coverage_source_attribution[source_id] = {
                "aircraftID": int(meta.aircraft_id) if meta is not None else None,
                "coveragePass": str(pass_name),
                "acquisitionID": source_id,
            }
        if pass_order:
            self._refresh_multi_pass_coverage(
                coverage_state,
                coverage_def,
                pass_order,
                required_depth=(
                    max(
                        int(meta.coverage_required_depth),
                        len(pass_order),
                    )
                    if meta is not None
                    else max(1, len(pass_order))
                ),
            )

    def _update_mission_coverage(
        self,
        mission_id: int,
        state: dict[str, Any],
        *,
        timestamp_ms: int | None,
    ) -> None:
        coverage_def = self._mission_coverage_defs.get(int(mission_id))
        if coverage_def is None:
            return
        coverage_state = self._mission_coverage_state.setdefault(
            int(mission_id),
            MissionCoverageState(),
        )
        meta = self._mission_meta.get(int(mission_id))
        pass_order = tuple(meta.coverage_pass_order or ()) if meta is not None else ()
        pass_by_waypoint = dict(meta.coverage_pass_by_waypoint_id or {}) if meta is not None else {}
        settings = load_coverage_settings()
        if _coerce_int(state.get("filming")) == 2:
            # The imported offline evaluator fills missing planned sweep ends
            # only after a completed candidate is known. In live processing,
            # filming=2 is the equivalent completion boundary.
            self._finalize_completed_sweep_endpoints(
                int(mission_id),
                coverage_state,
                coverage_def,
                meta,
                settings,
            )
        current_waypoint_id = _coerce_int(state.get("current_waypoint_id"))
        coverage_pass = (
            pass_by_waypoint.get(int(current_waypoint_id))
            if current_waypoint_id is not None and pass_by_waypoint
            else None
        )
        if pass_order and coverage_pass is None:
            # Reciprocal turn and re-entry points belong to neither pass.
            for pass_name in pass_order:
                coverage_state.last_footprint_geometry_by_pass[pass_name] = None
                coverage_state.last_footprint_timestamp_ms_by_pass[pass_name] = None
                coverage_state.last_sweep_waypoint_id_by_pass[pass_name] = None
                coverage_state.last_sweep_chainage_m_by_pass[pass_name] = None
            return
        required_depth = (
            max(int(meta.coverage_required_depth), len(pass_order))
            if meta is not None
            else max(1, len(pass_order))
        )
        planned_work_m2 = float(coverage_def.planned_area_m2) * max(1, required_depth)
        if coverage_state.covered_area_m2 >= planned_work_m2 - 1e-6:
            return
        footprint_corners = state.get("footprint_corners") or []
        footprint = build_footprint_geometry(footprint_corners, coverage_def.transformer)
        if footprint.is_empty:
            coverage_state.last_footprint_geometry = None
            coverage_state.last_footprint_timestamp_ms = None
            coverage_state.last_sweep_waypoint_id = None
            coverage_state.last_sweep_chainage_m = None
            if coverage_pass is not None:
                coverage_state.last_footprint_geometry_by_pass[coverage_pass] = None
                coverage_state.last_footprint_timestamp_ms_by_pass[coverage_pass] = None
                coverage_state.last_sweep_waypoint_id_by_pass[coverage_pass] = None
                coverage_state.last_sweep_chainage_m_by_pass[coverage_pass] = None
            return
        allow_sensor_offset_bypass = _coverage_sensor_offset_bypass_allowed(
            coverage_def,
            meta,
            current_waypoint_id=current_waypoint_id,
            footprint_geometry=footprint,
        )
        if not evaluate_capture_gate(
            state,
            width_hint_m=meta.width_hint_m if meta is not None else None,
            allow_sensor_offset_bypass=allow_sensor_offset_bypass,
        ):
            # 비스윕 상태에서는 면적을 적립하지 않고 직전 풋프린트/경로 연결도 끊는다.
            coverage_state.last_footprint_geometry = None
            coverage_state.last_footprint_timestamp_ms = None
            coverage_state.last_sweep_waypoint_id = None
            coverage_state.last_sweep_chainage_m = None
            if coverage_pass is not None:
                coverage_state.last_footprint_geometry_by_pass[coverage_pass] = None
                coverage_state.last_footprint_timestamp_ms_by_pass[coverage_pass] = None
                coverage_state.last_sweep_waypoint_id_by_pass[coverage_pass] = None
                coverage_state.last_sweep_chainage_m_by_pass[coverage_pass] = None
            return

        previous_footprint = (
            coverage_state.last_footprint_geometry_by_pass.get(coverage_pass)
            if coverage_pass is not None
            else coverage_state.last_footprint_geometry
        )
        previous_timestamp_ms = (
            coverage_state.last_footprint_timestamp_ms_by_pass.get(coverage_pass)
            if coverage_pass is not None
            else coverage_state.last_footprint_timestamp_ms
        )
        previous_sweep_waypoint_id = (
            coverage_state.last_sweep_waypoint_id_by_pass.get(coverage_pass)
            if coverage_pass is not None
            else coverage_state.last_sweep_waypoint_id
        )
        previous_sweep_chainage_m = (
            coverage_state.last_sweep_chainage_m_by_pass.get(coverage_pass)
            if coverage_pass is not None
            else coverage_state.last_sweep_chainage_m
        )
        sample_fractions: tuple[float, ...] | None = (1.0,)
        if (
            previous_footprint is not None
            and previous_timestamp_ms is not None
            and timestamp_ms is not None
        ):
            sample_fractions = resolve_frame_sample_fractions(
                previous_timestamp_ms,
                timestamp_ms,
                settings,
            )
            if sample_fractions is None:
                previous_footprint = None
                sample_fractions = (1.0,)
                previous_sweep_waypoint_id = None
                previous_sweep_chainage_m = None

        sweep_path = (
            self._mission_sweep_paths.get((int(mission_id), int(current_waypoint_id)))
            if current_waypoint_id is not None
            else None
        )
        projection_seed = (
            previous_sweep_chainage_m
            if previous_sweep_waypoint_id == current_waypoint_id
            else None
        )
        telemetry_delta_ms = (
            int(timestamp_ms) - int(previous_timestamp_ms)
            if timestamp_ms is not None and previous_timestamp_ms is not None
            else None
        )
        # A dense Area lineSearch can advance farther along the ordered hairpin
        # chain than the sensor centre's straight-line displacement.  The prior
        # 1.6 km/s + 150 m window clipped a measured 1.7 km/s scan at the local
        # search boundary, repeatedly dropping planned-path interpolation and
        # leaving false footprint gaps.  Keep the search forward/local, but
        # admit up to 2.0 km/s of chainage with enough turn slack for 5 Hz 0401.
        sweep_search_window_m = 2500.0
        if telemetry_delta_ms is not None and telemetry_delta_ms > 0:
            sweep_search_window_m = max(
                250.0,
                min(2500.0, (telemetry_delta_ms / 1000.0) * 2000.0 + 180.0),
            )
        current_sweep_chainage_m, sweep_path_offset_m = project_coordinate_to_sweep_path(
            sweep_path,
            state.get("sensor_center_coordinate"),
            coverage_def.transformer,
            previous_chainage_m=projection_seed,
            search_window_m=sweep_search_window_m,
        )
        path_offset_limit_m = max(
            60.0,
            min(
                200.0,
                (
                    float(meta.width_hint_m) * 1.5
                    if meta is not None and meta.width_hint_m is not None
                    else 60.0
                ),
            ),
        )
        if (
            current_sweep_chainage_m is None
            or sweep_path_offset_m is None
            or float(sweep_path_offset_m) > path_offset_limit_m
        ):
            current_sweep_chainage_m = None
        can_follow_planned_sweep = bool(
            previous_footprint is not None
            and sweep_path is not None
            and previous_sweep_waypoint_id == current_waypoint_id
            and previous_sweep_chainage_m is not None
            and current_sweep_chainage_m is not None
            and current_sweep_chainage_m > previous_sweep_chainage_m + 1e-3
            and sweep_path_offset_m is not None
            and float(sweep_path_offset_m) <= path_offset_limit_m
        )
        if can_follow_planned_sweep:
            sampled_footprints = build_path_frame_interpolated_footprint_geometry(
                previous_footprint,
                footprint,
                sweep_path,
                previous_sweep_chainage_m,
                current_sweep_chainage_m,
                sample_fractions,
                assignment_geometry=coverage_def.assignment_geometry,
            )
        else:
            sampled_footprints = build_frame_interpolated_footprint_geometry(
                previous_footprint,
                footprint,
                sample_fractions,
                assignment_geometry=coverage_def.assignment_geometry,
            )
        clipped = coverage_def.assignment_geometry.intersection(sampled_footprints)
        sample_timestamp = int(timestamp_ms) if timestamp_ms is not None else None
        if coverage_pass is None:
            merged = merge_coverage_geometry(coverage_state.covered_geometry, clipped)
            coverage_state.last_footprint_geometry = footprint
            coverage_state.last_footprint_timestamp_ms = sample_timestamp
            coverage_state.last_sweep_waypoint_id = current_waypoint_id
            coverage_state.last_sweep_chainage_m = current_sweep_chainage_m
            source_id = self._coverage_source_id(
                meta,
                coverage_pass=None,
                waypoint_id=current_waypoint_id,
            )
            coverage_state.covered_geometry_by_source[source_id] = merge_coverage_geometry(
                coverage_state.covered_geometry_by_source.get(source_id),
                clipped,
            )
            coverage_state.coverage_source_attribution[source_id] = {
                "aircraftID": int(meta.aircraft_id) if meta is not None else None,
                "coveragePass": None,
                "acquisitionID": source_id,
            }
            required_depth = (
                max(int(meta.coverage_required_depth), len(pass_order))
                if meta is not None
                else 1
            )
            if required_depth > 1:
                depth_metrics = SpatialCoverageDepthLedger(
                    required_depth=required_depth,
                    observations_by_source={
                        str(key): value
                        for key, value in coverage_state.covered_geometry_by_source.items()
                        if value is not None and not value.is_empty
                    },
                    attribution_by_source=dict(coverage_state.coverage_source_attribution),
                ).metrics(coverage_def.assignment_geometry)
                coverage_state.covered_geometry = depth_metrics.completed_geometry
                coverage_state.covered_area_m2 = float(depth_metrics.work_covered_m2)
                return
            if merged.is_empty and coverage_state.covered_area_m2 <= 0.0:
                return
            covered_area = float(max(0.0, min(coverage_def.planned_area_m2, merged.area)))
            if covered_area <= coverage_state.covered_area_m2 + 1e-6:
                return
            coverage_state.covered_geometry = merged
            coverage_state.covered_area_m2 = covered_area
            return

        previous_pass_covered = coverage_state.covered_geometry_by_pass.get(coverage_pass)
        merged_pass = merge_coverage_geometry(previous_pass_covered, clipped)
        coverage_state.last_footprint_geometry_by_pass[coverage_pass] = footprint
        coverage_state.last_footprint_timestamp_ms_by_pass[coverage_pass] = sample_timestamp
        coverage_state.last_sweep_waypoint_id_by_pass[coverage_pass] = current_waypoint_id
        coverage_state.last_sweep_chainage_m_by_pass[coverage_pass] = current_sweep_chainage_m
        pass_covered_area = float(
            max(0.0, min(coverage_def.planned_area_m2, merged_pass.area))
        )
        if pass_covered_area > float(
            coverage_state.covered_area_m2_by_pass.get(coverage_pass, 0.0)
        ) + 1e-6:
            coverage_state.covered_geometry_by_pass[coverage_pass] = merged_pass
            coverage_state.covered_area_m2_by_pass[coverage_pass] = pass_covered_area

        source_id = self._coverage_source_id(
            meta,
            coverage_pass=coverage_pass,
            waypoint_id=current_waypoint_id,
        )
        coverage_state.covered_geometry_by_source[source_id] = merge_coverage_geometry(
            coverage_state.covered_geometry_by_source.get(source_id),
            clipped,
        )
        coverage_state.coverage_source_attribution[source_id] = {
            "aircraftID": int(meta.aircraft_id) if meta is not None else None,
            "coveragePass": str(coverage_pass),
            "acquisitionID": source_id,
        }

        # Workload progress is the sum of both passes. The legacy geometry is
        # their intersection so ground is complete only after both observations.
        self._refresh_multi_pass_coverage(
            coverage_state,
            coverage_def,
            pass_order,
            required_depth=(
                max(
                    int(meta.coverage_required_depth),
                    len(pass_order),
                )
                if meta is not None
                else max(1, len(pass_order))
            ),
        )

    def _update_line_sweep_progress(
        self,
        mission_id: int,
        state: dict[str, Any],
        *,
        timestamp_ms: int | None,
    ) -> None:
        line_def = self._mission_line_defs.get(int(mission_id))
        if line_def is None:
            return
        line_state = self._mission_line_state.setdefault(int(mission_id), LineSweepState())
        if line_state.covered_length_m >= line_def.planned_length_m - 1e-6:
            return
        update_line_sweep_state(
            line_def,
            line_state,
            state,
            timestamp_ms=timestamp_ms,
        )

    def _update_sweep_point_progress(
        self,
        mission_id: int,
        state: dict[str, Any],
        *,
        current_wp: int | None,
    ) -> None:
        meta = self._mission_meta.get(int(mission_id))
        state_obj = self._progress_state.setdefault(int(mission_id), MissionProgressState())
        if meta is None or int(meta.sweep_point_count or 0) <= 0:
            return
        if state_obj.sweep_done or state_obj.done:
            state_obj.sweep_progress_points = int(meta.sweep_point_count or 0)
            return
        current_wp_int = _coerce_int(current_wp)
        if current_wp_int is None:
            return

        start_by_wp = meta.waypoint_sweep_start_index or {}
        count_by_wp = meta.waypoint_sweep_point_count or {}
        coords_by_wp = meta.waypoint_sweep_coords or {}

        candidate_points: int | None = None
        current_wp_index = meta.waypoint_index.get(int(current_wp_int))
        if current_wp_index is not None:
            prefix_points = 0
            for waypoint_id, start_idx in start_by_wp.items():
                wp_index = meta.waypoint_index.get(int(waypoint_id))
                if wp_index is None or int(wp_index) >= int(current_wp_index):
                    continue
                count = int(count_by_wp.get(int(waypoint_id), 0) or 0)
                prefix_points = max(prefix_points, int(start_idx) + count)
            candidate_points = int(prefix_points)

        coords = coords_by_wp.get(int(current_wp_int)) or []
        if coords and not evaluate_capture_gate(state, width_hint_m=meta.width_hint_m):
            # 센서 최근접 기반 정밀 전진은 실제 스윕 중일 때만 유효하다. 좌표지향
            # 주시 상태에서는 센서 중심이 촬영점 위에 있어도 진행으로 치지 않는다.
            # (웨이포인트 통과 기반 prefix 전진은 그대로 둔다.)
            coords = []
        if coords:
            sensor_center = state.get("sensor_center_coordinate")
            nearest_idx, nearest_dist_m = _nearest_coord_index(coords, sensor_center)
            if (
                nearest_idx is not None
                and nearest_dist_m is not None
                and float(nearest_dist_m) <= float(_PRECISE_SWEEP_MAX_SENSOR_DISTANCE_M)
            ):
                start_idx = int(start_by_wp.get(int(current_wp_int), 0) or 0)
                candidate_points = max(
                    int(candidate_points or 0),
                    start_idx + int(nearest_idx) + 1,
                )
        if candidate_points is None:
            return
        bounded = max(0, min(int(meta.sweep_point_count or 0), int(candidate_points)))
        if bounded > int(state_obj.sweep_progress_points or 0):
            state_obj.sweep_progress_points = int(bounded)

    def _coverage_metrics_for_mission(self, mission_id: int) -> tuple[float, float, int, bool]:
        coverage_def = self._mission_coverage_defs.get(int(mission_id))
        if coverage_def is None:
            return 0.0, 0.0, 0, False
        meta = self._mission_meta.get(int(mission_id))
        required_depth = max(
            1,
            int(meta.coverage_required_depth) if meta is not None else 1,
            len(tuple(meta.coverage_pass_order or ())) if meta is not None else 0,
        )
        planned_area = float(max(0.0, coverage_def.planned_area_m2)) * int(required_depth)
        coverage_state = self._mission_coverage_state.get(int(mission_id))
        covered_area = float(max(0.0, coverage_state.covered_area_m2 if coverage_state else 0.0))
        if planned_area <= 0.0:
            return covered_area, planned_area, 0, False
        percent = int(round((covered_area / planned_area) * 100))
        percent = max(0, min(percent, 100))
        return covered_area, planned_area, percent, True

    def _coverage_depth_metrics_for_mission(
        self,
        mission_id: int,
    ) -> tuple[Any | None, list[dict[str, Any]]]:
        coverage_def = self._mission_coverage_defs.get(int(mission_id))
        meta = self._mission_meta.get(int(mission_id))
        state = self._mission_coverage_state.get(int(mission_id))
        required_depth = max(
            int(meta.coverage_required_depth) if meta is not None else 1,
            len(tuple(meta.coverage_pass_order or ())) if meta is not None else 0,
        )
        if coverage_def is None or meta is None or required_depth <= 1:
            return None, []
        source_geometries = {
            str(source_id): geometry
            for source_id, geometry in (state.covered_geometry_by_source.items() if state else [])
            if geometry is not None and not geometry.is_empty
        }
        source_attribution = dict(state.coverage_source_attribution) if state else {}
        if not source_geometries and state is not None:
            for pass_name in tuple(meta.coverage_pass_order or ()):
                geometry = state.covered_geometry_by_pass.get(str(pass_name))
                if geometry is None or geometry.is_empty:
                    continue
                source_id = stable_capture_source_id(
                    aircraft_id=meta.aircraft_id,
                    coverage_pass=str(pass_name),
                    mission_id=meta.mission_id,
                    generation_token=meta.coverage_generation_token,
                )
                source_geometries[source_id] = geometry
                source_attribution[source_id] = {
                    "aircraftID": int(meta.aircraft_id),
                    "coveragePass": str(pass_name),
                    "acquisitionID": source_id,
                }
        ledger = SpatialCoverageDepthLedger(
            required_depth=int(required_depth),
            observations_by_source=source_geometries,
            attribution_by_source=source_attribution,
        )
        metrics = ledger.metrics(coverage_def.assignment_geometry)
        observations: list[dict[str, Any]] = []
        for source_id, geometry in ledger.observations_by_source.items():
            attribution = dict(ledger.attribution_by_source.get(source_id) or {})
            observations.append(
                {
                    "acquisition_id": str(source_id),
                    "aircraft_id": _coerce_int(attribution.get("aircraftID")),
                    "coverage_pass": attribution.get("coveragePass"),
                    "covered_area_m2": round(float(max(0.0, geometry.area or 0.0)), 3),
                }
            )
        return metrics, observations

    def _coverage_pass_metrics_for_mission(
        self,
        mission_id: int,
    ) -> list[dict[str, Any]]:
        coverage_def = self._mission_coverage_defs.get(int(mission_id))
        meta = self._mission_meta.get(int(mission_id))
        if coverage_def is None or meta is None or not (meta.coverage_pass_order or ()):
            return []
        coverage_state = self._mission_coverage_state.get(int(mission_id))
        progress_state = self._progress_state.get(int(mission_id))
        active_pass = None
        if progress_state is not None and progress_state.current_waypoint_id is not None:
            active_pass = (meta.coverage_pass_by_waypoint_id or {}).get(
                int(progress_state.current_waypoint_id)
            )
        rows: list[dict[str, Any]] = []
        planned_area_m2 = float(max(0.0, coverage_def.planned_area_m2))
        for pass_index, pass_name in enumerate(meta.coverage_pass_order, start=1):
            covered_area_m2 = float(
                max(
                    0.0,
                    min(
                        planned_area_m2,
                        (coverage_state.covered_area_m2_by_pass.get(pass_name, 0.0) if coverage_state else 0.0),
                    ),
                )
            )
            percent = (
                int(round((covered_area_m2 / planned_area_m2) * 100.0))
                if planned_area_m2 > 0.0
                else 0
            )
            remaining_area_m2, tolerance_m2, requirement_met = (
                _coverage_completion_metrics(covered_area_m2, planned_area_m2)
            )
            status = (
                "completed"
                if requirement_met
                else "active"
                if str(active_pass or "") == str(pass_name)
                else "partial"
                if covered_area_m2 > 0.0
                else "planned"
            )
            rows.append(
                {
                    "coverage_pass": str(pass_name),
                    "pass_index": int(pass_index),
                    "coverage_percent": max(0, min(100, int(percent))),
                    "covered_area_m2": round(covered_area_m2, 3),
                    "planned_area_m2": round(planned_area_m2, 3),
                    "actual_covered_area_m2": round(covered_area_m2, 3),
                    "required_area_m2": round(planned_area_m2, 3),
                    "remaining_area_m2": round(remaining_area_m2, 3),
                    "completion_tolerance_m2": round(tolerance_m2, 6),
                    "requirement_met": bool(requirement_met),
                    "is_done": bool(requirement_met),
                    "status": status,
                }
            )
        return rows

    def _spatial_coverage_metrics_for_mission(
        self,
        mission_id: int,
    ) -> tuple[float, float, int, bool]:
        """Measure ground covered on every required pass for this mission."""
        coverage_def = self._mission_coverage_defs.get(int(mission_id))
        if coverage_def is None:
            return 0.0, 0.0, 0, False
        required_area_m2 = float(max(0.0, coverage_def.planned_area_m2))
        if required_area_m2 <= 0.0:
            return 0.0, required_area_m2, 0, False
        coverage_state = self._mission_coverage_state.get(int(mission_id))
        covered_geometry = coverage_state.covered_geometry if coverage_state is not None else None
        covered_area_m2 = 0.0
        if covered_geometry is not None and not covered_geometry.is_empty:
            try:
                covered_area_m2 = float(
                    max(
                        0.0,
                        min(
                            required_area_m2,
                            coverage_def.assignment_geometry.intersection(covered_geometry).area,
                        ),
                    )
                )
            except Exception:
                covered_area_m2 = 0.0
        percent = int(round((covered_area_m2 / required_area_m2) * 100.0))
        return covered_area_m2, required_area_m2, max(0, min(100, percent)), True

    def _line_sweep_metrics_for_mission(self, mission_id: int) -> tuple[float, float, int, bool]:
        return line_sweep_metrics(
            self._mission_line_defs.get(int(mission_id)),
            self._mission_line_state.get(int(mission_id)),
        )

    def _mission_requires_sweep(self, mission_id: int) -> bool:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return False
        return bool(
            meta.requires_filming_completion
            or meta.has_filming
            or int(meta.sweep_point_count or 0) > 0
            or int(mission_id) in self._mission_line_defs
            or int(mission_id) in self._mission_coverage_defs
        )

    def _mission_requires_filming_completion(self, mission_id: int) -> bool:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return False
        return bool(meta.requires_filming_completion)

    def _sweep_complete_from_status(
        self,
        mission_id: int,
        *,
        flying_status: int | None,
        filming_status: int | None,
    ) -> bool:
        if flying_status != 2:
            return False
        if not self._mission_requires_filming_completion(int(mission_id)):
            return True
        return filming_status == 2

    def _combined_on_mission_status(
        self,
        mission_id: int,
        *,
        flying_status: int | None,
        filming_status: int | None,
    ) -> int | None:
        if flying_status == 2:
            if self._sweep_complete_from_status(
                int(mission_id),
                flying_status=flying_status,
                filming_status=filming_status,
            ):
                return 2
            return 1
        return flying_status

    def _force_complete_line_progress(self, mission_id: int) -> None:
        line_def = self._mission_line_defs.get(int(mission_id))
        if line_def is not None:
            line_state = self._mission_line_state.setdefault(int(mission_id), LineSweepState())
            force_complete_line_sweep_state(line_def, line_state)

    def _path_progress_seconds(
        self,
        meta: MissionMeta | None,
        state: MissionProgressState,
        timestamp_ms: int | None,
    ) -> float:
        if meta is None:
            return max(0.0, float(state.completed_seconds))
        planned = max(0.0, float(meta.planned_seconds))
        if state.path_done:
            return max(float(state.completed_seconds), planned)
        return self._progress_seconds(meta, state, timestamp_ms)

    def _sweep_progress_percent(
        self,
        mission_id: int,
        *,
        path_percent: int,
        line_percent: int,
        line_enabled: bool,
        coverage_percent: int,
        coverage_enabled: bool,
    ) -> int:
        state = self._progress_state.get(int(mission_id), MissionProgressState())
        if not self._mission_requires_sweep(int(mission_id)):
            return 100
        if state.sweep_done:
            return 100
        if line_enabled:
            return max(0, min(int(line_percent), 99))
        if coverage_enabled:
            return max(0, min(int(coverage_percent), 99))
        if state.filming_status in (1, 2):
            return max(0, min(int(path_percent), 99))
        return 0

    def _sync_formation_followers(
        self,
        timestamp_ms: int | None,
        new_completed_individual: list[dict[str, int | None]],
    ) -> dict[int, int]:
        if not self._formation_followers:
            return {}
        resolved: dict[int, int] = {}
        for follower_id, info in self._formation_followers.items():
            leader_id = self._formation_followers_map.get(follower_id)
            if leader_id is None:
                leader_aircraft = info.get("leader_aircraft_id")
                if leader_aircraft is not None:
                    leader_id = self._aircraft_current_mission.get(int(leader_aircraft))
                    if leader_id is None:
                        leader_list = self._aircraft_missions.get(int(leader_aircraft)) or []
                        leader_id = leader_list[0] if leader_list else None
                if leader_id is None:
                    continue
                resolved[follower_id] = int(leader_id)
            leader_state = self._progress_state.get(int(leader_id))
            if leader_state is None:
                continue
            follower_state = self._progress_state.setdefault(
                int(follower_id), MissionProgressState()
            )
            if leader_state.done and not follower_state.done:
                follower_state.done = True
                follower_state.awaiting_execute = False
                follower_state.path_done = True
                follower_state.sweep_done = True
                follower_state.flying_status = 2
                follower_state.filming_status = 2
                meta = self._mission_meta.get(int(follower_id))
                if meta is not None:
                    follower_state.completed_seconds = float(
                        max(follower_state.completed_seconds, meta.planned_seconds)
                    )
                self._force_complete_line_progress(int(follower_id))
                if timestamp_ms is not None:
                    follower_state.segment_start_ms = int(timestamp_ms)
                self._completed_mission_ids.add(int(follower_id))
                new_completed_individual.append(
                    {
                        "mission_id": int(follower_id),
                        "package_id": self._mission_to_package.get(int(follower_id)),
                    }
                )
        return resolved

    def _arm_on_mission_startup_guard(self) -> None:
        # Some simulators can momentarily publish flying=2 right after
        # mode-3 entry, before mission execution actually starts.
        self._on_mission_startup_guard_pending = set(self._aircraft_missions.keys())
        self._on_mission_startup_guard_first_wp = {
            int(aid): None for aid in self._on_mission_startup_guard_pending
        }
        self._on_mission_startup_guard_baselined = set()
        self._on_mission_startup_guard_start_ms = {}

    def _clear_on_mission_startup_guard(self) -> None:
        self._on_mission_startup_guard_pending = set()
        self._on_mission_startup_guard_first_wp = {}
        self._on_mission_startup_guard_baselined = set()
        self._on_mission_startup_guard_start_ms = {}

    def _release_on_mission_startup_guard(self, aircraft_id: int) -> None:
        aid = int(aircraft_id)
        self._on_mission_startup_guard_pending.discard(aid)
        self._on_mission_startup_guard_first_wp.pop(aid, None)
        self._on_mission_startup_guard_baselined.discard(aid)
        self._on_mission_startup_guard_start_ms.pop(aid, None)

    def _filter_startup_on_mission(
        self,
        *,
        aircraft_id: int,
        current_wp: int | None,
        on_mission: int | None,
        timestamp_ms: int | None,
    ) -> int | None:
        if self._system_mode_code != 3:
            return on_mission
        aid = int(aircraft_id)
        if aid not in self._on_mission_startup_guard_pending:
            return on_mission

        if timestamp_ms is not None:
            ts_int = int(timestamp_ms)
            start_ms = self._on_mission_startup_guard_start_ms.get(aid)
            if start_ms is None:
                self._on_mission_startup_guard_start_ms[aid] = ts_int
            elif ts_int >= start_ms and (ts_int - start_ms) >= _ON_MISSION_STARTUP_GUARD_MS:
                self._release_on_mission_startup_guard(aid)
                return on_mission

        if aid not in self._on_mission_startup_guard_baselined:
            self._on_mission_startup_guard_first_wp[aid] = current_wp
            self._on_mission_startup_guard_baselined.add(aid)
        else:
            first_wp = self._on_mission_startup_guard_first_wp.get(aid)
            if current_wp is not None and current_wp != first_wp:
                self._release_on_mission_startup_guard(aid)
                return on_mission

        if on_mission is None:
            return None
        if int(on_mission) != 2:
            self._release_on_mission_startup_guard(aid)
            return on_mission
        return None

    def _mission_input_id(self, mission_id: int | None) -> int | None:
        if mission_id is None:
            return None
        meta = self._mission_meta.get(int(mission_id))
        if meta is None or meta.input_id is None:
            return None
        return int(meta.input_id)

    def _forced_input_matches(self, input_id: int | None) -> bool:
        if input_id is None or self._forced_active_input_id is None:
            return False
        try:
            return int(input_id) == int(self._forced_active_input_id)
        except Exception:
            return False

    def _input_switch_allowed(
        self,
        *,
        prev_mission_id: int | None,
        candidate_mission_id: int | None,
    ) -> bool:
        if prev_mission_id is None or candidate_mission_id is None:
            return True
        if int(prev_mission_id) == int(candidate_mission_id):
            return True
        prev_input_id = self._mission_input_id(prev_mission_id)
        candidate_input_id = self._mission_input_id(candidate_mission_id)
        if prev_input_id is None or candidate_input_id is None:
            return True
        if int(prev_input_id) == int(candidate_input_id):
            return True
        return self._forced_input_matches(int(candidate_input_id))

    def _resolve_mission_for_waypoint(self, aircraft_id: int, waypoint_id: int | None) -> int | None:
        if waypoint_id is None:
            return None
        current_id = self._aircraft_current_mission.get(int(aircraft_id))
        if current_id is not None and self._mission_contains_waypoint(int(current_id), int(waypoint_id)):
            return int(current_id)
        mapping = self._waypoint_to_mission.get(aircraft_id, {})
        if waypoint_id in mapping:
            return mapping.get(waypoint_id)
        missions = self._aircraft_missions.get(aircraft_id) or []
        for mission_id in missions:
            meta = self._mission_meta.get(mission_id)
            if meta is None or not meta.waypoint_ids:
                continue
            try:
                min_wp = min(meta.waypoint_ids)
                max_wp = max(meta.waypoint_ids)
            except ValueError:
                continue
            if min_wp <= waypoint_id <= max_wp:
                return mission_id
        return None

    def _bounds_for_waypoint(
        self,
        meta: MissionMeta | None,
        waypoint_id: int | None,
    ) -> tuple[float, float, float] | None:
        if meta is None or waypoint_id is None or not meta.waypoint_ids:
            return None
        wp_id = int(waypoint_id)
        idx = meta.waypoint_index.get(wp_id)
        if idx is None:
            return None
        upper = float(meta.waypoint_eta_cumulative.get(wp_id, 0.0))
        if idx <= 0:
            lower = 0.0
        else:
            prev_wp = int(meta.waypoint_ids[idx - 1])
            lower = float(meta.waypoint_eta_cumulative.get(prev_wp, 0.0))
        if upper < lower:
            upper = lower
        leg = max(0.0, upper - lower)
        return lower, upper, leg

    def _lower_bound_seconds(self, meta: MissionMeta | None, waypoint_id: int | None) -> float:
        bounds = self._bounds_for_waypoint(meta, waypoint_id)
        if bounds is None:
            return 0.0
        lower, _upper, _leg = bounds
        return float(lower)

    def _progress_seconds(
        self,
        meta: MissionMeta | None,
        state: MissionProgressState,
        timestamp_ms: int | None,
    ) -> float:
        base = max(0.0, float(state.completed_seconds))
        if meta is None:
            return base
        planned = max(0.0, float(meta.planned_seconds))
        if state.paused:
            return min(base, planned) if planned > 0 else base
        if state.awaiting_execute:
            return max(base, planned)
        if state.done:
            return max(base, planned)
        bounds = self._bounds_for_waypoint(meta, state.current_waypoint_id)
        if bounds is None:
            return min(base, planned) if planned > 0 else base
        lower, upper, leg = bounds
        progress = max(base, lower)
        if timestamp_ms is None or state.segment_start_ms is None:
            return min(progress, planned) if planned > 0 else progress
        elapsed = (int(timestamp_ms) - int(state.segment_start_ms)) / 1000.0
        if elapsed < 0:
            elapsed = 0.0
        if leg > 0:
            progress = max(progress, lower + min(elapsed, leg))
        else:
            progress = max(progress, upper)
        if upper > 0:
            progress = min(progress, upper)
        if planned > 0:
            progress = min(progress, planned)
        return progress

    def _update_mission_state(
        self,
        mission_id: int,
        current_wp: int | None,
        on_mission: int | None,
        timestamp_ms: int | None,
    ) -> bool:
        state = self._progress_state.setdefault(mission_id, MissionProgressState())
        meta = self._mission_meta.get(mission_id)

        if state.done:
            return True
        if meta is None:
            return False
        if state.paused and on_mission != 2:
            return False
        if on_mission == 2:
            state.path_done = True
            state.sweep_done = True
            if self._is_terminal_mission(int(mission_id)):
                # Terminal missions should latch complete once flying=2 is observed.
                state.done = True
                state.awaiting_execute = False
            else:
                state.awaiting_execute = True
            self._force_complete_line_progress(int(mission_id))
            state.completed_seconds = float(max(state.completed_seconds, meta.planned_seconds))
            if current_wp is not None and current_wp in meta.waypoint_index:
                state.current_waypoint_id = int(current_wp)
            if timestamp_ms is not None:
                state.segment_start_ms = int(timestamp_ms)
            return bool(state.done)
        if on_mission is not None and state.awaiting_execute:
            state.awaiting_execute = False
        if current_wp is None:
            return False
        if current_wp not in meta.waypoint_index:
            return False

        lower_bound = self._lower_bound_seconds(meta, current_wp)
        if state.current_waypoint_id is None:
            state.completed_seconds = max(state.completed_seconds, lower_bound)
            state.current_waypoint_id = int(current_wp)
            state.segment_start_ms = int(timestamp_ms) if timestamp_ms is not None else None
            return False
        if current_wp == state.current_waypoint_id:
            state.completed_seconds = max(state.completed_seconds, lower_bound)
            return False

        current_progress = self._progress_seconds(meta, state, timestamp_ms)
        state.completed_seconds = max(current_progress, lower_bound)
        state.current_waypoint_id = int(current_wp)
        state.segment_start_ms = int(timestamp_ms) if timestamp_ms is not None else None
        return False

    def _is_terminal_mission(self, mission_id: int) -> bool:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return False
        if self._is_post_attack_boundary_hold_mission(int(mission_id)):
            return True
        missions = self._aircraft_missions.get(int(meta.aircraft_id)) or []
        if not missions:
            return False
        try:
            return int(missions[-1]) == int(mission_id)
        except Exception:
            return False

    def _is_post_attack_boundary_hold_mission(self, mission_id: int) -> bool:
        meta = self._mission_meta.get(int(mission_id))
        return meta is not None and bool(
            getattr(meta, "post_attack_boundary_hold", False)
        )

    def _terminal_mission_has_execution_evidence(
        self,
        mission_id: int,
        *,
        current_wp: int | None,
    ) -> bool:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return False
        if current_wp is not None and int(current_wp) in meta.waypoint_index:
            return True
        state = self._progress_state.get(int(mission_id))
        observed_wp = state.current_waypoint_id if state is not None else None
        return observed_wp is not None and int(observed_wp) in meta.waypoint_index

    def _build_snapshot(
        self,
        timestamp_ms: int | None,
        new_completed_individual: list[dict[str, int | None]],
        new_completed_input: list[int],
        new_completed_waypoints: list[dict[str, Any]],
        *,
        formation_map: dict[int, int] | None = None,
    ) -> dict[str, Any]:
        mission_progress: dict[int, dict[str, Any]] = {}
        for mission_id, meta in self._mission_meta.items():
            state = self._progress_state.get(mission_id, MissionProgressState())
            planned = max(meta.planned_seconds, 0.0)
            path_actual = self._path_progress_seconds(meta, state, timestamp_ms)
            actual = path_actual
            actual_real = float(max(0.0, state.elapsed_seconds))
            covered_area_m2, planned_area_m2, coverage_percent, coverage_enabled = (
                self._coverage_metrics_for_mission(mission_id)
            )
            coverage_pass_metrics = self._coverage_pass_metrics_for_mission(mission_id)
            coverage_depth_metrics, coverage_observation_metrics = (
                self._coverage_depth_metrics_for_mission(mission_id)
            )
            coverage_depth_status_rows: list[dict[str, Any]] = []
            remaining_coverage_depth = 0
            completed_coverage_depth = 0
            if coverage_depth_metrics is not None:
                depth_area_tolerance = max(
                    0.05,
                    float(coverage_depth_metrics.assignment_area_m2) * 1e-6,
                )
                for depth in range(int(coverage_depth_metrics.required_depth) + 1):
                    depth_area_m2 = float(
                        coverage_depth_metrics.area_m2_by_exact_depth.get(depth, 0.0)
                    )
                    if depth_area_m2 <= depth_area_tolerance:
                        continue
                    missing_count = max(
                        0,
                        int(coverage_depth_metrics.required_depth) - int(depth),
                    )
                    remaining_coverage_depth = max(
                        int(remaining_coverage_depth), int(missing_count)
                    )
                    coverage_depth_status_rows.append(
                        {
                            "coverage_depth": int(depth),
                            "remaining_capture_count": int(missing_count),
                            "area_m2": round(depth_area_m2, 3),
                            "is_done": bool(missing_count == 0),
                        }
                    )
                completed_coverage_depth = max(
                    0,
                    int(coverage_depth_metrics.required_depth)
                    - int(remaining_coverage_depth),
                )
            (
                spatial_covered_area_m2,
                spatial_required_area_m2,
                spatial_coverage_percent,
                spatial_coverage_enabled,
            ) = self._spatial_coverage_metrics_for_mission(mission_id)
            coverage_remaining_area_m2, coverage_tolerance_m2, coverage_requirement_met = (
                _coverage_completion_metrics(covered_area_m2, planned_area_m2)
            )
            if coverage_pass_metrics:
                coverage_requirement_met = all(
                    bool(row.get("requirement_met"))
                    for row in coverage_pass_metrics
                )
            elif coverage_depth_metrics is not None:
                coverage_requirement_met = bool(coverage_depth_metrics.satisfied)
            line_covered_length_m, line_planned_length_m, line_percent, line_enabled = (
                self._line_sweep_metrics_for_mission(mission_id)
            )
            footprint_covered_area_m2 = covered_area_m2
            footprint_planned_area_m2 = planned_area_m2
            footprint_coverage_percent = coverage_percent
            footprint_coverage_enabled = coverage_enabled
            coverage_source = "footprint" if coverage_enabled else "none"
            coverage_unit = "m2" if coverage_enabled else None
            if line_enabled:
                covered_area_m2 = line_covered_length_m
                planned_area_m2 = line_planned_length_m
                coverage_percent = line_percent
                coverage_enabled = True
                coverage_source = "line_sweep"
                coverage_unit = "m"
            last_wp = int(meta.waypoint_ids[-1]) if meta.waypoint_ids else None
            if (
                not state.done
                and not state.awaiting_execute
                and not state.path_done
                and last_wp is not None
                and state.current_waypoint_id == last_wp
            ):
                lower = self._lower_bound_seconds(meta, last_wp)
                cap_seconds = lower
                if planned > 0:
                    cap_seconds = max(lower, planned * 0.95)
                if cap_seconds > 0:
                    path_actual = min(path_actual, cap_seconds)
                    actual = path_actual
            if planned > 0:
                path_actual = min(path_actual, planned)
                actual = min(actual, planned)
            # Stale-waypoint safeguard: after a "next collab base mission" trigger,
            # any individual mission that belongs to the newly activated input but
            # has not really started (real progress < 5%) should still visibly
            # advance so the UI doesn't look stuck. Climb 0% -> 5% over the first
            # 5 wall-clock seconds, then stop. ETA length is irrelevant: the
            # advance is purely time-driven, independent of `planned`.
            stale_wp_fallback_active = (
                planned > 0
                and not state.done
                and not state.awaiting_execute
                and not state.paused
                and self._forced_active_input_id is not None
                and meta.input_id is not None
                and int(meta.input_id) == int(self._forced_active_input_id)
            )
            fallback_percent_floor = 0
            if stale_wp_fallback_active:
                # Use wall-clock (monotonic) so the bar still climbs even when the
                # test feed reuses the same telemetry timestamp on every sample.
                now_monotonic = time.monotonic()
                baseline_monotonic = self._fallback_baseline_monotonic.get(int(mission_id))
                if baseline_monotonic is None or float(baseline_monotonic) > now_monotonic:
                    self._fallback_baseline_monotonic[int(mission_id)] = now_monotonic
                    baseline_monotonic = now_monotonic
                elapsed_fallback = max(0.0, now_monotonic - float(baseline_monotonic))
                # 1초당 1%p, 최대 5%. ETA 길이와 무관.
                fallback_percent_floor = int(min(5.0, elapsed_fallback))
                if fallback_percent_floor > 0:
                    floor_actual = planned * (fallback_percent_floor / 100.0)
                    if floor_actual > path_actual:
                        path_actual = floor_actual
                        actual = floor_actual
            else:
                # Once a mission no longer needs the fallback (it has real progress,
                # is done, or is no longer the active forced one) drop the baseline
                # so a re-activation later starts fresh.
                if int(mission_id) in self._fallback_baseline_monotonic:
                    self._fallback_baseline_monotonic.pop(int(mission_id), None)
                if int(mission_id) in self._fallback_baseline_ms:
                    self._fallback_baseline_ms.pop(int(mission_id), None)
            path_percent = self._calc_percent(
                path_actual,
                planned,
                bool(state.done or state.path_done),
                force_full=state.awaiting_execute,
            )
            sweep_percent = self._sweep_progress_percent(
                int(mission_id),
                path_percent=path_percent,
                line_percent=line_percent,
                line_enabled=line_enabled,
                coverage_percent=coverage_percent,
                coverage_enabled=coverage_enabled,
            )
            sweep_progress_points = max(0, int(state.sweep_progress_points or 0))
            sweep_point_count = max(0, int(meta.sweep_point_count or 0))
            sweep_point_percent: int | None = None
            if sweep_point_count > 0:
                if state.sweep_done or state.done:
                    sweep_progress_points = sweep_point_count
                sweep_progress_points = max(0, min(sweep_point_count, sweep_progress_points))
                sweep_point_percent = int(round((sweep_progress_points / max(1, sweep_point_count)) * 100.0))
                if sweep_progress_points > 0 and not (state.sweep_done or state.done):
                    sweep_percent = max(int(sweep_percent), min(int(sweep_point_percent), 99))
            if self._mission_requires_sweep(int(mission_id)):
                percent = min(int(path_percent), int(sweep_percent))
            else:
                percent = int(path_percent)
            if state.done or state.awaiting_execute:
                percent = 100
            percent = max(0, min(int(percent), 100))
            actual = planned * (percent / 100.0) if planned > 0 else actual
            # Force the percent floor regardless of how _calc_percent rounded the
            # ratio. For very long ETAs the planned-relative bump can round down
            # to 0; this guarantees the bar visibly moves 1%p per second.
            if stale_wp_fallback_active and fallback_percent_floor > percent:
                percent = fallback_percent_floor
                actual = planned * (percent / 100.0) if planned > 0 else actual
            # The individual mission progress bar in the UI reads `actual_seconds_real`
            # (from state.elapsed_seconds) for its suffix display while the bar value
            # itself uses `progress_percent`. Mirror our fallback floor into both so
            # the suffix at least keeps pace with `actual` during the fallback window.
            if stale_wp_fallback_active and actual_real < path_actual:
                actual_real = path_actual
            mission_progress[mission_id] = {
                "progress_percent": percent,
                "actual_seconds": int(round(actual)),
                "actual_seconds_real": int(round(actual_real)),
                "planned_seconds": int(round(planned)),
                "done": state.done,
                "awaiting_execute": bool(state.awaiting_execute),
                "path_progress_percent": int(path_percent),
                "waypoint_progress_percent": int(path_percent),
                "path_actual_seconds": int(round(path_actual)),
                "sweep_progress_percent": int(sweep_percent),
                "sweep_actual_seconds": int(round(planned * (sweep_percent / 100.0))) if planned > 0 else 0,
                "sweep_progress_points": int(sweep_progress_points),
                "sweep_point_count": int(sweep_point_count),
                "sweep_point_progress_percent": (
                    int(sweep_point_percent) if sweep_point_percent is not None else None
                ),
                "sweep_required": bool(self._mission_requires_sweep(int(mission_id))),
                "path_done": bool(state.path_done),
                "sweep_done": bool(state.sweep_done),
                "flying": state.flying_status,
                "filming": state.filming_status,
                "input_id": meta.input_id,
                "aircraft_id": meta.aircraft_id,
                "current_waypoint_id": state.current_waypoint_id,
                "waypoint_status": self._serialize_waypoint_status(mission_id, meta),
                "coverage_percent": coverage_percent,
                "covered_area_m2": round(covered_area_m2, 3),
                "planned_area_m2": round(planned_area_m2, 3),
                "coverage_enabled": bool(coverage_enabled),
                "coverage_source": coverage_source,
                "coverage_unit": coverage_unit,
                "coverage_pass_count": len(coverage_pass_metrics),
                "coverage_pass_policy": (
                    "all_passes_required" if coverage_pass_metrics else None
                ),
                "coverage_pass_details": coverage_pass_metrics,
                "coverage_pass_requirement_mode": (
                    "all_passes_required" if coverage_pass_metrics else None
                ),
                "coverage_depth_policy": (
                    "spatial_capture_depth" if coverage_depth_metrics is not None else None
                ),
                "required_coverage_depth": (
                    int(coverage_depth_metrics.required_depth)
                    if coverage_depth_metrics is not None
                    else 1
                ),
                "coverage_depth_satisfied": (
                    bool(coverage_depth_metrics.satisfied)
                    if coverage_depth_metrics is not None
                    else bool(coverage_requirement_met)
                ),
                "coverage_depth_area_m2": (
                    {
                        str(depth): round(float(area_m2), 3)
                        for depth, area_m2 in coverage_depth_metrics.area_m2_by_exact_depth.items()
                    }
                    if coverage_depth_metrics is not None
                    else {}
                ),
                "coverage_depth_details": coverage_depth_status_rows,
                "remaining_coverage_depth": int(remaining_coverage_depth),
                "completed_coverage_depth": int(completed_coverage_depth),
                "coverage_observation_details": coverage_observation_metrics,
                "coverage_work_covered_area_m2": round(footprint_covered_area_m2, 3),
                "coverage_work_required_area_m2": round(footprint_planned_area_m2, 3),
                "coverage_work_remaining_area_m2": round(coverage_remaining_area_m2, 3),
                "coverage_completion_tolerance_m2": round(coverage_tolerance_m2, 6),
                "coverage_requirement_met": bool(
                    coverage_requirement_met if footprint_coverage_enabled else False
                ),
                "spatial_coverage_percent": int(spatial_coverage_percent),
                "spatial_covered_area_m2": round(spatial_covered_area_m2, 3),
                "spatial_required_area_m2": round(spatial_required_area_m2, 3),
                "spatial_coverage_enabled": bool(spatial_coverage_enabled),
                "line_coverage_percent": line_percent,
                "line_covered_length_m": round(line_covered_length_m, 3),
                "line_planned_length_m": round(line_planned_length_m, 3),
                "line_coverage_enabled": bool(line_enabled),
                "footprint_coverage_percent": footprint_coverage_percent,
                "footprint_covered_area_m2": round(footprint_covered_area_m2, 3),
                "footprint_planned_area_m2": round(footprint_planned_area_m2, 3),
                "footprint_coverage_enabled": bool(footprint_coverage_enabled),
                "sweep_progress_source": (
                    "line_sweep"
                    if line_enabled
                    else "footprint"
                    if footprint_coverage_enabled
                    else "path_fallback"
                    if state.filming_status in (1, 2)
                    else "none"
                ),
            }

        if formation_map:
            for follower_id, leader_id in formation_map.items():
                leader_prog = mission_progress.get(int(leader_id))
                if not isinstance(leader_prog, dict):
                    continue
                follower_prog = mission_progress.get(int(follower_id))
                if not isinstance(follower_prog, dict):
                    follower_prog = {}
                    meta = self._mission_meta.get(int(follower_id))
                    if meta is not None:
                        follower_prog["input_id"] = meta.input_id
                        follower_prog["aircraft_id"] = meta.aircraft_id
                    mission_progress[int(follower_id)] = follower_prog
                for key in (
                    "progress_percent",
                    "actual_seconds",
                    "actual_seconds_real",
                    "planned_seconds",
                    "done",
                    "awaiting_execute",
                    "path_progress_percent",
                    "waypoint_progress_percent",
                    "path_actual_seconds",
                    "sweep_progress_percent",
                    "sweep_actual_seconds",
                    "sweep_required",
                    "path_done",
                    "sweep_done",
                    "flying",
                    "filming",
                    "waypoint_status",
                    "coverage_percent",
                    "covered_area_m2",
                    "planned_area_m2",
                    "coverage_enabled",
                    "coverage_source",
                    "coverage_unit",
                    "coverage_pass_count",
                    "coverage_pass_policy",
                    "coverage_pass_details",
                    "coverage_pass_requirement_mode",
                    "coverage_depth_policy",
                    "required_coverage_depth",
                    "coverage_depth_satisfied",
                    "coverage_depth_area_m2",
                    "coverage_depth_details",
                    "remaining_coverage_depth",
                    "completed_coverage_depth",
                    "coverage_observation_details",
                    "coverage_work_covered_area_m2",
                    "coverage_work_required_area_m2",
                    "coverage_work_remaining_area_m2",
                    "coverage_completion_tolerance_m2",
                    "coverage_requirement_met",
                    "spatial_coverage_percent",
                    "spatial_covered_area_m2",
                    "spatial_required_area_m2",
                    "spatial_coverage_enabled",
                    "line_coverage_percent",
                    "line_covered_length_m",
                    "line_planned_length_m",
                    "line_coverage_enabled",
                    "footprint_coverage_percent",
                    "footprint_covered_area_m2",
                    "footprint_planned_area_m2",
                    "footprint_coverage_enabled",
                    "sweep_progress_source",
                ):
                    if key in leader_prog:
                        follower_prog[key] = leader_prog[key]

        package_progress: dict[int, dict[str, Any]] = {}
        for aircraft_id, mission_ids in self._aircraft_missions.items():
            package_progress[aircraft_id] = self._aggregate_count_progress(
                mission_ids,
                mission_progress,
            )

        input_progress: dict[int, dict[str, Any]] = {}
        for input_id in self._input_mission_ids:
            mission_ids = self._input_to_missions.get(input_id, [])
            done_override = input_id in self._completed_input_ids
            input_progress[input_id] = self._aggregate_progress(
                mission_ids,
                mission_progress,
                done_override=done_override,
            )
        boundary_guard_progress = self._boundary_guard_gate.statuses()
        for input_id, guard_status in boundary_guard_progress.items():
            progress_row = input_progress.get(int(input_id))
            if progress_row is None:
                continue
            duration_s = max(0.001, float(guard_status.get("duration_s") or 0.0))
            elapsed_s = max(0.0, float(guard_status.get("elapsed_s") or 0.0))
            ready = bool(guard_status.get("ready"))
            guard_percent = (
                100
                if ready
                else min(99, int(round(min(elapsed_s, duration_s) / duration_s * 100.0)))
            )
            progress_row.update(
                {
                    "progress_percent": int(guard_percent),
                    "actual_seconds": int(round(min(elapsed_s, duration_s))),
                    "planned_seconds": int(round(duration_s)),
                    "done": bool(ready),
                    "boundary_guard": dict(guard_status),
                }
            )

        package_coverage: dict[int, dict[str, Any]] = {}
        package_footprint_coverage: dict[int, dict[str, Any]] = {}
        for aircraft_id, mission_ids in self._aircraft_missions.items():
            package_coverage[aircraft_id] = self._aggregate_coverage_progress(
                mission_ids,
                mission_progress,
            )
            package_footprint_coverage[aircraft_id] = self._aggregate_footprint_coverage_progress(
                mission_ids,
                mission_progress,
            )

        input_coverage: dict[int, dict[str, Any]] = {}
        input_footprint_coverage: dict[int, dict[str, Any]] = {}
        for input_id in self._input_mission_ids:
            mission_ids = self._input_to_missions.get(input_id, [])
            input_coverage[input_id] = self._aggregate_coverage_progress(
                mission_ids,
                mission_progress,
            )
            input_footprint_coverage[input_id] = self._aggregate_input_footprint_geometry(
                mission_ids,
            )

        for input_id, data in input_progress.items():
            if not data.get("done"):
                continue
            if input_id in self._completed_input_ids:
                continue
            self._completed_input_ids.add(input_id)
            new_completed_input.append(input_id)

        plan_progress = self._aggregate_count_progress(
            [mid for mid in self._input_mission_ids if mid in input_progress],
            input_progress,
        )
        plan_coverage = self._aggregate_coverage_progress(
            [mid for mid in self._input_mission_ids if mid in input_coverage],
            input_coverage,
            covered_key="covered_area_m2",
            planned_key="planned_area_m2",
            percent_key="coverage_percent",
            enabled_key="coverage_enabled",
        )
        plan_footprint_coverage = self._aggregate_footprint_coverage_progress(
            [mid for mid in self._input_mission_ids if mid in input_footprint_coverage],
            input_footprint_coverage,
            covered_key="covered_area_m2",
            planned_key="planned_area_m2",
            percent_key="coverage_percent",
            enabled_key="coverage_enabled",
        )

        return {
            "timestamp_ms": timestamp_ms,
            "mission_progress": mission_progress,
            "aircraft_current_mission": {
                int(aircraft_id): int(mission_id)
                for aircraft_id, mission_id in self._aircraft_current_mission.items()
                if mission_id is not None
            },
            "active_input_id": self.get_active_input_id(),
            "package_progress": package_progress,
            "input_progress": input_progress,
            "boundary_guard_progress": boundary_guard_progress,
            "plan_progress": plan_progress,
            "package_coverage": package_coverage,
            "input_coverage": input_coverage,
            "plan_coverage": plan_coverage,
            "package_footprint_coverage": package_footprint_coverage,
            "input_footprint_coverage": input_footprint_coverage,
            "plan_footprint_coverage": plan_footprint_coverage,
            "new_completed_individual": new_completed_individual,
            "new_completed_input": new_completed_input,
            "new_completed_waypoints": new_completed_waypoints,
        }

    def _record_waypoint_completion(
        self,
        mission_id: int,
        current_wp: int | None,
        prev_wp: int | None,
        on_mission: int | None,
        timestamp_ms: int | list[dict[str, Any]] | None,
        out_updates: list[dict[str, Any]] | None = None,
    ) -> None:
        # Backward compatibility: older call sites passed out_updates in the timestamp slot.
        if out_updates is None and isinstance(timestamp_ms, list):
            out_updates = timestamp_ms
            timestamp_ms = None
        if out_updates is None:
            return
        meta = self._mission_meta.get(mission_id)
        if meta is None or not meta.waypoint_ids:
            return
        current_idx = None
        if current_wp is not None and current_wp in meta.waypoint_index:
            current_idx = meta.waypoint_index.get(int(current_wp))
        last_completed = self._last_completed_idx.get(mission_id, -1)
        if on_mission == 2:
            target_completed = len(meta.waypoint_ids) - 1
        else:
            if current_idx is None:
                return
            target_completed = int(current_idx) - 1
        if target_completed <= last_completed:
            return
        if meta.path_id is None:
            return
        start_idx = max(0, last_completed + 1)
        end_idx = max(start_idx, target_completed + 1)
        completed_ids = meta.waypoint_ids[start_idx:end_idx]
        if not completed_ids:
            return
        state = self._progress_state.get(int(mission_id), MissionProgressState())
        actual_seconds = self._progress_seconds(meta, state, timestamp_ms)
        actual_real = max(0.0, float(state.elapsed_seconds))
        for wid in completed_ids:
            self._mark_waypoint_skipped(mission_id, int(wid))
            self._record_waypoint_actual(
                mission_id=int(mission_id),
                waypoint_id=int(wid),
                timestamp_ms=timestamp_ms,
                actual_seconds=actual_seconds,
                actual_real_seconds=actual_real,
            )
        out_updates.append(
            {
                "mission_id": mission_id,
                "path_id": meta.path_id,
                "waypoint_ids": completed_ids,
            }
        )
        self._last_completed_idx[mission_id] = target_completed

    def _record_waypoint_observation(
        self,
        *,
        mission_id: int,
        waypoint_id: int | None,
        timestamp_ms: int | None,
        actual_seconds: float,
        actual_real_seconds: float,
    ) -> None:
        if waypoint_id is None:
            return
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return
        wid = int(waypoint_id)
        if wid not in meta.waypoint_index:
            return
        state = self._waypoint_state.setdefault(
            int(mission_id),
            {int(v): "pending" for v in meta.waypoint_ids},
        )
        state[wid] = "reached"
        self._record_waypoint_actual(
            mission_id=int(mission_id),
            waypoint_id=wid,
            timestamp_ms=timestamp_ms,
            actual_seconds=actual_seconds,
            actual_real_seconds=actual_real_seconds,
        )

    def _mark_waypoint_skipped(self, mission_id: int, waypoint_id: int) -> None:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return
        wid = int(waypoint_id)
        if wid not in meta.waypoint_index:
            return
        state = self._waypoint_state.setdefault(
            int(mission_id),
            {int(v): "pending" for v in meta.waypoint_ids},
        )
        if state.get(wid) == "reached":
            return
        state[wid] = "skipped"

    def _record_waypoint_actual(
        self,
        *,
        mission_id: int,
        waypoint_id: int,
        timestamp_ms: int | None,
        actual_seconds: float,
        actual_real_seconds: float,
    ) -> None:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None or int(waypoint_id) not in meta.waypoint_index:
            return
        actual_map = self._waypoint_actual_seconds.setdefault(int(mission_id), {})
        if int(waypoint_id) in actual_map:
            return
        actual_map[int(waypoint_id)] = max(0.0, float(actual_seconds))
        self._waypoint_actual_real_seconds.setdefault(int(mission_id), {})[int(waypoint_id)] = max(
            0.0,
            float(actual_real_seconds),
        )
        self._waypoint_completion_ts_ms.setdefault(int(mission_id), {})[int(waypoint_id)] = (
            int(timestamp_ms) if timestamp_ms is not None else None
        )

    def _serialize_waypoint_status(
        self,
        mission_id: int,
        meta: MissionMeta,
    ) -> list[dict[str, Any]]:
        state = self._waypoint_state.setdefault(
            int(mission_id),
            {int(v): "pending" for v in meta.waypoint_ids},
        )
        actual_map = self._waypoint_actual_seconds.setdefault(int(mission_id), {})
        actual_real_map = self._waypoint_actual_real_seconds.setdefault(int(mission_id), {})
        completion_ts_map = self._waypoint_completion_ts_ms.setdefault(int(mission_id), {})
        items: list[dict[str, Any]] = []
        for wid in meta.waypoint_ids:
            status = str(state.get(int(wid)) or "pending")
            if status not in ("pending", "reached", "skipped"):
                status = "pending"
            planned_seconds = float(meta.waypoint_eta_cumulative.get(int(wid), 0.0))
            actual_seconds = actual_map.get(int(wid))
            actual_real_seconds = actual_real_map.get(int(wid))
            delta_seconds = None
            if actual_real_seconds is not None:
                delta_seconds = float(actual_real_seconds) - float(planned_seconds)
            items.append(
                {
                    "waypoint_id": int(wid),
                    "status": status,
                    "planned_seconds": int(round(planned_seconds)),
                    "actual_seconds": int(round(actual_seconds)) if actual_seconds is not None else None,
                    "actual_seconds_real": int(round(actual_real_seconds)) if actual_real_seconds is not None else None,
                    "delta_seconds": int(round(delta_seconds)) if delta_seconds is not None else None,
                    "completion_timestamp_ms": completion_ts_map.get(int(wid)),
                }
            )
        return items

    def _aggregate_progress(
        self,
        ids: list[int],
        progress_map: dict[int, dict[str, Any]],
        *,
        done_override: bool | None,
    ) -> dict[str, Any]:
        actual_total = 0.0
        planned_total = 0.0
        done_flags: list[bool] = []
        ready_flags: list[bool] = []
        for mid in ids:
            data = progress_map.get(mid)
            if not data:
                continue
            actual_total += float(data.get("actual_seconds") or 0)
            planned_total += float(data.get("planned_seconds") or 0)
            done_flags.append(bool(data.get("done")))
            ready_flags.append(bool(data.get("awaiting_execute")))
        if done_override is None:
            done = all(done_flags) if done_flags else False
        else:
            done = bool(done_override) or (all(done_flags) if done_flags else False)
        force_full = False
        if not done and done_flags:
            force_full = all(
                (done_flags[idx] or ready_flags[idx]) for idx in range(len(done_flags))
            )
        percent = self._calc_percent(actual_total, planned_total, done, force_full=force_full)
        return {
            "progress_percent": percent,
            "actual_seconds": int(round(actual_total)),
            "planned_seconds": int(round(planned_total)),
            "done": done,
        }

    @staticmethod
    def _aggregate_coverage_progress(
        ids: list[int],
        progress_map: dict[int, dict[str, Any]],
        *,
        covered_key: str = "covered_area_m2",
        planned_key: str = "planned_area_m2",
        percent_key: str = "coverage_percent",
        enabled_key: str = "coverage_enabled",
    ) -> dict[str, Any]:
        covered_total = 0.0
        planned_total = 0.0
        enabled = False
        for item_id in ids:
            data = progress_map.get(item_id)
            if not data:
                continue
            if not bool(data.get(enabled_key)):
                continue
            enabled = True
            covered_total += float(data.get(covered_key) or 0.0)
            planned_total += float(data.get(planned_key) or 0.0)
        if planned_total <= 0.0:
            percent = 0
            enabled = False
        else:
            percent = int(round((covered_total / planned_total) * 100))
            percent = max(0, min(percent, 100))
        return {
            percent_key: percent,
            covered_key: round(covered_total, 3),
            planned_key: round(planned_total, 3),
            enabled_key: bool(enabled),
        }

    @staticmethod
    def _aggregate_footprint_coverage_progress(
        ids: list[int],
        progress_map: dict[int, dict[str, Any]],
        *,
        covered_key: str = "footprint_covered_area_m2",
        planned_key: str = "footprint_planned_area_m2",
        percent_key: str = "footprint_coverage_percent",
        enabled_key: str = "footprint_coverage_enabled",
    ) -> dict[str, Any]:
        work_covered = 0.0
        work_required = 0.0
        spatial_covered = 0.0
        spatial_required = 0.0
        coverage_enabled = False
        requirement_rows: list[bool] = []
        pass_order: list[str] = []
        pass_totals: dict[str, dict[str, float | bool]] = {}
        for item_id in ids:
            data = progress_map.get(item_id)
            if not isinstance(data, dict) or not bool(data.get(enabled_key)):
                continue
            coverage_enabled = True
            spatial_covered += float(
                data.get("spatial_covered_area_m2", data.get(covered_key)) or 0.0
            )
            spatial_required += float(
                data.get("spatial_required_area_m2", data.get(planned_key)) or 0.0
            )
            work_covered += float(
                data.get("coverage_work_covered_area_m2", data.get(covered_key)) or 0.0
            )
            work_required += float(
                data.get("coverage_work_required_area_m2", data.get(planned_key)) or 0.0
            )
            if "coverage_requirement_met" in data:
                requirement_rows.append(bool(data.get("coverage_requirement_met")))
            for row in data.get("coverage_pass_details") or []:
                if not isinstance(row, dict):
                    continue
                pass_name = str(row.get("coverage_pass") or "").strip().lower()
                if not pass_name:
                    continue
                if pass_name not in pass_order:
                    pass_order.append(pass_name)
                bucket = pass_totals.setdefault(
                    pass_name,
                    {"covered": 0.0, "required": 0.0, "requirement_met": True},
                )
                bucket["covered"] = float(bucket["covered"]) + float(
                    row.get("actual_covered_area_m2", row.get("covered_area_m2")) or 0.0
                )
                bucket["required"] = float(bucket["required"]) + float(
                    row.get("required_area_m2", row.get("planned_area_m2")) or 0.0
                )
                bucket["requirement_met"] = bool(bucket["requirement_met"]) and bool(
                    row.get("requirement_met", row.get("is_done"))
                )

        pass_details: list[dict[str, Any]] = []
        for pass_index, pass_name in enumerate(pass_order, start=1):
            bucket = pass_totals[pass_name]
            covered = max(0.0, float(bucket["covered"]))
            required = max(0.0, float(bucket["required"]))
            remaining, tolerance, measured_done = _coverage_completion_metrics(covered, required)
            requirement_met = bool(bucket["requirement_met"]) and bool(measured_done)
            pass_details.append(
                {
                    "coverage_pass": pass_name,
                    "pass_index": pass_index,
                    "coverage_percent": (
                        max(0, min(100, int(round((covered / required) * 100.0))))
                        if required > 0.0
                        else 0
                    ),
                    "covered_area_m2": round(min(covered, required), 3),
                    "planned_area_m2": round(required, 3),
                    "actual_covered_area_m2": round(min(covered, required), 3),
                    "required_area_m2": round(required, 3),
                    "remaining_area_m2": round(remaining, 3),
                    "completion_tolerance_m2": round(tolerance, 6),
                    "requirement_met": requirement_met,
                    "is_done": requirement_met,
                    "status": "completed" if requirement_met else "partial" if covered > 0 else "planned",
                }
            )

        work_percent = (
            max(0, min(100, int(round((work_covered / work_required) * 100.0))))
            if work_required > 0.0
            else 0
        )
        work_remaining, work_tolerance, measured_work_done = _coverage_completion_metrics(
            work_covered,
            work_required,
        )
        requirements_met = bool(measured_work_done)
        if requirement_rows:
            requirements_met = requirements_met and all(requirement_rows)
        if pass_details:
            requirements_met = requirements_met and all(
                bool(row.get("requirement_met")) for row in pass_details
            )
        spatial_covered = max(0.0, min(spatial_covered, spatial_required))
        spatial_percent = (
            max(0, min(100, int(round((spatial_covered / spatial_required) * 100.0))))
            if spatial_required > 0.0
            else 0
        )
        return {
            "coverage_percent": int(work_percent),
            "covered_area_m2": round(spatial_covered, 3),
            "planned_area_m2": round(spatial_required, 3),
            "coverage_enabled": bool(coverage_enabled),
            "spatial_coverage_percent": int(spatial_percent),
            "spatial_covered_area_m2": round(spatial_covered, 3),
            "spatial_required_area_m2": round(spatial_required, 3),
            "coverage_work_covered_area_m2": round(min(work_covered, work_required), 3),
            "coverage_work_required_area_m2": round(work_required, 3),
            "coverage_work_remaining_area_m2": round(work_remaining, 3),
            "coverage_completion_tolerance_m2": round(work_tolerance, 6),
            "coverage_requirement_met": bool(requirements_met),
            "coverage_pass_count": len(pass_details),
            "coverage_pass_policy": "all_passes_required" if pass_details else None,
            "coverage_pass_details": pass_details,
        }

    def _aggregate_input_footprint_geometry(
        self,
        mission_ids: list[int],
    ) -> dict[str, Any]:
        planned_geometry: BaseGeometry | None = None
        single_required_geometry: BaseGeometry | None = None
        single_covered_geometry: BaseGeometry | None = None
        pass_order: list[str] = []
        pass_required_geometry: dict[str, BaseGeometry | None] = {}
        pass_covered_geometry: dict[str, BaseGeometry | None] = {}
        active_passes: set[str] = set()
        depth_ledger = SpatialCoverageDepthLedger(required_depth=2)
        depth_enabled = False
        for mission_id in mission_ids:
            definition = self._mission_coverage_defs.get(int(mission_id))
            if definition is None or definition.assignment_geometry.is_empty:
                continue
            planned_geometry = merge_coverage_geometry(
                planned_geometry,
                definition.assignment_geometry,
            )
            meta = self._mission_meta.get(int(mission_id))
            state = self._mission_coverage_state.get(int(mission_id))
            effective_required_depth = (
                max(
                    int(meta.coverage_required_depth),
                    len(tuple(meta.coverage_pass_order or ())),
                )
                if meta is not None
                else 1
            )
            if meta is not None and effective_required_depth > 1:
                depth_enabled = True
                depth_ledger.required_depth = max(
                    int(depth_ledger.required_depth),
                    int(effective_required_depth),
                )
                if state is not None:
                    for source_id, source_geometry in state.covered_geometry_by_source.items():
                        depth_ledger.add_observation(
                            source_id,
                            source_geometry,
                            attribution=state.coverage_source_attribution.get(str(source_id)),
                        )
                    if not state.covered_geometry_by_source:
                        for pass_name in tuple(meta.coverage_pass_order or ()):
                            pass_geometry = state.covered_geometry_by_pass.get(str(pass_name))
                            if pass_geometry is None or pass_geometry.is_empty:
                                continue
                            source_id = stable_capture_source_id(
                                aircraft_id=meta.aircraft_id,
                                coverage_pass=str(pass_name),
                                mission_id=meta.mission_id,
                                generation_token=meta.coverage_generation_token,
                            )
                            depth_ledger.add_observation(
                                source_id,
                                pass_geometry,
                                attribution={
                                    "aircraftID": int(meta.aircraft_id),
                                    "coveragePass": str(pass_name),
                                    "acquisitionID": source_id,
                                },
                            )
            mission_passes = tuple(meta.coverage_pass_order or ()) if meta is not None else ()
            # A pass-labelled mission is already unambiguous here: legacy
            # one-off markers are filtered out while MissionMeta is built.
            # Keep an explicit reverse-only resume in the pass buckets too;
            # treating it as an ordinary single-coverage mission drops the
            # RETURN identity from input-level monitoring/visualization.
            if mission_passes:
                progress_state = self._progress_state.get(int(mission_id))
                if progress_state is not None and progress_state.current_waypoint_id is not None:
                    active_name = (meta.coverage_pass_by_waypoint_id or {}).get(
                        int(progress_state.current_waypoint_id)
                    )
                    if active_name:
                        active_passes.add(str(active_name))
                for pass_name in mission_passes:
                    pass_name = str(pass_name)
                    if pass_name not in pass_order:
                        pass_order.append(pass_name)
                    pass_required_geometry[pass_name] = merge_coverage_geometry(
                        pass_required_geometry.get(pass_name),
                        definition.assignment_geometry,
                    )
                    pass_geometry = (
                        state.covered_geometry_by_pass.get(pass_name)
                        if state is not None
                        else None
                    )
                    if pass_geometry is not None and not pass_geometry.is_empty:
                        pass_covered_geometry[pass_name] = merge_coverage_geometry(
                            pass_covered_geometry.get(pass_name),
                            pass_geometry,
                        )
                continue
            single_required_geometry = merge_coverage_geometry(
                single_required_geometry,
                definition.assignment_geometry,
            )
            if state is not None and state.covered_geometry is not None and not state.covered_geometry.is_empty:
                single_covered_geometry = merge_coverage_geometry(
                    single_covered_geometry,
                    state.covered_geometry,
                )

        if planned_geometry is None or planned_geometry.is_empty:
            return {
                "coverage_percent": 0,
                "covered_area_m2": 0.0,
                "planned_area_m2": 0.0,
                "coverage_enabled": False,
            }
        planned_area = float(max(0.0, planned_geometry.area))
        if planned_area <= 0.0:
            return {
                "coverage_percent": 0,
                "covered_area_m2": 0.0,
                "planned_area_m2": 0.0,
                "coverage_enabled": False,
            }
        common_pass_covered: BaseGeometry | None = None
        for pass_name in pass_order:
            covered = pass_covered_geometry.get(pass_name)
            if covered is None or covered.is_empty:
                common_pass_covered = None
                break
            if common_pass_covered is None:
                common_pass_covered = covered
            else:
                try:
                    common_pass_covered = common_pass_covered.intersection(covered)
                except Exception:
                    common_pass_covered = None
                    break
        spatial_covered_geometry = merge_coverage_geometry(
            single_covered_geometry,
            common_pass_covered,
        )
        covered_area = 0.0
        if spatial_covered_geometry is not None and not spatial_covered_geometry.is_empty:
            try:
                covered_area = float(
                    max(
                        0.0,
                        min(
                            planned_area,
                            planned_geometry.intersection(spatial_covered_geometry).area,
                        ),
                    )
                )
            except Exception:
                covered_area = 0.0

        single_required_area = float(
            max(0.0, single_required_geometry.area)
            if single_required_geometry is not None and not single_required_geometry.is_empty
            else 0.0
        )
        single_covered_area = 0.0
        if single_required_geometry is not None and single_covered_geometry is not None:
            try:
                single_covered_area = float(
                    max(
                        0.0,
                        min(
                            single_required_area,
                            single_required_geometry.intersection(single_covered_geometry).area,
                        ),
                    )
                )
            except Exception:
                single_covered_area = 0.0
        _, _, single_requirement_met = _coverage_completion_metrics(
            single_covered_area,
            single_required_area,
        )
        if single_required_area <= 0.0:
            single_requirement_met = True

        pass_details: list[dict[str, Any]] = []
        work_required_area = single_required_area
        work_covered_area = single_covered_area
        for pass_index, pass_name in enumerate(pass_order, start=1):
            required_geometry = pass_required_geometry.get(pass_name)
            actual_geometry = pass_covered_geometry.get(pass_name)
            required_area = float(
                max(0.0, required_geometry.area)
                if required_geometry is not None and not required_geometry.is_empty
                else 0.0
            )
            actual_area = 0.0
            if required_geometry is not None and actual_geometry is not None:
                try:
                    actual_area = float(
                        max(
                            0.0,
                            min(required_area, required_geometry.intersection(actual_geometry).area),
                        )
                    )
                except Exception:
                    actual_area = 0.0
            remaining_area, tolerance_m2, requirement_met = _coverage_completion_metrics(
                actual_area,
                required_area,
            )
            percent = (
                max(0, min(100, int(round((actual_area / required_area) * 100.0))))
                if required_area > 0.0
                else 0
            )
            status = (
                "completed"
                if requirement_met
                else "active"
                if pass_name in active_passes
                else "partial"
                if actual_area > 0.0
                else "planned"
            )
            pass_details.append(
                {
                    "coverage_pass": pass_name,
                    "pass_index": pass_index,
                    "coverage_percent": percent,
                    "covered_area_m2": round(actual_area, 3),
                    "planned_area_m2": round(required_area, 3),
                    "actual_covered_area_m2": round(actual_area, 3),
                    "required_area_m2": round(required_area, 3),
                    "remaining_area_m2": round(remaining_area, 3),
                    "completion_tolerance_m2": round(tolerance_m2, 6),
                    "requirement_met": bool(requirement_met),
                    "is_done": bool(requirement_met),
                    "status": status,
                }
            )
            work_required_area += required_area
            work_covered_area += actual_area

        work_remaining_area, work_tolerance_m2, work_measured_done = (
            _coverage_completion_metrics(work_covered_area, work_required_area)
        )
        requirement_met = bool(single_requirement_met and work_measured_done)
        if pass_details:
            requirement_met = requirement_met and all(
                bool(row.get("requirement_met")) for row in pass_details
            )
        result = {
            "coverage_percent": (
                int(round((work_covered_area / work_required_area) * 100.0))
                if work_required_area > 0.0
                else 0
            ),
            "covered_area_m2": round(covered_area, 3),
            "planned_area_m2": round(planned_area, 3),
            "coverage_enabled": True,
            "spatial_coverage_percent": int(round((covered_area / planned_area) * 100.0)),
            "spatial_covered_area_m2": round(covered_area, 3),
            "spatial_required_area_m2": round(planned_area, 3),
            "coverage_work_covered_area_m2": round(work_covered_area, 3),
            "coverage_work_required_area_m2": round(work_required_area, 3),
            "coverage_work_remaining_area_m2": round(work_remaining_area, 3),
            "coverage_completion_tolerance_m2": round(work_tolerance_m2, 6),
            "coverage_requirement_met": bool(requirement_met),
            "coverage_pass_count": len(pass_details),
            "coverage_pass_policy": "all_passes_required" if pass_details else None,
            "coverage_pass_details": pass_details,
            "coverage_pass_requirement_mode": (
                "all_passes_required" if pass_details else None
            ),
        }
        if depth_enabled and not pass_details:
            depth_metrics = depth_ledger.metrics(planned_geometry)
            depth_remaining = 0
            depth_details: list[dict[str, Any]] = []
            for depth in range(int(depth_metrics.required_depth) + 1):
                area_m2 = float(depth_metrics.area_m2_by_exact_depth.get(depth, 0.0))
                if area_m2 <= max(0.05, planned_area * 1e-6):
                    continue
                missing_count = max(0, int(depth_metrics.required_depth) - int(depth))
                depth_remaining = max(depth_remaining, missing_count)
                depth_details.append(
                    {
                        "coverage_depth": int(depth),
                        "remaining_capture_count": int(missing_count),
                        "area_m2": round(area_m2, 3),
                        "is_done": bool(missing_count == 0),
                    }
                )
            completed_depth = max(
                0,
                int(depth_metrics.required_depth) - int(depth_remaining),
            )
            result.update(
                {
                    "coverage_percent": (
                        int(
                            round(
                                depth_metrics.work_covered_m2
                                / max(depth_metrics.work_required_m2, 1e-9)
                                * 100.0
                            )
                        )
                    ),
                    "covered_area_m2": round(
                        float(depth_metrics.completed_geometry.area or 0.0), 3
                    ),
                    "planned_area_m2": round(planned_area, 3),
                    "spatial_coverage_percent": int(
                        round(
                            float(depth_metrics.completed_geometry.area or 0.0)
                            / max(planned_area, 1e-9)
                            * 100.0
                        )
                    ),
                    "spatial_covered_area_m2": round(
                        float(depth_metrics.completed_geometry.area or 0.0), 3
                    ),
                    "spatial_required_area_m2": round(planned_area, 3),
                    "coverage_work_covered_area_m2": round(
                        depth_metrics.work_covered_m2, 3
                    ),
                    "coverage_work_required_area_m2": round(
                        depth_metrics.work_required_m2, 3
                    ),
                    "coverage_work_remaining_area_m2": round(
                        depth_metrics.work_remaining_m2, 3
                    ),
                    "coverage_requirement_met": bool(depth_metrics.satisfied),
                    "coverage_pass_requirement_mode": None,
                    "coverage_depth_policy": "spatial_capture_depth",
                    "required_coverage_depth": int(depth_metrics.required_depth),
                    "remaining_coverage_depth": int(depth_remaining),
                    "completed_coverage_depth": int(completed_depth),
                    "coverage_depth_satisfied": bool(depth_metrics.satisfied),
                    "coverage_depth_details": depth_details,
                    "coverage_observation_details": [
                        {
                            "acquisition_id": str(source_id),
                            "aircraft_id": _coerce_int(
                                (depth_ledger.attribution_by_source.get(source_id) or {}).get(
                                    "aircraftID"
                                )
                            ),
                            "coverage_pass": (
                                depth_ledger.attribution_by_source.get(source_id) or {}
                            ).get("coveragePass"),
                            "covered_area_m2": round(
                                float(source_geometry.area or 0.0), 3
                            ),
                        }
                        for source_id, source_geometry in sorted(
                            depth_ledger.observations_by_source.items()
                        )
                    ],
                }
            )
        return result

    @staticmethod
    def _aggregate_count_progress(
        ids: list[int],
        progress_map: dict[int, dict[str, Any]],
    ) -> dict[str, Any]:
        total = 0
        done_count = 0
        for mid in ids:
            data = progress_map.get(mid)
            if not data:
                continue
            total += 1
            if data.get("done"):
                done_count += 1
        if total <= 0:
            percent = 0
            done = False
        else:
            percent = int(round((done_count / total) * 100))
            done = done_count >= total
        return {
            "progress_percent": percent,
            "actual_seconds": done_count,
            "planned_seconds": total,
            "done": done,
        }

    @staticmethod
    def _calc_percent(actual: float, planned: float, done: bool, *, force_full: bool = False) -> int:
        if planned <= 0:
            return 100 if (done or force_full) else 0
        percent = int(round((actual / planned) * 100))
        if done or force_full:
            return 100
        return max(0, min(percent, 99))
