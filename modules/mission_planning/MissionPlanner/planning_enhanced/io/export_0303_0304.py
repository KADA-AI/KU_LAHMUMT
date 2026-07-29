from __future__ import annotations

import concurrent.futures
import json
from importlib import reload
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
from modules.mission_planning.pipelines.type2_boundary_guard_loop import (
    sync_boundary_guard_contract_from_flight_paths,
)
try:
    from ....runtime.json_io import dumps_json, prepare_json_payload
except Exception:
    try:
        from modules.mission_planning.runtime.json_io import dumps_json, prepare_json_payload  # type: ignore
    except Exception:
        dumps_json = None
        prepare_json_payload = None

try:
    from ...runtime_settings import (
        apply_runtime_camera_adjusted_fov_deg,
        load_runtime_settings,
        load_runtime_values,
        load_runtime_flyover,
        runtime_override,
        get_runtime_altitude_layers_m,
        get_runtime_float,
        get_runtime_int,
        get_runtime_str,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        apply_runtime_camera_adjusted_fov_deg,
        load_runtime_settings,
        load_runtime_values,
        load_runtime_flyover,
        runtime_override,
        get_runtime_altitude_layers_m,
        get_runtime_float,
        get_runtime_int,
        get_runtime_str,
    )


def _to_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default

def _import_runtime_modules():
    try:
        from ...data_def import d0303, d0304, search_speed  # type: ignore
        from ... import config as mp_config  # type: ignore
    except Exception:
        try:
            from modules.mission_planning.MissionPlanner.data_def import d0303, d0304, search_speed  # type: ignore
            from modules.mission_planning.MissionPlanner import config as mp_config  # type: ignore
        except Exception:
            from data_def import d0303, d0304, search_speed  # type: ignore
            import config as mp_config  # type: ignore
    return reload(d0303), reload(d0304), search_speed, mp_config


def _payload_with_uav_plan_mode(payload: Dict[str, Any], uav_plan_mode: Optional[str]) -> Dict[str, Any]:
    mode = str(uav_plan_mode or "").strip().lower()
    if mode not in {"dub_path"}:
        return dict(payload) if isinstance(payload, dict) else {}
    base = dict(payload) if isinstance(payload, dict) else {}
    values = dict(load_runtime_values(payload))
    values["uav_plan_mode"] = mode
    base["values"] = values
    return base


def _apply_runtime_params(
    d0303,
    d0304,
    search_speed_module,
    mp_config_module,
    payload: Optional[Dict[str, Any]] = None,
) -> tuple[float, float]:
    if not isinstance(payload, dict):
        payload = load_runtime_settings()
    values = load_runtime_values(payload)
    flyover = load_runtime_flyover(payload)

    def _get_float(key: str, default: float) -> float:
        try:
            return float(values.get(key, default))
        except Exception:
            return default

    def _get_int(key: str, default: int) -> int:
        try:
            return int(float(values.get(key, default)))
        except Exception:
            return default

    sweep_sep = _get_float("default_sweep_separation_m", float(getattr(mp_config_module, "DEFAULT_SWEEP_SEPARATION_M", 1000.0)))
    search_weight = _get_float("search_speed_weight", float(getattr(mp_config_module, "SEARCH_SPEED_WEIGHT", 1.0)))
    area_search_weight = _get_float("area_search_speed_weight", float(getattr(d0303, "AREA_SEARCH_SPEED_WEIGHT", 1.2)))
    db_fov_weight = _get_float("db_fov_weight", float(getattr(mp_config_module, "DB_FOV_WEIGHT", 1.0)))
    if db_fov_weight <= 0.0:
        db_fov_weight = 1.0
    line_fov_deg_raw = _get_float(
        "line_custom_fov_deg",
        float(getattr(d0303, "FOV_DEG", 2.4)),
    )
    area_custom_fov_deg_raw = _get_float(
        "area_custom_fov_deg",
        float(line_fov_deg_raw),
    )
    area_output_fov_scale = _get_float(
        "area_output_fov_scale",
        float(getattr(d0303, "AREA_OUTPUT_FOV_SCALE", 3.0)),
    )
    line_density_scale = _get_float(
        "line_density_scale",
        float(getattr(d0303, "LINE_SWEEP_DENSITY_SCALE", 1.18)),
    )
    area_density_scale = _get_float(
        "area_density_scale",
        float(getattr(d0303, "AREA_SWEEP_DENSITY_SCALE", 1.0)),
    )
    line_route_offset_scale = _get_float(
        "line_route_offset_scale",
        float(getattr(d0303, "LINE_ROUTE_OFFSET_SCALE", 1.0)),
    )
    area_route_offset_scale = _get_float(
        "area_route_offset_scale",
        float(getattr(d0303, "AREA_ROUTE_OFFSET_SCALE", 0.5)),
    )
    uav_wp_interval_m = _get_float(
        "uav_wp_interval_m",
        float(getattr(d0303, "SWEEP_ROUTE_WP_SPACING_M", 2000.0)),
    )
    area_wp_interval_m = _get_float(
        "area_wp_interval_m",
        float(getattr(d0303, "AREA_SWEEP_ROUTE_WP_SPACING_M", 1000.0)),
    )
    lah_wp_interval_m = _get_float(
        "lah_wp_interval_m",
        float(getattr(d0304, "WP_INTERVAL_M", 3000.0)),
    )
    dubins_turn_radius_m = _get_float(
        "dubins_turn_radius_m",
        float(getattr(d0303, "DUBINS_TURN_RADIUS_M", 450.0)),
    )
    altitude = _get_int("altitude_m", int(getattr(d0303, "Altitude", 1000)))
    altitude_layers_m = get_runtime_altitude_layers_m(payload)

    line_fov_deg = float(
        apply_runtime_camera_adjusted_fov_deg(
            line_fov_deg_raw,
            payload,
            context="MISSION_PLAN LINE",
        )
    )
    area_custom_fov_deg = float(
        apply_runtime_camera_adjusted_fov_deg(
            area_custom_fov_deg_raw,
            payload,
            context="MISSION_PLAN AREA",
        )
    )
    d0303.FOV_DEG = float(line_fov_deg)
    d0303.AREA_CUSTOM_FOV_DEG = float(area_custom_fov_deg)
    d0303.AREA_OUTPUT_FOV_SCALE = float(area_output_fov_scale)
    d0303.LINE_SWEEP_DENSITY_SCALE = float(line_density_scale)
    d0303.AREA_SWEEP_DENSITY_SCALE = float(area_density_scale)
    d0303.LINE_ROUTE_OFFSET_SCALE = float(line_route_offset_scale)
    d0303.AREA_ROUTE_OFFSET_SCALE = float(area_route_offset_scale)
    d0303.LINE_SEARCH_SPEED_WEIGHT = float(search_weight)
    d0303.AREA_SEARCH_SPEED_WEIGHT = float(area_search_weight)
    d0303.SWEEP_ROUTE_WP_SPACING_M = float(uav_wp_interval_m)
    d0303.AREA_SWEEP_ROUTE_WP_SPACING_M = float(area_wp_interval_m)
    d0303.DUBINS_TURN_RADIUS_M = float(dubins_turn_radius_m)
    d0304.WP_INTERVAL_M = float(lah_wp_interval_m)
    d0303.DB_FOV_WEIGHT = float(db_fov_weight)
    d0303.Altitude = int(round(altitude))
    d0303.ALTITUDE_LAYERS_M = altitude_layers_m
    d0304.ALTITUDE_LAYERS_M = altitude_layers_m
    d0303.AREA_FIRST_PACKET_SEARCH_SPEED_SCALE = _get_float(
        "area_first_packet_search_speed_scale",
        float(getattr(d0303, "AREA_FIRST_PACKET_SEARCH_SPEED_SCALE", 1.2)),
    )
    d0303.AREA_FIRST_PACKET_SWEEP_GROUP_SCALE = _get_float(
        "area_first_packet_sweep_group_scale",
        float(getattr(d0303, "AREA_FIRST_PACKET_SWEEP_GROUP_SCALE", 1.0)),
    )
    d0303.SWEEP_MERGE_HEADING_DEG = _get_float("sweep_merge_heading_deg", float(getattr(d0303, "SWEEP_MERGE_HEADING_DEG", 5.0)))
    d0303.SWEEP_LINE_INTERP_POINTS = _get_int("sweep_line_interp_points", int(getattr(d0303, "SWEEP_LINE_INTERP_POINTS", 3)))
    d0303.MAX_LINESEARCH_COORDS_PER_WAYPOINT = _get_int(
        "max_linesearch_coords_per_waypoint",
        int(getattr(d0303, "MAX_LINESEARCH_COORDS_PER_WAYPOINT", 2000)),
    )
    d0303.LINESEARCH_INNER_PARALLEL_MIN_STRIPS = _get_int(
        "linesearch_inner_parallel_min_strips",
        int(getattr(d0303, "LINESEARCH_INNER_PARALLEL_MIN_STRIPS", 256)),
    )
    d0303.LINESEARCH_INNER_PARALLEL_MIN_COORDS = _get_int(
        "linesearch_inner_parallel_min_coords",
        int(getattr(d0303, "LINESEARCH_INNER_PARALLEL_MIN_COORDS", 512)),
    )
    d0303.LINESEARCH_INNER_PARALLEL_WORKERS = _get_int(
        "linesearch_inner_parallel_workers",
        int(getattr(d0303, "LINESEARCH_INNER_PARALLEL_WORKERS", 2)),
    )
    d0303.FORMATION_FOLLOWER_POSTPROCESS_PARALLEL_MIN_FOLLOWERS = _get_int(
        "formation_follower_postprocess_parallel_min_followers",
        int(getattr(d0303, "FORMATION_FOLLOWER_POSTPROCESS_PARALLEL_MIN_FOLLOWERS", 2)),
    )
    d0303.FORMATION_FOLLOWER_POSTPROCESS_WORKERS = _get_int(
        "formation_follower_postprocess_workers",
        int(getattr(d0303, "FORMATION_FOLLOWER_POSTPROCESS_WORKERS", 2)),
    )
    d0303.MIN_SWEEP_LEN_M = _get_float("min_sweep_len_m", float(getattr(d0303, "MIN_SWEEP_LEN_M", 3.0)))
    d0303.MIN_ROUTE_SPACING_M = _get_float("min_route_spacing_m", float(getattr(d0303, "MIN_ROUTE_SPACING_M", 200.0)))
    d0303.AREA_DUBINS_ENTRY_LINKS_ENABLED = bool(
        values.get("area_dubins_entry_links_enabled", getattr(d0303, "AREA_DUBINS_ENTRY_LINKS_ENABLED", True))
    )
    d0303.DEFAULT_SEARCH_SPEED_MULTIPLIER = _get_float(
        "default_search_speed_multiplier",
        float(getattr(d0303, "DEFAULT_SEARCH_SPEED_MULTIPLIER", 16.0)),
    )
    d0303.POINT_FOV_DEG = apply_runtime_camera_adjusted_fov_deg(
        _get_float("point_fov_deg", float(getattr(d0303, "POINT_FOV_DEG", 31.2))),
        payload,
        context="MISSION_PLAN POINT",
    )
    d0303.AREA_NADIR_FOV_DEG = apply_runtime_camera_adjusted_fov_deg(
        _get_float("area_nadir_fov_deg", float(getattr(d0303, "AREA_NADIR_FOV_DEG", 31.2))),
        payload,
        context="MISSION_PLAN AREA_NADIR",
    )
    d0303.ENTRY_HOLD_FOV_DEG = apply_runtime_camera_adjusted_fov_deg(
        _get_float("entry_hold_fov_deg", float(getattr(d0303, "ENTRY_HOLD_FOV_DEG", 10.0))),
        payload,
        context="MISSION_PLAN ENTRY_HOLD",
    )
    d0303.ENTRY_HOLD_GIMBAL_PITCH = _get_float(
        "entry_hold_gimbal_pitch",
        float(getattr(d0303, "ENTRY_HOLD_GIMBAL_PITCH", -90.0)),
    )
    d0303.ENTRY_HOLD_GIMBAL_YAW = _get_float(
        "entry_hold_gimbal_yaw",
        float(getattr(d0303, "ENTRY_HOLD_GIMBAL_YAW", 0.0)),
    )
    d0303.LOITER_RADIUS_M = _get_float("loiter_radius_m", float(getattr(d0303, "LOITER_RADIUS_M", 800.0)))
    d0303.LOITER_DIRECTION = _get_int("loiter_direction", int(getattr(d0303, "LOITER_DIRECTION", 1)))
    d0303.LOITER_TIME_S = _get_float("loiter_time_s", float(getattr(d0303, "LOITER_TIME_S", 30.0)))
    d0303.LOITER_SPEED_MPS = _get_float("loiter_speed_mps", float(getattr(d0303, "LOITER_SPEED_MPS", 30.0)))
    d0303.SWEEP_GEOMETRY = d0303.SweepConfig(
        separation_m=float(sweep_sep),
        fov_deg=float(line_fov_deg),
    )

    if mp_config_module is not None:
        mp_config_module.DEFAULT_SWEEP_SEPARATION_M = float(sweep_sep)
        mp_config_module.SEARCH_SPEED_WEIGHT = float(search_weight)
        mp_config_module.DB_FOV_WEIGHT = float(db_fov_weight)
    if search_speed_module is not None:
        search_speed_module._CFG_WEIGHT = float(search_weight)

    d0303.set_flyover_options(
        entry_offset=bool(flyover.get("entry_offset", False)),
        dubins_prefix=bool(flyover.get("dubins_prefix", False)),
        last_point=bool(flyover.get("last_point", False)),
        all_wps=bool(flyover.get("all_wps", False)),
    )
    return _get_float("cruise_speed_mps", 40.0), _get_float("turn_step_deg", 15.0)


def _missions_from_0302_packages(
    packages: Iterable[Dict[str, Any]],
    *,
    min_aircraft_id: int,
    max_aircraft_id: int,
) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for pkg in packages:
        if not isinstance(pkg, dict):
            continue
        aid = _to_int(pkg.get("aircraftID"), 0)
        if aid < int(min_aircraft_id) or aid > int(max_aircraft_id):
            continue
        im_list = pkg.get("individualMissionList")
        if not isinstance(im_list, list):
            continue
        for im in im_list:
            if not isinstance(im, dict):
                continue
            cp = dict(im)
            cp["aircraftID"] = int(aid)
            out.append(cp)
    return out


def build_0303_0304_from_0302_packages(
    packages: List[Dict[str, Any]],
    *,
    mrpk: Optional[Dict[str, Any]] = None,
    planning_mode: object | None = None,
    cruise_speed_mps: Optional[float] = None,
    lah_cruise_speed_mps: Optional[float] = None,
    uav_plan_mode: Optional[str] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    d0303, d0304, search_speed, mp_config = _import_runtime_modules()
    runtime_payload = load_runtime_settings()
    uav_runtime_payload = _payload_with_uav_plan_mode(runtime_payload, uav_plan_mode)
    cfg_cruise_speed, cfg_turn_step = _apply_runtime_params(
        d0303,
        d0304,
        search_speed,
        mp_config,
        runtime_payload,
    )
    manned_plan_mode = str(get_runtime_str("manned_plan_mode", "normal", runtime_payload) or "normal").strip().lower()
    effective_uav_cruise = float(cruise_speed_mps) if cruise_speed_mps and float(cruise_speed_mps) > 0.0 else float(cfg_cruise_speed)
    effective_lah_cruise = float(lah_cruise_speed_mps) if lah_cruise_speed_mps and float(lah_cruise_speed_mps) > 0.0 else 40.0

    uav_missions = _missions_from_0302_packages(packages, min_aircraft_id=4, max_aircraft_id=6)
    lah_missions = _missions_from_0302_packages(packages, min_aircraft_id=1, max_aircraft_id=3)

    flight_plans_0303: List[Dict[str, Any]] = []
    flight_plans_0304: List[Dict[str, Any]] = []

    if isinstance(planning_mode, dict):
        planning_package_type = _to_int(
            planning_mode.get("package_type", planning_mode.get("inputMissionPackageType")),
            0,
        )
    else:
        planning_package_type = _to_int(getattr(planning_mode, "package_type", 0), 0)

    def _build_0303() -> List[Dict[str, Any]]:
        with runtime_override(uav_runtime_payload):
            return d0303.build_flight_plans(
                missions=uav_missions,
                cruise_speed=float(effective_uav_cruise),
                turn_step_deg=float(cfg_turn_step),
                ref0203=mrpk if isinstance(mrpk, dict) else None,
            )

    def _build_0304() -> List[Dict[str, Any]]:
        initial_hold_by_aircraft = (
            d0304.lah_initial_hold_by_aircraft_from_mrpk(mrpk)
            if planning_package_type in d0304.lah_initial_to_start_package_types()
            else {}
        )
        return d0304.build_lah_flight_plans_fixed(
            lah_missions,
            cruise_speed=float(effective_lah_cruise),
            manned_plan_mode=manned_plan_mode,
            initial_hold_by_aircraft=initial_hold_by_aircraft,
            runtime_payload=runtime_payload,
        )

    if uav_missions and lah_missions:
        with concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="Build03x") as executor:
            future_0303 = executor.submit(_build_0303)
            future_0304 = executor.submit(_build_0304)
            flight_plans_0303 = future_0303.result()
            flight_plans_0304 = future_0304.result()
    else:
        if uav_missions:
            flight_plans_0303 = _build_0303()
        if lah_missions:
            flight_plans_0304 = _build_0304()

    if flight_plans_0303 and flight_plans_0304:
        flight_plans_0304 = d0304.apply_uav_eta_follow_speed_plan(
            list(flight_plans_0304),
            list(flight_plans_0303),
            lah_missions=list(lah_missions),
        )

    package_missions = [
        mission
        for package in packages
        if isinstance(package, dict)
        for mission in (package.get("individualMissionList") or [])
        if isinstance(mission, dict)
    ]
    sync_boundary_guard_contract_from_flight_paths(
        package_missions,
        flight_plans_0303,
    )

    return list(flight_plans_0303), list(flight_plans_0304)


def save_0303_plans(plans: List[Dict[str, Any]], out_dir: str | Path) -> List[Path]:
    root = Path(out_dir)
    root.mkdir(parents=True, exist_ok=True)
    rows: List[tuple[Path, Dict[str, Any]]] = []
    for idx, row in enumerate(plans, start=1):
        aid = _to_int(row.get("aircraftID"), 0)
        pid = _to_int(row.get("pathID"), 0)
        if aid > 0 and pid > 0:
            name = f"FlightPath_UAV{aid}_Path{pid}.json"
        else:
            name = f"FlightPath_{idx:03d}.json"
        rows.append((root / name, row))
    _write_plan_rows(rows)
    written = [path for path, _ in rows]
    return written


def save_0304_plans(plans: List[Dict[str, Any]], out_dir: str | Path) -> List[Path]:
    root = Path(out_dir)
    root.mkdir(parents=True, exist_ok=True)
    rows: List[tuple[Path, Dict[str, Any]]] = []
    for idx, row in enumerate(plans, start=1):
        aid = _to_int(row.get("aircraftID"), 0)
        pid = _to_int(row.get("pathID"), 0)
        if aid > 0 and pid > 0:
            name = f"FlightPath_LAH{aid:03d}_Path{pid}.json"
        else:
            name = f"FlightPath_LAH_{idx:03d}.json"
        rows.append((root / name, row))
    _write_plan_rows(rows)
    written = [path for path, _ in rows]
    return written


def _write_plan_rows(rows: List[tuple[Path, Dict[str, Any]]]) -> None:
    if not rows:
        return

    def _write_one(item: tuple[Path, Dict[str, Any]]) -> None:
        path, payload = item
        prepared = prepare_json_payload(path, payload) if prepare_json_payload is not None else payload
        if dumps_json is not None:
            path.write_bytes(dumps_json(prepared, pretty=True, ensure_ascii=False))
        else:
            path.write_text(json.dumps(prepared, ensure_ascii=False, indent=2), encoding="utf-8")

    if len(rows) == 1:
        _write_one(rows[0])
        return

    with concurrent.futures.ThreadPoolExecutor(max_workers=min(8, len(rows)), thread_name_prefix="Save03x") as executor:
        futures = [executor.submit(_write_one, row) for row in rows]
        for future in concurrent.futures.as_completed(futures):
            future.result()
