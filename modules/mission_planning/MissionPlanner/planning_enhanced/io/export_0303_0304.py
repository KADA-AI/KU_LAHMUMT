from __future__ import annotations

import concurrent.futures
import json
from importlib import reload
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
try:
    from ....runtime.json_io import dumps_json
except Exception:
    try:
        from modules.mission_planning.runtime.json_io import dumps_json  # type: ignore
    except Exception:
        dumps_json = None

try:
    from ...runtime_settings import (
        load_runtime_settings,
        load_runtime_values,
        load_runtime_flyover,
        get_runtime_float,
        get_runtime_int,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        load_runtime_settings,
        load_runtime_values,
        load_runtime_flyover,
        get_runtime_float,
        get_runtime_int,
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
        from data_def import d0303, d0304, search_speed  # type: ignore
        import config as mp_config  # type: ignore
    except Exception:
        from ...data_def import d0303, d0304, search_speed  # type: ignore
        from ... import config as mp_config  # type: ignore
    return reload(d0303), reload(d0304), search_speed, mp_config


def _apply_runtime_params(d0303, search_speed_module, mp_config_module) -> tuple[float, float]:
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

    sweep_sep = _get_float("default_sweep_separation_m", float(getattr(mp_config_module, "DEFAULT_SWEEP_SEPARATION_M", 600.0)))
    search_weight = _get_float("search_speed_weight", float(getattr(mp_config_module, "SEARCH_SPEED_WEIGHT", 1.0)))
    fov_deg = _get_float("fov_deg", float(getattr(d0303, "FOV_DEG", 2.4)))
    altitude = _get_int("altitude_m", int(getattr(d0303, "Altitude", 610)))

    d0303.FOV_DEG = float(fov_deg)
    d0303.Altitude = int(round(altitude))
    d0303.SWEEP_ENTRY_OFFSET_M = _get_float("sweep_entry_offset_m", float(getattr(d0303, "SWEEP_ENTRY_OFFSET_M", 500.0)))
    d0303.SWEEP_MERGE_HEADING_DEG = _get_float("sweep_merge_heading_deg", float(getattr(d0303, "SWEEP_MERGE_HEADING_DEG", 5.0)))
    d0303.SWEEP_LINE_INTERP_POINTS = _get_int("sweep_line_interp_points", int(getattr(d0303, "SWEEP_LINE_INTERP_POINTS", 3)))
    d0303.MIN_SWEEP_LEN_M = _get_float("min_sweep_len_m", float(getattr(d0303, "MIN_SWEEP_LEN_M", 3.0)))
    d0303.MIN_ROUTE_SPACING_M = _get_float("min_route_spacing_m", float(getattr(d0303, "MIN_ROUTE_SPACING_M", 200.0)))
    d0303.DEFAULT_SEARCH_SPEED_MULTIPLIER = _get_float(
        "default_search_speed_multiplier",
        float(getattr(d0303, "DEFAULT_SEARCH_SPEED_MULTIPLIER", 16.0)),
    )
    d0303.POINT_FOV_DEG = _get_float("point_fov_deg", float(getattr(d0303, "POINT_FOV_DEG", 66.638654)))
    d0303.AREA_NADIR_FOV_DEG = _get_float("area_nadir_fov_deg", float(getattr(d0303, "AREA_NADIR_FOV_DEG", 31.2)))
    d0303.ENTRY_HOLD_FOV_DEG = _get_float("entry_hold_fov_deg", float(getattr(d0303, "ENTRY_HOLD_FOV_DEG", 10.0)))
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
        fov_deg=float(fov_deg),
    )

    if mp_config_module is not None:
        mp_config_module.DEFAULT_SWEEP_SEPARATION_M = float(sweep_sep)
        mp_config_module.SEARCH_SPEED_WEIGHT = float(search_weight)
    if search_speed_module is not None:
        search_speed_module._CFG_WEIGHT = float(search_weight)

    algo_map = {"dtatrim": "dtatrim", "algo2": "linear", "algo3": "algo3"}
    algo_name = algo_map.get(str(payload.get("algo_key") or ""))
    if algo_name:
        d0303.set_route_planner(algo_name)
    d0303.set_flyover_options(
        entry_offset=bool(flyover.get("entry_offset", False)),
        dubins_prefix=bool(flyover.get("dubins_prefix", False)),
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
    cruise_speed_mps: Optional[float] = None,
    lah_cruise_speed_mps: Optional[float] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    d0303, d0304, search_speed, mp_config = _import_runtime_modules()
    cfg_cruise_speed, cfg_turn_step = _apply_runtime_params(d0303, search_speed, mp_config)
    effective_uav_cruise = float(cruise_speed_mps) if cruise_speed_mps and float(cruise_speed_mps) > 0.0 else float(cfg_cruise_speed)
    effective_lah_cruise = float(lah_cruise_speed_mps) if lah_cruise_speed_mps and float(lah_cruise_speed_mps) > 0.0 else 15.0

    uav_missions = _missions_from_0302_packages(packages, min_aircraft_id=4, max_aircraft_id=6)
    lah_missions = _missions_from_0302_packages(packages, min_aircraft_id=1, max_aircraft_id=3)

    flight_plans_0303: List[Dict[str, Any]] = []
    flight_plans_0304: List[Dict[str, Any]] = []

    def _build_0303() -> List[Dict[str, Any]]:
        return d0303.build_flight_plans(
            missions=uav_missions,
            cruise_speed=float(effective_uav_cruise),
            turn_step_deg=float(cfg_turn_step),
            ref0203=mrpk if isinstance(mrpk, dict) else None,
        )

    def _build_0304() -> List[Dict[str, Any]]:
        return d0304.build_lah_flight_plans_fixed(
            lah_missions,
            cruise_speed=float(effective_lah_cruise),
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
        if dumps_json is not None:
            path.write_bytes(dumps_json(payload, pretty=True, ensure_ascii=False))
        else:
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    if len(rows) == 1:
        _write_one(rows[0])
        return

    with concurrent.futures.ThreadPoolExecutor(max_workers=min(8, len(rows)), thread_name_prefix="Save03x") as executor:
        futures = [executor.submit(_write_one, row) for row in rows]
        for future in concurrent.futures.as_completed(futures):
            future.result()
