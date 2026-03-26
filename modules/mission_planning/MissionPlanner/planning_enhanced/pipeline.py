from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set

from modules.common import db_paths
try:
    from ..runtime_settings import (
        load_runtime_settings,
        get_runtime_bool,
        get_runtime_float,
        get_runtime_area_review_max_segment_m,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        load_runtime_settings,
        get_runtime_bool,
        get_runtime_float,
        get_runtime_area_review_max_segment_m,
    )

from .assignment import resolve_uav_ids
from .algo import run_split_pipeline, review_overflow_areas
from .io import build_0302_packages_from_split_with_lah, save_0302_packages
from .pathing import calculate_expected_velocity, generate_expected_paths
from .type_decider import PROFILE_DEFAULT, apply_logic_type_decider


def _to_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _load_vehicle_status_available(cmpk_path: str | Path | None = None) -> Optional[Set[int]]:
    candidate_paths: List[Path] = []
    if cmpk_path:
        try:
            cmpk = Path(cmpk_path)
            candidate_paths.append(cmpk.parent.parent / "VehicleStatus" / "status.json")
        except Exception:
            pass
    try:
        candidate_paths.append(db_paths.get_db_subpath("VehicleStatus", "status.json"))
    except Exception:
        pass

    status_path = next((p for p in candidate_paths if isinstance(p, Path) and p.exists()), None)
    if status_path is None:
        return None
    try:
        payload = json.loads(status_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    raw = payload.get("available")
    if not isinstance(raw, list):
        return None
    available: Set[int] = set()
    for item in raw:
        try:
            available.add(int(item))
        except Exception:
            continue
    return available


def _extract_aircraft_id(entry: Any) -> Optional[int]:
    if entry is None:
        return None
    if isinstance(entry, dict):
        for key in ("aircraftID", "aircraftId", "id"):
            if key in entry:
                entry = entry.get(key)
                break
    try:
        return int(entry)
    except Exception:
        return None


def _apply_vehicle_status_filter(cmpk: Dict[str, Any], log: Callable[[str], None], *, cmpk_path: str | Path | None = None) -> None:
    status_available = _load_vehicle_status_available(cmpk_path)
    if status_available is None:
        return
    raw = cmpk.get("availableAircraftList")
    if not isinstance(raw, list):
        return
    filtered = []
    removed: Set[int] = set()
    for item in raw:
        aid = _extract_aircraft_id(item)
        if aid is None:
            filtered.append(item)
            continue
        if aid in status_available:
            filtered.append(item)
        else:
            removed.add(aid)
    cmpk["availableAircraftList"] = filtered
    if removed:
        log(f"[ENHANCED] VehicleStatus filter removed aircraft {sorted(removed)}")

def _load_runtime_settings() -> Dict[str, Any]:
    return load_runtime_settings()


def _settings_bool(payload: Dict[str, Any], key: str, default: bool) -> bool:
    return get_runtime_bool(key, default, payload)


def _settings_float(payload: Dict[str, Any], key: str, default: float) -> float:
    return get_runtime_float(key, default, payload)


def _settings_area_mode(payload: Dict[str, Any]) -> str:
    try:
        raw = str((payload.get("values") or {}).get("area_sweep_mode", "vertical") or "vertical").strip().lower()
    except Exception:
        raw = "vertical"
    if raw in {"vertical", "ver", "perpendicular", "orthogonal"}:
        return "vertical"
    if raw in {"nadir", "directdown", "bf_nadir"}:
        return "nadir"
    return "parallel"


def _profile_code_from_option(option_code: Optional[int]) -> int:
    _ = option_code
    return PROFILE_DEFAULT


def _is_single_point_coordinate_only_mission(mission: Any) -> bool:
    if not isinstance(mission, dict):
        return False
    detail = mission.get("missionDetail")
    if not isinstance(detail, dict):
        return False
    line_list = detail.get("lineList")
    area_list = detail.get("areaList")
    if isinstance(line_list, list) and line_list:
        return False
    if isinstance(area_list, list) and area_list:
        return False
    coord_list = detail.get("coordinateList")
    return isinstance(coord_list, list) and len(coord_list) == 1


def _filter_unplannable_coordinate_only_missions(
    cmpk: Dict[str, Any],
    log: Callable[[str], None],
) -> None:
    missions = cmpk.get("inputMissionList")
    if not isinstance(missions, list):
        return

    filtered: List[Any] = []
    skipped_ids: List[int] = []
    for idx, mission in enumerate(missions, start=1):
        if _is_single_point_coordinate_only_mission(mission):
            mission_id = _to_int(
                mission.get("inputMissionID") if isinstance(mission, dict) else None,
                idx,
            )
            skipped_ids.append(int(mission_id))
            continue
        filtered.append(mission)

    if not skipped_ids:
        return

    cmpk["inputMissionList"] = filtered
    log(
        "[ENHANCED] skipped single-point coordinate-only missions: "
        f"{sorted(skipped_ids)}"
    )
    if not filtered:
        raise RuntimeError(
            "No plannable mission remains after skipping single-point coordinate-only missions."
        )


def run_enhanced_divide_and_pattern(
    cmpk_path: str,
    ref_path: str,
    out_dir: str,
    log: Callable[[str], None] = print,
    option_code: int | None = None,
) -> List[str]:
    t0 = time.perf_counter()
    cmpk = json.loads(Path(cmpk_path).read_text(encoding="utf-8"))
    mrpk = json.loads(Path(ref_path).read_text(encoding="utf-8"))
    _apply_vehicle_status_filter(cmpk, log, cmpk_path=cmpk_path)
    _filter_unplannable_coordinate_only_missions(cmpk, log)

    uav_ids = resolve_uav_ids(cmpk)
    if not uav_ids:
        raise RuntimeError("No UAV available for mission planning.")
    log(f"[ENHANCED][1] split start: uavs={uav_ids}")
    split_result = run_split_pipeline(
        cmpk,
        mrpk,
        uav_ids,
        apply_assignment=True,
        apply_scheduling=True,
    )
    log(f"[ENHANCED][1] split done: pieces={len(split_result.pieces)}")

    profile_code = _profile_code_from_option(option_code)
    type_report = apply_logic_type_decider(split_result, cmpk, profile_code=profile_code)
    log(
        "[ENHANCED][2] type-decider done: "
        f"profile={profile_code} changed={int(type_report.get('changedPieces', 0))}/"
        f"{int(type_report.get('pieceCount', 0))}"
    )

    expected_paths = generate_expected_paths(split_result, mrpk)
    split_result.expected_paths = expected_paths
    log(f"[ENHANCED][3] expected-path done: count={len(expected_paths)}")

    runtime_cfg = _load_runtime_settings()
    review_enabled = _settings_bool(runtime_cfg, "enhanced_area_review_enabled", True)
    review_max_segment_m = get_runtime_area_review_max_segment_m(3000.0, runtime_cfg)
    if _settings_area_mode(runtime_cfg) == "nadir":
        review_enabled = False
    if review_enabled:
        review_report = review_overflow_areas(
            split_result,
            expected_paths,
            max_segment_m=review_max_segment_m,
        )
        line_paths = [
            row for row in expected_paths
            if isinstance(row, dict) and str(row.get("source", "")).startswith("line_center_offset_dir")
        ]
        split_result.expected_paths = line_paths
        log(
            "[ENHANCED][4] area-review done: "
            f"overflow={int(review_report.get('overflowRows', 0))} "
            f"targets={int(review_report.get('targets', 0))} "
            f"pieces={int(review_report.get('oldPieceCount', 0))}->{int(review_report.get('newPieceCount', 0))}"
        )
    else:
        log("[ENHANCED][4] area-review skipped by config")

    vel_report = calculate_expected_velocity(
        split_result,
        expected_paths=split_result.expected_paths,
    )
    log(
        "[ENHANCED][5] expected-velocity done: "
        f"pieces={int(vel_report.get('pieceCount', 0))} dbRows={int(vel_report.get('dbRowCount', 0))}"
    )

    packages = build_0302_packages_from_split_with_lah(split_result, cmpk=cmpk, source="MMR")
    out_root = Path(out_dir)
    out_root.mkdir(parents=True, exist_ok=True)
    paths = save_0302_packages(packages, out_root)
    log(f"[ENHANCED][6] 0302 export done: files={len(paths)}")
    log(f"[ENHANCED] total={((time.perf_counter() - t0) * 1000.0):.1f} ms")
    return [str(p) for p in paths]
