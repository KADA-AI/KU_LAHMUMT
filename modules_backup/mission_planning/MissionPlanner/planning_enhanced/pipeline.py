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
        get_runtime_int,
        get_runtime_area_review_max_segment_m,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        load_runtime_settings,
        get_runtime_bool,
        get_runtime_float,
        get_runtime_int,
        get_runtime_area_review_max_segment_m,
    )

from .assignment import resolve_uav_ids
from .algo import run_split_pipeline, review_assigned_areas_local, review_overflow_areas
from .io import build_0302_packages_from_split_with_lah, save_0302_packages
from .pathing import calculate_expected_velocity, generate_expected_paths
from .type_decider import PROFILE_DEFAULT, PROFILE_MIN_TIME, PROFILE_RECON, apply_logic_type_decider


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


def _count_area_pieces(split_result: Any) -> int:
    pieces = getattr(split_result, "pieces", None)
    if not isinstance(pieces, list):
        return 0
    return sum(1 for piece in pieces if _to_int(getattr(piece, "mission_type", 0)) in {2, 3, 6})


def _review_detail_stats(report: Dict[str, Any]) -> Dict[str, Any]:
    details = report.get("details")
    if not isinstance(details, list):
        details = []
    changed_details = 0
    split_count_sum = 0
    raw_split_count_sum = 0
    max_split_count = 0
    max_raw_split_count = 0
    split_capped_count = 0
    max_projected_span_m = 0.0
    max_total_len_m = 0.0
    max_segment_len_m = 0.0
    reason_counts: Dict[str, int] = {}
    for row in details:
        if not isinstance(row, dict):
            continue
        if bool(row.get("changed")):
            changed_details += 1
        reason = str(row.get("reason") or "").strip()
        if reason:
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
        split_count = _to_int(row.get("splitCount"), 0)
        raw_split_count = _to_int(row.get("rawSplitCount"), split_count)
        split_count_sum += max(0, split_count)
        raw_split_count_sum += max(0, raw_split_count)
        max_split_count = max(max_split_count, split_count)
        max_raw_split_count = max(max_raw_split_count, raw_split_count)
        if bool(row.get("splitCapped")):
            split_capped_count += 1
        max_projected_span_m = max(max_projected_span_m, _to_float(row.get("projectedSpanM"), 0.0))
        max_total_len_m = max(max_total_len_m, _to_float(row.get("totalLenM"), 0.0))
        max_segment_len_m = max(max_segment_len_m, _to_float(row.get("segmentLenM"), 0.0))
    return {
        "detailCount": len(details),
        "changedDetails": changed_details,
        "splitCountSum": split_count_sum,
        "rawSplitCountSum": raw_split_count_sum,
        "maxSplitCount": max_split_count,
        "maxRawSplitCount": max_raw_split_count,
        "splitCappedCount": split_capped_count,
        "maxProjectedSpanM": max_projected_span_m,
        "maxTotalLenM": max_total_len_m,
        "maxSegmentLenM": max_segment_len_m,
        "reasonCounts": reason_counts,
    }


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


def _apply_vehicle_status_filter(
    cmpk: Dict[str, Any],
    log: Callable[[str], None],
    *,
    cmpk_path: str | Path | None = None,
    trust_input_aircraft: bool = False,
) -> None:
    if trust_input_aircraft:
        log("[ENHANCED] VehicleStatus filter skipped for initial planning; using 0201 availableAircraftList")
        return
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
    try:
        code = int(option_code or 0)
    except Exception:
        code = 0
    if code == 4:
        return PROFILE_RECON
    if code == 5:
        return PROFILE_MIN_TIME
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
    trust_input_aircraft: bool = False,
) -> List[str]:
    t0 = time.perf_counter()
    cmpk = json.loads(Path(cmpk_path).read_text(encoding="utf-8"))
    mrpk = json.loads(Path(ref_path).read_text(encoding="utf-8"))
    _apply_vehicle_status_filter(
        cmpk,
        log,
        cmpk_path=cmpk_path,
        trust_input_aircraft=trust_input_aircraft,
    )
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

    runtime_cfg = _load_runtime_settings()
    review_max_segment_m = get_runtime_area_review_max_segment_m(3000.0, runtime_cfg)
    review_max_split_count = max(0, get_runtime_int("recon_area_review_max_split_count", 0, runtime_cfg))
    review_min_segment_m = max(0.0, get_runtime_float("recon_area_review_min_segment_m", 0.0, runtime_cfg))
    area_review_mode = _settings_area_mode(runtime_cfg)
    review_enabled = _settings_bool(runtime_cfg, "enhanced_area_review_enabled", True)
    if area_review_mode == "nadir":
        review_enabled = False
    if int(option_code or 0) == 4 and review_enabled:
        area_before = _count_area_pieces(split_result)
        pieces_before = len(split_result.pieces)
        fixed_fov_deg = get_runtime_float("recon_area_fixed_fov_deg", 0.0, runtime_cfg)
        area_fov_deg = get_runtime_float("area_custom_fov_deg", fixed_fov_deg, runtime_cfg)
        sweep_sep_m = get_runtime_float("default_sweep_separation_m", 0.0, runtime_cfg)
        sweep_sep_scale = get_runtime_float("recon_sweep_separation_scale", 0.0, runtime_cfg)
        turn_radius_m = get_runtime_float("dubins_turn_radius_m", 0.0, runtime_cfg)
        log(
            "[REPLAN][METRIC] recon_area_review_before "
            f"option=4 pieces={pieces_before} areaPieces={area_before} "
            f"maxSegmentM={float(review_max_segment_m):.3f} "
            f"maxSplitCountLimit={int(review_max_split_count)} "
            f"minSegmentM={float(review_min_segment_m):.3f} "
            f"fixedFovDeg={float(fixed_fov_deg):.3f} areaFovDeg={float(area_fov_deg):.3f} "
            f"sweepSeparationM={float(sweep_sep_m):.3f} sweepSeparationScale={float(sweep_sep_scale):.3f} "
            f"turnRadiusM={float(turn_radius_m):.3f}"
        )
        review_t0 = time.perf_counter()
        recon_review = review_assigned_areas_local(
            split_result,
            mrpk,
            max_segment_m=review_max_segment_m,
            max_split_count=review_max_split_count,
            min_segment_m=review_min_segment_m,
        )
        review_elapsed_ms = (time.perf_counter() - review_t0) * 1000.0
        area_after = _count_area_pieces(split_result)
        detail_stats = _review_detail_stats(recon_review)
        reason_counts = detail_stats.get("reasonCounts") if isinstance(detail_stats, dict) else {}
        reason_text = "-"
        if isinstance(reason_counts, dict) and reason_counts:
            reason_text = "|".join(
                f"{key}:{value}" for key, value in sorted(reason_counts.items())
            )
        log(
            "[REPLAN][METRIC] recon_area_review "
            f"option=4 elapsed_ms={review_elapsed_ms:.3f} "
            f"maxSegmentM={float(review_max_segment_m):.3f} "
            f"maxSplitCountLimit={int(review_max_split_count)} "
            f"minSegmentM={float(review_min_segment_m):.3f} "
            f"pieces_before={pieces_before} pieces_after={int(recon_review.get('newPieceCount', 0))} "
            f"area_before={area_before} area_after={area_after} "
            f"targets={int(recon_review.get('targets', 0))} "
            f"localized={int(recon_review.get('localized', 0))} "
            f"changed_details={int(detail_stats.get('changedDetails', 0))} "
            f"detail_count={int(detail_stats.get('detailCount', 0))} "
            f"split_count_sum={int(detail_stats.get('splitCountSum', 0))} "
            f"raw_split_count_sum={int(detail_stats.get('rawSplitCountSum', 0))} "
            f"max_split_count={int(detail_stats.get('maxSplitCount', 0))} "
            f"max_raw_split_count={int(detail_stats.get('maxRawSplitCount', 0))} "
            f"split_capped_count={int(detail_stats.get('splitCappedCount', 0))} "
            f"max_projected_span_m={float(detail_stats.get('maxProjectedSpanM', 0.0)):.3f} "
            f"max_total_len_m={float(detail_stats.get('maxTotalLenM', 0.0)):.3f} "
            f"max_segment_len_m={float(detail_stats.get('maxSegmentLenM', 0.0)):.3f} "
            f"fixedFovDeg={float(fixed_fov_deg):.3f} areaFovDeg={float(area_fov_deg):.3f} "
            f"sweepSeparationM={float(sweep_sep_m):.3f} sweepSeparationScale={float(sweep_sep_scale):.3f} "
            f"turnRadiusM={float(turn_radius_m):.3f} "
            f"reasons={reason_text}"
        )
        log(
            "[ENHANCED][1R] recon-area-review done: "
            f"targets={int(recon_review.get('targets', 0))} "
            f"localized={int(recon_review.get('localized', 0))} "
            f"pieces={int(recon_review.get('oldPieceCount', 0))}->"
            f"{int(recon_review.get('newPieceCount', 0))}"
        )
    elif int(option_code or 0) == 4:
        log(
            "[REPLAN][METRIC] recon_area_review_skipped "
            f"option=4 reason={'nadir_mode' if area_review_mode == 'nadir' else 'disabled_by_config'}"
        )

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

    if review_enabled:
        review_report = review_overflow_areas(
            split_result,
            expected_paths,
            max_segment_m=review_max_segment_m,
            max_split_count=review_max_split_count if int(option_code or 0) == 4 else 0,
            min_segment_m=review_min_segment_m if int(option_code or 0) == 4 else 0.0,
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
