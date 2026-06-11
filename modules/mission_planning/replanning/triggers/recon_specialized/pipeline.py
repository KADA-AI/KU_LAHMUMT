from __future__ import annotations

import copy
from typing import Any, Dict, Optional

from modules.common.option_codes import normalize_option_code

from modules.mission_planning.MissionPlanner.runtime_settings import (
    canonicalize_runtime_payload,
    get_runtime_fov_db_path,
    read_fov_db_rows_from_path,
)


RECON_OPTION_CODE = 4
DEFAULT_RECON_AREA_SPLIT_WIDTH_M = 600.0
DEFAULT_RECON_AREA_FIXED_FOV_DEG = 3.0
DEFAULT_RECON_SWEEP_SEPARATION_SCALE = 0.50
DEFAULT_RECON_FALLBACK_SEP_M = 500.0
DEFAULT_RECON_LINESEARCH_INTERP_POINTS = 2
_SAFE_RECON_REDUCTION_REASONS = {
    "",
    "-",
    "min_segment_m",
    "skip_piece_span_within_limit",
    "skip_patternType_not_6",
    "invalid_polygon_coords",
    "invalid_polygon_geom",
    "missing_takeover",
    "unassigned_piece",
}


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def _min_sep_for_fov(
    payload: Dict[str, Any],
    fov_deg: float,
    fallback_m: float = DEFAULT_RECON_FALLBACK_SEP_M,
) -> float:
    try:
        target_fov = float(fov_deg)
    except Exception:
        target_fov = 0.0
    if target_fov <= 0.0:
        return float(fallback_m)
    try:
        db_path = get_runtime_fov_db_path(payload)
        rows = read_fov_db_rows_from_path(db_path)
    except Exception:
        rows = []
    matches: list[float] = []
    for row in rows:
        try:
            row_fov = float(row.get("fov", 0.0) or 0.0)
            row_sep = float(row.get("sep", 0.0) or 0.0)
        except Exception:
            continue
        if row_fov <= 0.0 or row_sep <= 0.0:
            continue
        if abs(row_fov - target_fov) <= 0.05:
            matches.append(float(row_sep))
    if matches:
        return float(min(matches))
    return float(fallback_m)


def _path_coordinate_count(path: Dict[str, Any]) -> int:
    for key in ("coordinateList", "waypointList", "uavWaypointList", "lahWaypointList"):
        rows = path.get(key) if isinstance(path, dict) else None
        if isinstance(rows, list):
            return len([row for row in rows if isinstance(row, dict)])
    return 0


def _path_signature(paths: Any) -> tuple[tuple[Any, ...], ...]:
    rows = []
    for path in paths or []:
        if not isinstance(path, dict):
            continue
        rows.append(
            (
                str(path.get("source") or ""),
                str(path.get("pathRole") or ""),
                _as_int(path.get("parentOrder"), 0),
                _as_int(path.get("index"), 0),
                round(_as_float(path.get("sepM"), 0.0), 3),
                _path_coordinate_count(path),
            )
        )
    return tuple(sorted(rows))


def _detail_stats_from_report(review_report: Dict[str, Any] | None) -> Dict[str, Any]:
    details = review_report.get("details") if isinstance(review_report, dict) else None
    if not isinstance(details, list):
        details = []
    reason_counts: Dict[str, int] = {}
    cap_reasons: Dict[str, int] = {}
    max_split_count = 0
    max_raw_split_count = 0
    split_capped_count = 0
    max_segment_len_m = 0.0
    for row in details:
        if not isinstance(row, dict):
            continue
        reason = str(row.get("reason") or "").strip()
        if reason:
            reason_counts[reason] = reason_counts.get(reason, 0) + 1
        for key in ("splitCapReason", "pairSplitCapReason"):
            cap_reason = str(row.get(key) or "").strip()
            if cap_reason and cap_reason != "-":
                cap_reasons[cap_reason] = cap_reasons.get(cap_reason, 0) + 1
        split_count = max(_as_int(row.get("splitCount"), 0), _as_int(row.get("pairSplitCount"), 0))
        raw_split_count = max(_as_int(row.get("rawSplitCount"), split_count), _as_int(row.get("pairRawSplitCount"), 0))
        max_split_count = max(max_split_count, split_count)
        max_raw_split_count = max(max_raw_split_count, raw_split_count)
        if bool(row.get("splitCapped")) or bool(row.get("pairSplitCapped")):
            split_capped_count += 1
        max_segment_len_m = max(
            max_segment_len_m,
            _as_float(row.get("segmentLenM"), 0.0),
            _as_float(row.get("pairSegmentLenM"), 0.0),
        )
    return {
        "reasonCounts": reason_counts,
        "capReasonCounts": cap_reasons,
        "maxSplitCount": max_split_count,
        "maxRawSplitCount": max_raw_split_count,
        "splitCappedCount": split_capped_count,
        "maxSegmentLenM": max_segment_len_m,
    }


def is_recon_specialized_option(
    option_code: object | None = None,
    option_label: object | None = None,
) -> bool:
    code = normalize_option_code(option_code)
    if code is None:
        code = normalize_option_code(option_label)
    return int(code or 0) == RECON_OPTION_CODE


def build_recon_specialized_runtime_payload(
    base_payload: Dict[str, Any] | None,
) -> Dict[str, Any]:
    if isinstance(base_payload, dict):
        payload = copy.deepcopy(base_payload)
    else:
        payload = canonicalize_runtime_payload(None)
    values = payload.get("values")
    if not isinstance(values, dict):
        values = {}
        payload["values"] = values

    split_width_m = max(
        50.0,
        _as_float(
            values.get("recon_area_split_width_m"),
            DEFAULT_RECON_AREA_SPLIT_WIDTH_M,
        ),
    )
    recon_override_active = bool(values.get("recon_override_enabled", False))
    fixed_fov_deg = max(
        0.1,
        _as_float(
            values.get("recon_area_fixed_fov_deg") if recon_override_active else None,
            DEFAULT_RECON_AREA_FIXED_FOV_DEG,
        ),
    )
    recon_sep_m = max(1.0, _min_sep_for_fov(payload, fixed_fov_deg, DEFAULT_RECON_FALLBACK_SEP_M))
    values["enhanced_area_review_max_segment_m"] = float(split_width_m)
    values["capture_mission_param_mode"] = "manual"
    values["area_override_enabled"] = True
    values["area_override_fov_deg"] = float(fixed_fov_deg)
    values["area_override_density_scale"] = float(values.get("area_density_scale", 1.2) or 1.2)
    values["area_override_search_speed_weight"] = float(values.get("area_search_speed_weight", 1.0) or 1.0)
    values["area_override_route_offset_scale"] = 1.0
    values["recon_override_enabled"] = True
    values["recon_override_fixed_fov_deg"] = float(fixed_fov_deg)
    values["recon_override_sweep_separation_scale"] = 1.0
    values["area_custom_fov_deg"] = float(fixed_fov_deg)
    values["recon_area_fixed_fov_deg"] = float(fixed_fov_deg)
    values["default_sweep_separation_m"] = float(recon_sep_m)
    values["recon_sweep_separation_scale"] = 1.0
    values["area_route_offset_scale"] = 1.0
    values["area_route_offset_sep_m"] = None
    values["recon_route_offset_sep_m"] = None
    recon_interp_enabled = bool(values.get("recon_linesearch_interp_points_enabled", True))
    if recon_interp_enabled:
        values["sweep_line_interp_points"] = max(
            2,
            _as_int(
                values.get("recon_linesearch_interp_points"),
                DEFAULT_RECON_LINESEARCH_INTERP_POINTS,
            ),
        )
    # Recon-specialized planning keeps its own area FOV/offset separate from the global manual FOV.
    values["enhanced_auto_fov_from_db"] = False
    values["manual_fov_global_sync_enabled"] = False
    return payload


def summarize_recon_area_review_guard(
    *,
    review_report: Dict[str, Any] | None,
    detail_stats: Dict[str, Any] | None = None,
    max_segment_m: float = 0.0,
    max_split_count: int = 0,
    min_segment_m: float = 0.0,
) -> Dict[str, Any]:
    report = review_report if isinstance(review_report, dict) else {}
    stats = detail_stats if isinstance(detail_stats, dict) else _detail_stats_from_report(report)
    cap_reasons = dict(stats.get("capReasonCounts") or {})
    if not cap_reasons:
        cap_reasons = _detail_stats_from_report(report).get("capReasonCounts") or {}
    reason_counts = dict(stats.get("reasonCounts") or {})
    risky_cap_reasons = sorted(
        str(reason)
        for reason in cap_reasons
        if str(reason) not in {"", "-", "min_segment_m"}
    )
    unsafe_reasons = sorted(
        str(reason)
        for reason in reason_counts
        if str(reason) not in _SAFE_RECON_REDUCTION_REASONS
    )
    max_split_observed = _as_int(stats.get("maxSplitCount"), 0)
    max_raw_split_observed = _as_int(stats.get("maxRawSplitCount"), max_split_observed)
    max_split_limit = max(0, int(max_split_count or 0))
    min_segment_threshold_active = float(min_segment_m or 0.0) > 0.0
    hard_cap_active = max_split_limit > 0
    piece_count_within_cap = (
        not hard_cap_active or max_split_observed <= max_split_limit
    )
    accuracy_risk = bool(risky_cap_reasons or unsafe_reasons)
    return {
        "oldPieceCount": _as_int(report.get("oldPieceCount"), 0),
        "newPieceCount": _as_int(report.get("newPieceCount"), 0),
        "pieceCountDelta": _as_int(report.get("newPieceCount"), 0) - _as_int(report.get("oldPieceCount"), 0),
        "maxSegmentM": float(max_segment_m or 0.0),
        "maxSplitCountLimit": int(max_split_limit),
        "minSegmentM": float(min_segment_m or 0.0),
        "maxSplitCountObserved": int(max_split_observed),
        "maxRawSplitCountObserved": int(max_raw_split_observed),
        "splitCappedCount": _as_int(stats.get("splitCappedCount"), 0),
        "pieceCountWithinCap": bool(piece_count_within_cap),
        "pathExplosionGuardReviewed": True,
        "hardCapActive": bool(hard_cap_active),
        "minSegmentThresholdActive": bool(min_segment_threshold_active),
        "accuracyPreservingReductionOnly": not accuracy_risk,
        "accuracyRiskReasons": risky_cap_reasons + unsafe_reasons,
        "capReasonCounts": cap_reasons,
        "reasonCounts": reason_counts,
    }


def summarize_recon_expected_path_quality(
    expected_paths: Any,
    *,
    option_code: int | None = None,
) -> Dict[str, Any]:
    rows = [row for row in (expected_paths or []) if isinstance(row, dict)]
    line_rows = [
        row
        for row in rows
        if str(row.get("source") or "").startswith("line_center_offset_dir")
    ]
    coordinate_counts = [_path_coordinate_count(row) for row in rows]
    line_coordinate_counts = [_path_coordinate_count(row) for row in line_rows]
    min_coords = min(coordinate_counts) if coordinate_counts else 0
    min_line_coords = min(line_coordinate_counts) if line_coordinate_counts else 0
    return {
        "optionCode": _as_int(option_code, 0),
        "pathCount": len(rows),
        "lineSearchPathCount": len(line_rows),
        "minCoordinateCount": int(min_coords),
        "minLineSearchCoordinateCount": int(min_line_coords),
        "lineSearchQualityGuardPassed": bool(line_rows and min_line_coords >= 2),
        "coverageQualityGuardPassed": bool(rows and min_coords >= 2),
    }


def compare_recon_option_path_signatures(
    *,
    option4_expected_paths: Any,
    option6_expected_paths: Any,
) -> Dict[str, Any]:
    sig4 = _path_signature(option4_expected_paths)
    sig6 = _path_signature(option6_expected_paths)
    return {
        "option4PathCount": len(sig4),
        "option6PathCount": len(sig6),
        "option4EqualsOption6": bool(sig4 == sig6),
        "option4Signature": sig4,
        "option6Signature": sig6,
    }
