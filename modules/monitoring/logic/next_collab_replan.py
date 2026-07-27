from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

from modules.monitoring.logic.init_replan import allocate_mission_plan_ids
from modules.monitoring.logic.turn_radius_monitor import (
    _math_rad_to_heading_deg,
    is_path_deviation_turn_warning_view,
)
from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float, get_runtime_str
from modules.mission_planning.runtime import next_collab_replan_runtime, next_collab_replan_store


REPLAN_LEVEL = 3
REPLAN_REASON = "_협업 기저 전환으로 인한 재계획"
OPTION_NAME = "비행/촬영"
TRIGGER_TYPE = "nextCollaborativeMission"
REPLAN_REASON = next_collab_replan_runtime.REPLAN_REASON
OPTION_NAME = next_collab_replan_runtime.OPTION_NAME
TRIGGER_TYPE = next_collab_replan_runtime.TRIGGER_TYPE
FORMATION_FLIGHT_INPUT_MISSION_TYPE = 7
ENTRY_ALTITUDE_MIN_M = 700.0
ENTRY_ALTITUDE_MAX_REFERENCE_DELTA_M = 600.0

def _planner_turn_radius_scale() -> float:
    try:
        scale = float(get_runtime_float("next_collab_turn_radius_scale", 1.20))
    except Exception:
        scale = 1.20
    return max(0.1, float(scale))


def _planner_default_entry_strategy() -> str:
    try:
        value = str(get_runtime_str("next_collab_default_entry_strategy", "turn_projection"))
    except Exception:
        value = "turn_projection"
    text = value.strip().lower()
    if text in {"current_position", "midpoint_to_next_start", "turn_projection"}:
        return text
    return "turn_projection"


def _coerce_int(value: object | None) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _coerce_float(value: object | None) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _coerce_altitude_int(value: object | None) -> int | None:
    altitude = _coerce_float(value)
    if altitude is None:
        return None
    return int(round(float(altitude)))


def _normalize_coordinate(payload: object | None) -> dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _coerce_float(payload.get("latitude"))
    lon = _coerce_float(payload.get("longitude"))
    alt = _coerce_altitude_int(payload.get("altitude"))
    if lat is None or lon is None:
        return None
    coord: dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        coord["altitude"] = int(alt)
    return coord


def _centroid_coordinate(coords: list[dict[str, float]]) -> dict[str, float] | None:
    if not coords:
        return None
    lat_vals = [float(item["latitude"]) for item in coords if "latitude" in item]
    lon_vals = [float(item["longitude"]) for item in coords if "longitude" in item]
    if not lat_vals or not lon_vals:
        return None
    out: dict[str, float] = {
        "latitude": sum(lat_vals) / float(len(lat_vals)),
        "longitude": sum(lon_vals) / float(len(lon_vals)),
    }
    alt_vals = [
        float(item["altitude"])
        for item in coords
        if isinstance(item, dict) and _coerce_float(item.get("altitude")) is not None
    ]
    if alt_vals:
        out["altitude"] = int(round(sum(alt_vals) / float(len(alt_vals))))
    return out


def _midpoint_coordinate(
    start: dict[str, float] | None,
    end: dict[str, float] | None,
) -> dict[str, float] | None:
    if start is None or end is None:
        return None
    lat1 = _coerce_float(start.get("latitude"))
    lon1 = _coerce_float(start.get("longitude"))
    lat2 = _coerce_float(end.get("latitude"))
    lon2 = _coerce_float(end.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    coord: dict[str, float] = {
        "latitude": (float(lat1) + float(lat2)) / 2.0,
        "longitude": (float(lon1) + float(lon2)) / 2.0,
    }
    alt1 = _coerce_float(start.get("altitude"))
    alt2 = _coerce_float(end.get("altitude"))
    if alt1 is not None and alt2 is not None:
        coord["altitude"] = int(round((float(alt1) + float(alt2)) / 2.0))
    elif alt1 is not None:
        coord["altitude"] = int(round(float(alt1)))
    elif alt2 is not None:
        coord["altitude"] = int(round(float(alt2)))
    return coord


def _sanitize_entry_altitude(
    coord: dict[str, float] | None,
    reference_coord: dict[str, float] | None,
) -> dict[str, float] | None:
    if coord is None:
        return None
    out = dict(coord)
    reference_altitude = _coerce_float((reference_coord or {}).get("altitude"))
    if reference_altitude is None:
        return out
    altitude = _coerce_float(out.get("altitude"))
    if (
        altitude is None
        or float(altitude) < float(ENTRY_ALTITUDE_MIN_M)
        or abs(float(altitude) - float(reference_altitude)) > float(ENTRY_ALTITUDE_MAX_REFERENCE_DELTA_M)
    ):
        out["altitude"] = int(round(float(reference_altitude)))
    return out


def _resolve_entry_altitude(
    coord: dict[str, float] | None,
    *,
    position_coord: dict[str, float] | None,
    reference_coord: dict[str, float] | None,
) -> tuple[dict[str, float] | None, float | None, str | None]:
    """진입 좌표의 고도를 확정한다. 반환: (coord, 이전 고도, 보정 사유).

    진입 좌표의 lat/lon은 전략(현재위치/중간점/다음시작)을 따르지만, 고도는
    기체의 0401 실측 고도를 그대로 쓴다 — 이 값이 플래너의 "첫 WP 고도 =
    0401 고정" 정책의 입력이라, 중간점 평균이나 목표 진입 고도가 섞이면
    첫 WP 고도가 기체 실제 고도와 어긋나 임무 시작부터 불필요한 급상승/
    급강하 지시가 생긴다. 0401 고도가 비정상(결측·ENTRY_ALTITUDE_MIN_M
    미만)일 때만 종전 참조 고도 보정으로 폴백한다.
    """
    if coord is None:
        return None, None, None
    previous_altitude = _coerce_float(coord.get("altitude"))
    telemetry_altitude = _coerce_float((position_coord or {}).get("altitude"))
    if telemetry_altitude is not None and float(telemetry_altitude) >= float(ENTRY_ALTITUDE_MIN_M):
        rounded = int(round(float(telemetry_altitude)))
        if previous_altitude is not None and int(round(float(previous_altitude))) == rounded:
            return dict(coord), previous_altitude, None
        out = dict(coord)
        out["altitude"] = rounded
        return out, previous_altitude, "follows_0401"
    sanitized = _sanitize_entry_altitude(coord, reference_coord)
    if sanitized is not None and sanitized != coord:
        return sanitized, previous_altitude, "reference_fallback"
    return sanitized, previous_altitude, None


@dataclass(frozen=True)
class ExecuteNextContext:
    input_mission_package_id: int
    current_input_mission_id: int
    target_input_mission_id: int
    target_input_mission_type: int | None
    target_aircraft_ids: list[int]
    current_input_progress_percent: int
    current_input_is_done: bool
    current_input_recommendation_active: bool
    entry_strategy: str
    target_entry_by_aircraft: dict[int, dict[str, float]]
    representative_target_entry_coordinate: dict[str, float] | None


class NextCollabMissionReplanCoordinator:
    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger

    def on_execute_next(
        self,
        context_payload: dict[str, Any] | None,
        *,
        turn_views: dict[int, Any] | None,
        current_mission_plan_id: int | None,
        system_mode: int | None,
    ) -> tuple[dict[str, Any] | None, list[str]]:
        logs: list[str] = []
        if system_mode not in (3, 4):
            logs.append(f"[0803] execute=1 skipped: mode={system_mode} (need 3/4)")
            return None, logs
        if current_mission_plan_id is None:
            logs.append("[0803] execute=1 skipped: current missionPlanID unavailable")
            return None, logs

        context = self._parse_context(context_payload)
        if context is None:
            logs.append("[0803] execute=1 skipped: next mission context unavailable")
            return None, logs
        is_formation_target = int(context.target_input_mission_type or 0) == FORMATION_FLIGHT_INPUT_MISSION_TYPE
        if is_formation_target:
            logs.append(
                "[0803] execute=1 formation-flight target: "
                f"target input mission {context.target_input_mission_id} is formation-flight mode "
                f"(inputMissionType={FORMATION_FLIGHT_INPUT_MISSION_TYPE}); preparing reference-route replan"
            )

        entries: list[dict[str, Any]] = []
        coords_for_centroid: list[dict[str, float]] = []
        views = turn_views if isinstance(turn_views, dict) else {}
        strategy = str(context.entry_strategy or "").strip().lower()
        use_turn_projection = strategy == "turn_projection"
        use_current_position = strategy == "current_position"
        turn_radius_scale = _planner_turn_radius_scale()
        for aircraft_id in context.target_aircraft_ids:
            view = views.get(int(aircraft_id))
            coord = None
            source = None
            eta_s = None
            raw_heading_deg = None
            heading_rad = None
            speed_mps = None
            turn_rate_dps = None
            turn_sign = None
            ideal_radius_m = None
            actual_radius_m = None
            turn_circle_radius_m = None
            position_coord = None
            line_trend_heading_deg = None
            line_trend_turn_rate_dps = None
            line_trend_turn_sign = None
            line_turn_confidence = None
            line_trend_sample_count = None
            line_trend_source = None
            line_predicted_coord = None
            line_predicted_heading_deg = None
            line_prediction_lead_s = None
            turn_data_age_s = None
            if view is not None:
                position_coord = _normalize_coordinate(getattr(view, "position_coordinate", None))
                target_entry_coord = context.target_entry_by_aircraft.get(int(aircraft_id))
                if target_entry_coord is None:
                    target_entry_coord = context.representative_target_entry_coordinate
                raw_heading_deg = _coerce_float(getattr(view, "raw_heading_deg", None))
                heading_rad = _coerce_float(getattr(view, "heading_rad", None))
                speed_mps = _coerce_float(getattr(view, "speed_mps", None))
                turn_rate_dps = _coerce_float(getattr(view, "turn_rate_dps", None))
                turn_sign_raw = _coerce_int(getattr(view, "turn_sign", None))
                turn_sign = int(turn_sign_raw) if turn_sign_raw is not None else None
                ideal_radius_m = _coerce_float(getattr(view, "ideal_radius_m", None))
                actual_radius_m = _coerce_float(getattr(view, "actual_radius_m", None))
                turn_circle_radius_m = _coerce_float(getattr(view, "turn_circle_radius_m", None))
                line_trend_heading_deg = _coerce_float(
                    getattr(view, "line_trend_heading_deg", None)
                )
                line_trend_turn_rate_dps = _coerce_float(
                    getattr(view, "line_trend_turn_rate_dps", None)
                )
                line_trend_turn_sign = _coerce_int(
                    getattr(view, "line_trend_turn_sign", None)
                )
                line_turn_confidence = _coerce_float(
                    getattr(view, "line_turn_direction_confidence", None)
                )
                line_trend_sample_count = _coerce_int(
                    getattr(view, "line_trend_sample_count", None)
                )
                line_trend_source = getattr(view, "line_trend_source", None)
                line_predicted_coord = _normalize_coordinate(
                    getattr(view, "line_predicted_entry_coordinate", None)
                )
                line_predicted_heading_deg = _coerce_float(
                    getattr(view, "line_predicted_heading_deg", None)
                )
                line_prediction_lead_s = _coerce_float(
                    getattr(view, "line_prediction_lead_s", None)
                )
                turn_data_age_s = _coerce_float(getattr(view, "age_s", None))
                if turn_data_age_s is not None and float(turn_data_age_s) > 5.0:
                    line_turn_confidence = 0.0
                    line_trend_turn_sign = 0
                    line_predicted_coord = None
                    line_predicted_heading_deg = None
                if use_current_position:
                    coord = position_coord
                    eta_s = 0.0 if coord is not None else None
                    if coord is not None:
                        source = "currentPosition0401"
                    if coord is None and target_entry_coord is not None:
                        coord = dict(target_entry_coord)
                        source = "nextStartFallback"
                elif use_turn_projection:
                    coord = position_coord
                    eta_s = 0.0 if coord is not None else None
                    if coord is not None:
                        source = "currentPosition0401"
                else:
                    coord = _midpoint_coordinate(position_coord, target_entry_coord)
                    if coord is not None:
                        source = "midpointCurrentToNextStart"
                    if coord is None and target_entry_coord is not None:
                        coord = dict(target_entry_coord)
                        source = "nextStart"
                    if coord is None and position_coord is not None:
                        coord = dict(position_coord)
                        eta_s = 0.0
                        source = "currentPosition"
            if coord is None:
                logs.append(f"[0803] execute=1 skipped aircraft {aircraft_id}: no entry coordinate")
                continue
            altitude_reference = target_entry_coord or context.representative_target_entry_coordinate
            resolved_coord, previous_altitude, altitude_reason = _resolve_entry_altitude(
                coord,
                position_coord=position_coord,
                reference_coord=altitude_reference,
            )
            if resolved_coord is not None and altitude_reason is not None:
                new_alt = resolved_coord.get("altitude")
                label = "follows 0401" if altitude_reason == "follows_0401" else "sanitized"
                logs.append(
                    f"[0803] execute=1 aircraft {aircraft_id}: entry altitude {label} "
                    f"{previous_altitude if previous_altitude is not None else 'n/a'} -> {new_alt}"
                )
            if resolved_coord is not None:
                coord = resolved_coord
            coords_for_centroid.append(coord)
            entry: dict[str, Any] = {
                "aircraftID": int(aircraft_id),
                "coordinate": coord,
                "source": str(source or "unknown"),
            }
            if raw_heading_deg is None:
                if heading_rad is not None:
                    raw_heading_deg = _math_rad_to_heading_deg(float(heading_rad))
            if raw_heading_deg is not None:
                entry["headingDeg"] = float(raw_heading_deg) % 360.0
            if eta_s is not None:
                entry["etaS"] = float(eta_s)
            if speed_mps is not None:
                entry["speedMps"] = float(speed_mps)
            if turn_rate_dps is not None:
                entry["turnRateDps"] = float(turn_rate_dps)
            if turn_sign is not None:
                entry["turnSign"] = int(turn_sign)
            turn_radius_m = turn_circle_radius_m or actual_radius_m or ideal_radius_m
            if turn_radius_m is not None:
                entry["turnRadiusM"] = float(turn_radius_m)
            if ideal_radius_m is not None:
                entry["idealTurnRadiusM"] = float(ideal_radius_m)
            if actual_radius_m is not None:
                entry["actualTurnRadiusM"] = float(actual_radius_m)
            if turn_circle_radius_m is not None:
                entry["turnCircleRadiusM"] = float(turn_circle_radius_m)
            if position_coord is not None:
                entry["currentCoordinate"] = dict(position_coord)
            if line_trend_heading_deg is not None:
                entry["lineTrendHeadingDeg"] = float(line_trend_heading_deg) % 360.0
            if line_trend_turn_rate_dps is not None:
                entry["lineTrendTurnRateDps"] = float(line_trend_turn_rate_dps)
            if line_trend_turn_sign is not None:
                entry["lineTrendTurnSign"] = (
                    1 if int(line_trend_turn_sign) > 0 else -1 if int(line_trend_turn_sign) < 0 else 0
                )
            if line_turn_confidence is not None:
                entry["lineTurnDirectionConfidence"] = max(
                    0.0,
                    min(1.0, float(line_turn_confidence)),
                )
            if line_trend_sample_count is not None:
                entry["lineTrendSampleCount"] = max(0, int(line_trend_sample_count))
            if line_trend_source:
                entry["lineTrendSource"] = str(line_trend_source)
            if line_predicted_coord is not None:
                entry["linePredictedEntryCoordinate"] = dict(line_predicted_coord)
            if line_predicted_heading_deg is not None:
                entry["linePredictedHeadingDeg"] = float(line_predicted_heading_deg) % 360.0
            if line_prediction_lead_s is not None:
                entry["linePredictionLeadS"] = max(0.0, float(line_prediction_lead_s))
            if turn_data_age_s is not None:
                entry["lineTurnDataAgeS"] = max(0.0, float(turn_data_age_s))
            if (
                line_turn_confidence is not None
                and float(line_turn_confidence) >= 0.45
                and line_trend_turn_sign not in (None, 0)
            ):
                logs.append(
                    f"[0803][LINE-TREND] UAV{aircraft_id} "
                    f"sign={int(line_trend_turn_sign):+d} "
                    f"rate={float(line_trend_turn_rate_dps or 0.0):+.2f}dps "
                    f"confidence={float(line_turn_confidence):.2f} "
                    f"model={str(line_trend_source or 'recent')}"
                )
            entries.append(entry)

        if not entries:
            logs.append("[0803] execute=1 skipped: no aircraft entry coordinates resolved")
            return None, logs

        mission_plan_ids = allocate_mission_plan_ids(1)
        if not mission_plan_ids:
            logs.append("[0803] execute=1 skipped: MissionPlanID allocation failed")
            return None, logs

        mission_plan_id = int(mission_plan_ids[0])
        timestamp_ms = int(self._now_ms())
        representative_entry = _centroid_coordinate(coords_for_centroid)
        detail_payload: dict[str, Any] = {
            "trigger": "0803",
            "triggerType": TRIGGER_TYPE,
            "sourceMissionPlanID": int(current_mission_plan_id),
            "inputMissionPackageID": int(context.input_mission_package_id),
            "currentInputMissionID": int(context.current_input_mission_id),
            "targetInputMissionID": int(context.target_input_mission_id),
            "turnRadiusScale": float(turn_radius_scale),
            "entryStrategy": str(context.entry_strategy or ""),
            "entryAircraftList": entries,
            "keepCurrentMissionDone": True,
            "forceDirectUpdate": True,
            "suppress0702Fallback": True,
            "selectedReplanReason": REPLAN_REASON,
            "timestamp": int(timestamp_ms),
        }
        if representative_entry is not None:
            detail_payload["representativeEntryCoordinate"] = representative_entry
        if context.representative_target_entry_coordinate is not None:
            detail_payload["representativeTargetEntryCoordinate"] = dict(
                context.representative_target_entry_coordinate
            )

        payload = {
            "timestamp": int(timestamp_ms),
            "source": "MSM",
            "inputMissionPackageID": int(context.input_mission_package_id),
            "replanRequestTime": {"replanRequestTimestamp": int(timestamp_ms)},
            "replanLevel": int(REPLAN_LEVEL),
            "replanRequest": REPLAN_REASON,
            "replanReason": REPLAN_REASON,
            "inputMissionIDList": [
                {"inputMissionID": int(context.target_input_mission_id)},
            ],
            "pendingOptionList": [
                {
                    "optionID": 1,
                    "optionName": OPTION_NAME,
                    "missionPlanID": int(mission_plan_id),
                }
            ],
            "replanDetail": detail_payload,
        }

        try:
            next_collab_replan_store.save_detail(int(mission_plan_id), detail_payload)
            next_collab_replan_store.save_event(
                "monitor_dispatch",
                {
                    "missionPlanID": int(mission_plan_id),
                    "detail": dict(detail_payload),
                },
            )
        except Exception:
            pass

        logs.append(
            "[0803] execute=1 -> next collaborative mission replan prepared "
            f"(currentInput={context.current_input_mission_id}, targetInput={context.target_input_mission_id}, "
            f"strategy={context.entry_strategy}, "
            f"aircraft={','.join(str(v) for v in context.target_aircraft_ids)}, missionPlanID={mission_plan_id})"
        )
        return payload, logs

    def _parse_context(self, payload: dict[str, Any] | None) -> ExecuteNextContext | None:
        if not isinstance(payload, dict):
            return None
        input_package_id = _coerce_int(payload.get("input_mission_package_id"))
        current_input_id = _coerce_int(payload.get("current_input_mission_id"))
        target_input_id = _coerce_int(payload.get("target_input_mission_id"))
        target_input_type = _coerce_int(payload.get("target_input_mission_type"))
        current_input_progress_percent = _coerce_int(payload.get("current_input_progress_percent")) or 0
        current_input_is_done = bool(payload.get("current_input_is_done"))
        current_input_recommendation_active = bool(payload.get("current_input_recommendation_active"))
        default_entry_strategy = _planner_default_entry_strategy()
        entry_strategy = str(payload.get("entry_strategy") or "").strip().lower() or default_entry_strategy
        aircraft_ids_raw = payload.get("target_aircraft_ids")
        aircraft_ids: list[int] = []
        if isinstance(aircraft_ids_raw, list):
            for value in aircraft_ids_raw:
                aid = _coerce_int(value)
                if aid is None or aid <= 0 or aid in aircraft_ids:
                    continue
                aircraft_ids.append(int(aid))
        target_entry_by_aircraft: dict[int, dict[str, float]] = {}
        target_entry_raw = payload.get("target_entry_aircraft_list")
        if isinstance(target_entry_raw, list):
            for item in target_entry_raw:
                if not isinstance(item, dict):
                    continue
                aid = _coerce_int(item.get("aircraftID"))
                coord = _normalize_coordinate(item.get("coordinate"))
                if aid is None or aid <= 0 or coord is None:
                    continue
                target_entry_by_aircraft[int(aid)] = coord
        representative_target_entry_coordinate = _normalize_coordinate(
            payload.get("representative_target_entry_coordinate")
        )
        if (
            input_package_id is None
            or input_package_id <= 0
            or current_input_id is None
            or current_input_id <= 0
            or target_input_id is None
            or target_input_id <= 0
            or not aircraft_ids
        ):
            return None
        return ExecuteNextContext(
            input_mission_package_id=int(input_package_id),
            current_input_mission_id=int(current_input_id),
            target_input_mission_id=int(target_input_id),
            target_input_mission_type=(
                int(target_input_type) if target_input_type is not None else None
            ),
            target_aircraft_ids=list(aircraft_ids),
            current_input_progress_percent=int(current_input_progress_percent),
            current_input_is_done=bool(current_input_is_done),
            current_input_recommendation_active=bool(current_input_recommendation_active),
            entry_strategy=str(entry_strategy),
            target_entry_by_aircraft=dict(target_entry_by_aircraft),
            representative_target_entry_coordinate=(
                dict(representative_target_entry_coordinate)
                if isinstance(representative_target_entry_coordinate, dict)
                else None
            ),
        )
