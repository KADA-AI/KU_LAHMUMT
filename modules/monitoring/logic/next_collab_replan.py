from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from modules.common import next_collab_replan_store
from modules.monitoring.logic.init_replan import allocate_mission_plan_ids
from modules.monitoring.logic.replan_runtime_settings import get_next_collab_settings


REPLAN_LEVEL = 3
REPLAN_REASON = "조기 임무 전환으로 인한 재계획"
OPTION_NAME = "비행/촬영"
TRIGGER_TYPE = "nextCollaborativeMission"
ENTRY_LEAD_TIME_S = 5.0


def _next_collab_config() -> dict[str, Any]:
    return get_next_collab_settings()


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


def _normalize_coordinate(payload: object | None) -> dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _coerce_float(payload.get("latitude"))
    lon = _coerce_float(payload.get("longitude"))
    alt = _coerce_float(payload.get("altitude"))
    if lat is None or lon is None:
        return None
    coord: dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        coord["altitude"] = float(alt)
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
        out["altitude"] = sum(alt_vals) / float(len(alt_vals))
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
        coord["altitude"] = (float(alt1) + float(alt2)) / 2.0
    elif alt1 is not None:
        coord["altitude"] = float(alt1)
    elif alt2 is not None:
        coord["altitude"] = float(alt2)
    return coord


@dataclass(frozen=True)
class ExecuteNextContext:
    input_mission_package_id: int
    current_input_mission_id: int
    target_input_mission_id: int
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

        entries: list[dict[str, Any]] = []
        coords_for_centroid: list[dict[str, float]] = []
        views = turn_views if isinstance(turn_views, dict) else {}
        use_turn_projection = str(context.entry_strategy or "").strip().lower() == "turn_projection"
        for aircraft_id in context.target_aircraft_ids:
            view = views.get(int(aircraft_id))
            coord = None
            source = None
            eta_s = None
            if view is not None:
                position_coord = _normalize_coordinate(getattr(view, "position_coordinate", None))
                target_entry_coord = context.target_entry_by_aircraft.get(int(aircraft_id))
                if target_entry_coord is None:
                    target_entry_coord = context.representative_target_entry_coordinate
                if use_turn_projection:
                    coord = _normalize_coordinate(getattr(view, "predicted_entry_coordinate", None))
                    eta_s = _coerce_float(getattr(view, "predicted_entry_eta_s", None))
                    if coord is not None:
                        source = "turnProjection5s"
                    if coord is None:
                        coord = _normalize_coordinate(getattr(view, "alternate_waypoint_coordinate", None))
                        eta_s = _coerce_float(getattr(view, "alternate_waypoint_eta_s", None))
                        if coord is not None:
                            source = "altWaypoint"
                    if coord is None:
                        coord = position_coord
                        eta_s = 0.0
                        if coord is not None:
                            source = "currentPosition"
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
            coords_for_centroid.append(coord)
            entry: dict[str, Any] = {
                "aircraftID": int(aircraft_id),
                "coordinate": coord,
                "source": str(source or "unknown"),
            }
            if eta_s is not None:
                entry["etaS"] = float(eta_s)
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
            "entryLeadTimeS": float(_next_collab_config().get("entry_lead_time_s", ENTRY_LEAD_TIME_S)),
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
        config = _next_collab_config()
        input_package_id = _coerce_int(payload.get("input_mission_package_id"))
        current_input_id = _coerce_int(payload.get("current_input_mission_id"))
        target_input_id = _coerce_int(payload.get("target_input_mission_id"))
        current_input_progress_percent = _coerce_int(payload.get("current_input_progress_percent")) or 0
        current_input_is_done = bool(payload.get("current_input_is_done"))
        current_input_recommendation_active = bool(payload.get("current_input_recommendation_active"))
        default_entry_strategy = str(config.get("default_entry_strategy") or "midpoint_to_next_start").strip().lower()
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
