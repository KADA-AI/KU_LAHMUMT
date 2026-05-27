# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Iterable, Callable

from modules.common import db_paths, prior_replan_store
from modules.common import prior_target_rediscovery_store
from modules.monitoring.logic.init_replan import (
    allocate_mission_plan_ids,
    collect_input_mission_ids,
)
from modules.monitoring.logic.mission_update import parse_payload
from modules.monitoring.logic.replan_runtime_settings import (
    get_post_attack_rejoin_settings,
    get_prior_mission_settings,
)
from modules.mission_planning.runtime.attack_tracking_state import resolve_plan_lineage_ids
from modules.mission_planning.runtime.prior_tracking_state import (
    list_active_prior_assignments,
    mark_prior_handoff,
)


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


def _coerce_altitude_int(value: object) -> int | None:
    altitude = _coerce_float(value)
    if altitude is None:
        return None
    return int(round(float(altitude)))


def _normalize_coordinate(value: object) -> dict[str, Any]:
    if not isinstance(value, dict):
        return {}
    lat = _coerce_float(value.get("latitude") or value.get("lat"))
    lon = _coerce_float(value.get("longitude") or value.get("lon"))
    alt = _coerce_altitude_int(value.get("altitude") or value.get("alt"))
    if lat is None or lon is None:
        return {}
    payload: dict[str, Any] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        payload["altitude"] = int(alt)
    return payload


def _normalize_positive_int_list(value: object) -> list[int]:
    source = value if isinstance(value, (list, tuple, set)) else []
    out: list[int] = []
    seen: set[int] = set()
    for item in source:
        parsed = _coerce_int(item)
        if parsed is None or parsed <= 0 or parsed in seen:
            continue
        seen.add(int(parsed))
        out.append(int(parsed))
    return out


def _load_path_waypoint_summary(path_id: object) -> tuple[list[int], int | None]:
    parsed_path_id = _coerce_int(path_id)
    if parsed_path_id is None or parsed_path_id <= 0:
        return [], None
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{int(parsed_path_id)}.json")
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return [], None
    waypoints = payload.get("waypointList") if isinstance(payload, dict) else []
    if not isinstance(waypoints, list):
        return [], None
    waypoint_ids: list[int] = []
    first_active_waypoint_id: int | None = None
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        waypoint_id = _coerce_int(waypoint.get("waypointID"))
        if waypoint_id is None or waypoint_id <= 0:
            continue
        waypoint_ids.append(int(waypoint_id))
        if first_active_waypoint_id is None and not bool(waypoint.get("isDone")):
            first_active_waypoint_id = int(waypoint_id)
    if first_active_waypoint_id is None:
        first_active_waypoint_id = waypoint_ids[0] if waypoint_ids else None
    return waypoint_ids, first_active_waypoint_id


def _extract_state_waypoint_id(value: object) -> int | None:
    if not isinstance(value, dict):
        return None
    for container in (value, value.get("unmannedInfo"), value.get("flightMode")):
        if not isinstance(container, dict):
            continue
        for key in ("current_waypoint_id", "currentWaypointID", "CurrentWaypointID", "currentWaypointId"):
            if key not in container:
                continue
            raw = container.get(key)
            if isinstance(raw, dict):
                for sub_key in ("waypointID", "WaypointID", "waypointId"):
                    parsed = _coerce_int(raw.get(sub_key))
                    if parsed is not None and parsed > 0:
                        return int(parsed)
                continue
            parsed = _coerce_int(raw)
            if parsed is not None and parsed > 0:
                return int(parsed)
    return None


def _extract_state_coordinate(value: object) -> dict[str, Any]:
    if not isinstance(value, dict):
        return {}
    return (
        _normalize_coordinate(value.get("coordinate"))
        or _normalize_coordinate(value.get("Coordinate"))
        or _normalize_coordinate((value.get("unmannedInfo") or {}).get("coordinate"))
        or _normalize_coordinate((value.get("unmannedInfo") or {}).get("Coordinate"))
        or _normalize_coordinate((value.get("mannedInfo") or {}).get("coordinate"))
        or _normalize_coordinate((value.get("mannedInfo") or {}).get("Coordinate"))
    )


def _build_prior_close_reason(prior_mission_id: int, aircraft_id: int) -> str:
    return f"UAV{int(aircraft_id)} 선행임무 종료"


def _extract_timestamp(plan: dict[str, Any]) -> int | None:
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in plan:
            return _coerce_int(plan.get(key))
    return None


def _extract_source(plan: dict[str, Any]) -> str:
    for key in ("source", "Source", "sourceModuleName", "SourceModuleName"):
        if key in plan:
            try:
                text = str(plan.get(key))
            except Exception:
                text = ""
            if text:
                return text
    return "MMR"


def _extract_prior_entries(plan: dict[str, Any]) -> list[dict[str, Any]]:
    raw_list = plan.get("priorMissionList") or plan.get("PriorMissionList") or []
    entries: list[dict[str, Any]] = []
    for item in raw_list:
        if not isinstance(item, dict):
            continue
        prior_id = _coerce_int(item.get("priorMissionID") or item.get("PriorMissionID"))
        mission_type = _coerce_int(item.get("missionType") or item.get("MissionType"))
        if prior_id is None or mission_type is None:
            continue
        entries.append(
            {
                "priorMissionID": prior_id,
                "missionType": mission_type,
                "coordinateOrientation": item.get("coordinateOrientation")
                or item.get("CoordinateOrientation"),
                "targetOrientation": item.get("targetOrientation")
                or item.get("TargetOrientation"),
                "rawEntry": item,
            }
        )
    return entries


def _extract_coordinate(entry: dict[str, Any]) -> dict[str, Any]:
    coord_block = entry.get("coordinateOrientation") or {}
    coordinate = {}
    if isinstance(coord_block, dict):
        coordinate = coord_block.get("coordinate") or coord_block.get("Coordinate") or {}
    if not isinstance(coordinate, dict):
        return {}

    def _pick(keys: Iterable[str]) -> object | None:
        for key in keys:
            if key in coordinate:
                return coordinate.get(key)
        return None

    # NOTE: do not use "or" here; 0.0 is a valid value but falsy.
    lat = _coerce_float(_pick(("latitude", "Latitude")))
    lon = _coerce_float(_pick(("longitude", "Longitude")))
    alt = _coerce_altitude_int(_pick(("altitude", "Altitude")))
    out: dict[str, Any] = {}
    if lat is not None:
        out["latitude"] = lat
    if lon is not None:
        out["longitude"] = lon
    if alt is not None:
        out["altitude"] = alt
    return out


def _extract_target_id(entry: dict[str, Any]) -> int | None:
    orient = entry.get("targetOrientation") or {}
    if not isinstance(orient, dict):
        return None
    for key in ("targetID", "TargetID", "targetId", "TargetId"):
        value = _coerce_int(orient.get(key))
        if value is not None and value > 0:
            return value
    return None


MISSION_TYPE_LABELS: dict[int, str] = {
    1: "좌표지정",
    2: "표적추적",
}


@dataclass
class PriorMissionState:
    handled_prior_ts: dict[int, int] = field(default_factory=dict)
    handled_close_ts: dict[str, int] = field(default_factory=dict)


class PriorMissionReplanCoordinator:
    """Handle 0202 PriorMissionInfo inputs and emit level-4 0902 requests."""

    OPTION_LABEL = "선행임무 반영"
    REPLAN_LEVEL = 4
    DL_RISK_REPLAN_LEVEL = 5

    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._state = PriorMissionState()

    def on_prior_mission(
        self,
        payload: object | None,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        logs: list[str] = []
        if system_mode not in (3, 4):
            logs.append(f"[PRIOR] skipped: mode={system_mode} (need 3/4)")
            return [], logs

        plan = parse_payload(payload)
        if not plan:
            logs.append("[PRIOR] skipped: PriorMissionInfo payload parse failed or empty")
            return [], logs

        source = _extract_source(plan)
        message_ts = _extract_timestamp(plan)
        if message_ts is None:
            message_ts = int(self._now_ms())

        entries = _extract_prior_entries(plan)
        if not entries:
            logs.append(
                "[PRIOR] skipped: no valid priorMissionList entries "
                f"(source={source}, timestamp={message_ts})"
            )
            return [], logs

        mission_ids = collect_input_mission_ids()
        if not mission_ids:
            mission_ids = [0]
        input_models = [{"inputMissionID": int(mid)} for mid in mission_ids]

        dispatch_payloads: list[dict[str, Any]] = []
        for entry in entries:
            prior_id = int(entry["priorMissionID"])
            mission_type = int(entry["missionType"])
            target_id = _extract_target_id(entry)

            last_ts = self._state.handled_prior_ts.get(prior_id)
            if last_ts is not None and int(message_ts) <= int(last_ts):
                logs.append(
                    f"[PRIOR] skip priorMissionID={prior_id}: duplicate/stale timestamp "
                    f"(message={int(message_ts)}, last={int(last_ts)})"
                )
                continue
            if mission_type == 2 and target_id is None:
                logs.append(
                    f"[PRIOR] skip priorMissionID={prior_id}: "
                    "missionType=2 requires targetOrientation.targetID"
                )
                continue

            now_ts = int(self._now_ms())
            mission_plan_id = self._allocate_plan_id(now_ts)
            reason = self._build_reason(mission_type)
            coordinate = _extract_coordinate(entry)
            target_orientation_payload = (
                {"targetID": int(target_id)} if target_id is not None else {}
            )
            coordinate_orientation_payload = (
                {"coordinate": dict(coordinate)} if coordinate else {}
            )
            rediscovery_arm = prior_target_rediscovery_store.arm_target_rediscovery(
                prior_mission_id=prior_id,
                mission_type=mission_type,
                timestamp=now_ts,
                target_id=target_id,
                coordinate=coordinate,
            )
            if rediscovery_arm:
                arm_target_label = f"targetID={target_id}" if target_id is not None else "coordinate-arm"
                logs.append(
                    f"[PRIOR] rediscovery arm set: priorMissionID={prior_id}, "
                    f"missionType={mission_type}, {arm_target_label}"
                )

            detail_payload = {
                "sourceMissionPlanID": current_mission_plan_id,
                "priorMissionID": prior_id,
                "missionType": mission_type,
                "targetCoordinate": coordinate,
                "targetOrientation": target_orientation_payload,
                "targetID": int(target_id) if target_id is not None else None,
                "timestamp": now_ts,
                "rawEntry": entry.get("rawEntry") or {},
            }
            # Persist detail for the prior-mission pipeline without adding
            # non-ICD fields into the outgoing 0902 payload.
            self._persist_detail(mission_plan_id, detail_payload)

            # NOTE: the current mission-planning prior pipeline expects exactly
            # one plan option to carry the missionPlanID, even though this is
            # not a user-facing multi-option flow.
            option_block = [
                {
                    "optionID": 1,
                    "optionName": self.OPTION_LABEL,
                    "missionPlanID": int(mission_plan_id),
                }
            ]
            prior_item: dict[str, Any] = {
                "priorMissionID": prior_id,
                "missionType": mission_type,
            }
            if coordinate_orientation_payload:
                prior_item["coordinateOrientation"] = coordinate_orientation_payload
            if target_orientation_payload:
                prior_item["targetOrientation"] = target_orientation_payload
            prior_block = [prior_item]

            payload_0902: dict[str, Any] = {
                "timestamp": now_ts,
                "source": "MSM",
                "replanRequestTime": {"replanRequestTimestamp": now_ts},
                "replanLevel": int(self.REPLAN_LEVEL),
                "replanRequest": reason,
                "inputMissionIDList": input_models,
                "priorMissionList": prior_block,
                "pendingOptionList": option_block,
                "replanDetail": detail_payload,
            }
            dispatch_payloads.append(payload_0902)
            self._state.handled_prior_ts[prior_id] = int(message_ts)
            logs.append(
                "[PRIOR] 0202 processed -> dispatching 0902"
                f" (priorMissionID={prior_id}, missionPlanID={mission_plan_id})"
            )

        return dispatch_payloads, logs

    def on_agent_states(
        self,
        agent_states: object | None,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        logs: list[str] = []
        if system_mode not in (3, 4):
            return [], logs
        if not isinstance(agent_states, list) or not agent_states:
            return [], logs
        if current_mission_plan_id is None or current_mission_plan_id <= 0:
            return [], logs

        current_plan_lineage = resolve_plan_lineage_ids(int(current_mission_plan_id)) or {
            int(current_mission_plan_id)
        }
        state_index: dict[int, dict[str, Any]] = {}
        for entry in agent_states:
            if not isinstance(entry, dict):
                continue
            aircraft_id = _coerce_int(entry.get("aircraft_id") or entry.get("aircraftID"))
            if aircraft_id is None or aircraft_id <= 0:
                continue
            state_index[int(aircraft_id)] = dict(entry)

        close_cfg = get_post_attack_rejoin_settings()
        close_cooldown_ms = max(
            0,
            _coerce_int(close_cfg.get("closure_cooldown_ms")) or 30000,
        )
        dispatch_payloads: list[dict[str, Any]] = []
        now_ts = int(self._now_ms())
        active_assignments = [
            dict(item)
            for item in list_active_prior_assignments()
            if isinstance(item, dict) and bool(item.get("active"))
        ]
        if not active_assignments:
            return dispatch_payloads, logs

        for assignment in active_assignments:
            aircraft_id = _coerce_int(assignment.get("aircraft_id"))
            prior_plan_id = _coerce_int(assignment.get("prior_plan_id"))
            if aircraft_id is None or aircraft_id <= 0:
                continue
            if prior_plan_id is None or prior_plan_id <= 0:
                continue
            if int(prior_plan_id) not in current_plan_lineage:
                continue

            state_entry = state_index.get(int(aircraft_id))
            if not isinstance(state_entry, dict):
                continue

            prior_waypoint_list = _normalize_positive_int_list(assignment.get("prior_waypoint_ids"))
            prior_waypoint_ids = set(prior_waypoint_list)
            if not prior_waypoint_ids:
                continue

            previous_wp = _coerce_int(
                assignment.get("last_nonzero_waypoint_id")
                or assignment.get("original_current_waypoint_id")
            )
            current_wp = _extract_state_waypoint_id(state_entry)
            if previous_wp is None or previous_wp <= 0:
                continue
            if current_wp is None or current_wp <= 0:
                continue

            resume_waypoint_list = _normalize_positive_int_list(assignment.get("resume_waypoint_ids"))
            resume_first_active_waypoint_id = _coerce_int(assignment.get("resume_first_active_waypoint_id"))
            if not resume_waypoint_list or resume_first_active_waypoint_id is None or resume_first_active_waypoint_id <= 0:
                loaded_resume_waypoints, loaded_resume_active_waypoint_id = _load_path_waypoint_summary(
                    assignment.get("resume_path_id")
                )
                if not resume_waypoint_list:
                    resume_waypoint_list = list(loaded_resume_waypoints)
                if resume_first_active_waypoint_id is None or resume_first_active_waypoint_id <= 0:
                    resume_first_active_waypoint_id = loaded_resume_active_waypoint_id
            resume_waypoint_ids = set(resume_waypoint_list)
            resume_first_waypoint_id = _coerce_int(assignment.get("resume_first_waypoint_id"))
            handoff_waypoint_id = _coerce_int(assignment.get("handoff_waypoint_id"))
            wp_changed = int(current_wp) != int(previous_wp)

            current_in_resume = (
                int(current_wp) in resume_waypoint_ids
                or (
                    resume_first_active_waypoint_id is not None
                    and resume_first_active_waypoint_id > 0
                    and int(current_wp) == int(resume_first_active_waypoint_id)
                )
                or (
                    resume_first_waypoint_id is not None
                    and resume_first_waypoint_id > 0
                    and int(current_wp) == int(resume_first_waypoint_id)
                )
            )
            observed_prior_exit = (
                wp_changed
                and int(previous_wp) in prior_waypoint_ids
                and int(current_wp) not in prior_waypoint_ids
                and current_in_resume
            )
            fallback_prior_close_wp = (
                int(prior_waypoint_list[-1])
                if prior_waypoint_list
                else int(previous_wp)
            )
            direct_resume_entry = (
                wp_changed
                and current_in_resume
                and int(current_wp) not in prior_waypoint_ids
            )
            late_resume_presence = (
                not wp_changed
                and handoff_waypoint_id is None
                and current_in_resume
                and int(current_wp) not in prior_waypoint_ids
            )
            if not observed_prior_exit and not direct_resume_entry and not late_resume_presence:
                continue
            transition_previous_wp = (
                int(previous_wp)
                if observed_prior_exit
                else int(fallback_prior_close_wp)
            )

            dedupe_key = (
                f"{int(prior_plan_id)}|{int(aircraft_id)}|{int(transition_previous_wp)}|{int(current_wp)}|priorClose"
            )
            prev_ts = self._state.handled_close_ts.get(dedupe_key)
            if prev_ts is not None and (int(now_ts) - int(prev_ts)) < int(close_cooldown_ms):
                continue
            self._state.handled_close_ts[dedupe_key] = int(now_ts)

            current_coord = _extract_state_coordinate(state_entry)
            if not current_coord:
                current_coord = _normalize_coordinate(
                    assignment.get("last_nonzero_coordinate") or assignment.get("original_coordinate")
                )
            mark_prior_handoff(
                int(aircraft_id),
                handoff_waypoint_id=int(transition_previous_wp),
                handoff_coordinate=current_coord or None,
            )

            plan_ids = allocate_mission_plan_ids(1)
            if not plan_ids:
                logs.append(
                    f"[PRIOR-CLOSE] missionPlanID allocation failed; skip aircraft={int(aircraft_id)}"
                )
                continue

            current_input_id = _coerce_int(assignment.get("current_input_mission_id"))
            if current_input_id is not None and current_input_id > 0:
                mission_ids = [int(current_input_id)]
            else:
                mission_ids = [
                    int(mid)
                    for mid in (_coerce_int(item) for item in (collect_input_mission_ids() or [0]))
                    if mid is not None and mid >= 0
                ] or [0]
            input_models = [{"inputMissionID": int(mid)} for mid in mission_ids]
            prior_mission_id = _coerce_int(assignment.get("prior_mission_id")) or 0
            mission_type = _coerce_int(assignment.get("mission_type"))
            detail_payload = {
                "trigger": "0401",
                "triggerType": "priorClosedResume",
                "closureReason": "resumeWaypointEntered",
                "sourceMissionPlanID": int(assignment.get("source_plan_id") or current_mission_plan_id),
                "currentMissionPlanID": int(current_mission_plan_id),
                "missionPlanID": int(plan_ids[0]),
                "aircraftID": int(aircraft_id),
                "priorMissionID": int(prior_mission_id),
                "missionType": int(mission_type) if mission_type is not None else None,
                "currentInputMissionID": int(current_input_id) if current_input_id is not None else None,
                "priorPlanID": int(prior_plan_id),
                "previousWaypointID": int(transition_previous_wp),
                "currentWaypointID": int(current_wp),
                "resumeFirstWaypointID": int(resume_first_waypoint_id)
                if resume_first_waypoint_id is not None
                else None,
                "returningAircraftIDList": [int(aircraft_id)],
                "targetID": _coerce_int(assignment.get("target_id")),
                "targetCoordinate": _normalize_coordinate(assignment.get("target_coordinate")),
                "coordinate": dict(current_coord),
                "timestamp": int(now_ts),
            }
            dispatch_payloads.append(
                {
                    "timestamp": int(now_ts),
                    "source": "MSM",
                    "replanRequestTime": {"replanRequestTimestamp": int(now_ts)},
                    "replanLevel": int(self.REPLAN_LEVEL),
                    "replanRequest": _build_prior_close_reason(int(prior_mission_id), int(aircraft_id)),
                    "inputMissionIDList": input_models,
                    "missionPlanIDList": [{"missionPlanID": int(plan_ids[0])}],
                    "replanDetail": detail_payload,
                }
            )
            logs.append(
                "[PRIOR-CLOSE] prior mission closed -> post-rejoin request queued "
                f"(aircraft={int(aircraft_id)}, priorMissionID={int(prior_mission_id)}, "
                f"wp={int(transition_previous_wp)}->{int(current_wp)})"
            )

        return dispatch_payloads, logs

    def on_risk_update(
        self,
        risk_score: float,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
        risky_aircraft_ids: list[int] | None = None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        """Handle DL-based risk updates and emit level-5 0902 requests."""
        logs: list[str] = []
        if system_mode not in (3, 4):
            return [], logs
        config = get_prior_mission_settings()
        if risk_score <= float(config.get("dl_risk_threshold", 0.5)):
            return [], logs

        now_ts = int(self._now_ms())
        mission_plan_id = self._allocate_plan_id(now_ts)

        risky_ids_str = ",".join(map(str, sorted(risky_aircraft_ids))) if risky_aircraft_ids else "Unknown"
        reason = f"위험도 높음 AC:{risky_ids_str}"

        mission_ids = collect_input_mission_ids()
        if not mission_ids:
            mission_ids = [0]
        input_models = [{"inputMissionID": int(mid)} for mid in mission_ids]

        option_block = [
            {
                "optionID": 1,
                "optionName": "위험 회피",
                "missionPlanID": int(mission_plan_id),
            }
        ]
        payload_0902: dict[str, Any] = {
            "timestamp": now_ts,
            "source": "MSM",
            "replanRequestTime": {"replanRequestTimestamp": now_ts},
            "replanLevel": int(self.DL_RISK_REPLAN_LEVEL),
            "replanRequest": reason,
            "inputMissionIDList": input_models,
            "priorMissionList": [],
            "pendingOptionList": option_block,
            "replanDetail": {
                "sourceMissionPlanID": current_mission_plan_id,
                "riskScore": float(risk_score),
                "riskyAircraftIDList": [int(aid) for aid in (risky_aircraft_ids or [])],
                "timestamp": now_ts,
            },
        }
        logs.append(f"[DL] High Risk ({risk_score:.2f}, AC:{risky_ids_str}) -> dispatching 0902")
        return [payload_0902], logs

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _allocate_plan_id(self, now_ts: int) -> int:
        allocated = allocate_mission_plan_ids(1)
        if allocated:
            try:
                return int(allocated[0])
            except Exception:
                pass
        # Fallback seed to avoid blocking the flow on allocator failure.
        return int(700_000_000 + (int(now_ts) % 1_000))

    def _build_reason(self, mission_type: int) -> str:
        label = MISSION_TYPE_LABELS.get(int(mission_type))
        if label:
            return f"선행임무:{label}"
        return f"선행임무:{int(mission_type)}"

    def _persist_detail(self, mission_plan_id: int, detail_payload: dict[str, Any]) -> Any | None:
        try:
            return prior_replan_store.save_detail(int(mission_plan_id), detail_payload)
        except Exception:
            return None
