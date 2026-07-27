# -*- coding: utf-8 -*-
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Any, Callable

from modules.common import imaging_schedule_replan_store
from modules.monitoring.logic.init_replan import allocate_mission_plan_ids, collect_input_mission_ids
from modules.monitoring.logic.mission_update import load_db_json
from modules.monitoring.logic.replan_runtime_settings import get_imaging_schedule_settings
from modules.monitoring.logic.source_artifact_index import SourceArtifactIndex
from modules.monitoring.logic.turn_radius_monitor import TRACKED_AIRCRAFT_IDS


REPLAN_LEVEL = 3
REPLAN_REASON = "000 무인기 촬영 스케줄 미준수"
REPLAN_REASON_SEQUENCE = (
    REPLAN_REASON,
    "000 경로 스케줄 미준수로 인한 재계획",
)
OPTION_NAME = "비행/촬영"
TRIGGER_TYPE = "imagingScheduleDeviation"


def _imaging_schedule_config() -> dict[str, Any]:
    return get_imaging_schedule_settings()


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


def _extract_waypoint_id(value: object | None) -> int | None:
    if isinstance(value, dict):
        return _coerce_int(
            value.get("waypointID")
            or value.get("WaypointID")
            or value.get("waypointId")
        )
    return _coerce_int(value)


@dataclass(frozen=True)
class SourceMissionArtifacts:
    input_mission_package_id: int | None
    input_mission_ids: list[int]
    individual_mission_package_id: int
    individual_mission_id: int
    path_id: int
    current_waypoint: dict[str, Any]
    pattern_type: int | None


@dataclass
class ImagingScheduleReplanState:
    sampled_key_by_aircraft: dict[int, tuple[int, int, int]] = field(default_factory=dict)
    triggered_keys: set[tuple[int, int, int]] = field(default_factory=set)
    next_reason_index: int = 0


class ImagingScheduleReplanCoordinator:
    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
        rng: random.Random | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._rng = rng or random.Random()
        self._state = ImagingScheduleReplanState()

    def on_agent_states(
        self,
        states: list[dict[str, Any]] | None,
        *,
        enabled: bool = True,
        system_mode: int | None,
        current_mission_plan_id: int | None,
        aircraft_filter: Callable[[int], bool] | None = None,
        suppressed_aircraft: set[int] | None = None,
        dispatch_context: Any | None = None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        logs: list[str] = []
        if not enabled:
            return [], logs
        if system_mode not in (3, 4):
            return [], logs
        if current_mission_plan_id is None:
            return [], logs
        if not isinstance(states, list):
            return [], logs
        config = _imaging_schedule_config()
        trigger_probability = float(config.get("trigger_probability", 0.20))

        suppressed = {int(value) for value in (suppressed_aircraft or set())}
        payloads: list[dict[str, Any]] = []
        for state in states:
            if not isinstance(state, dict):
                continue
            aircraft_id = _coerce_int(state.get("aircraft_id") or state.get("aircraftID"))
            if aircraft_id not in TRACKED_AIRCRAFT_IDS:
                continue
            if aircraft_id in suppressed:
                continue
            if aircraft_filter is not None:
                try:
                    if not bool(aircraft_filter(int(aircraft_id))):
                        continue
                except Exception:
                    continue

            current_waypoint_id = _coerce_int(
                state.get("current_waypoint_id") or state.get("currentWaypointID")
            )
            if current_waypoint_id is None or current_waypoint_id <= 0:
                continue

            trigger_key = (
                int(current_mission_plan_id),
                int(aircraft_id),
                int(current_waypoint_id),
            )
            if self._state.sampled_key_by_aircraft.get(int(aircraft_id)) == trigger_key:
                continue
            self._state.sampled_key_by_aircraft[int(aircraft_id)] = trigger_key

            artifacts = self._resolve_source_artifacts(
                source_plan_id=int(current_mission_plan_id),
                aircraft_id=int(aircraft_id),
                current_waypoint_id=int(current_waypoint_id),
            )
            if artifacts is None:
                continue
            if not self._is_imaging_target(artifacts):
                continue

            roll = float(self._rng.random())
            if roll >= trigger_probability:
                logs.append(
                    f"[0401] imaging-schedule temp trigger skipped "
                    f"(aircraftID={aircraft_id}, currentWP={current_waypoint_id}, roll={roll:.2f})"
                )
                continue
            if trigger_key in self._state.triggered_keys:
                continue

            mission_plan_ids = allocate_mission_plan_ids(1)
            if not mission_plan_ids:
                logs.append(
                    f"[0401] imaging-schedule replan skipped: MissionPlanID allocation failed "
                    f"(aircraftID={aircraft_id})"
                )
                continue

            mission_plan_id = int(mission_plan_ids[0])
            now_ts = int(self._now_ms())
            replan_reason = self._next_replan_reason()
            payload = self._build_payload(
                timestamp_ms=now_ts,
                mission_plan_id=mission_plan_id,
                source_plan_id=int(current_mission_plan_id),
                aircraft_id=int(aircraft_id),
                current_waypoint_id=int(current_waypoint_id),
                artifacts=artifacts,
                roll=roll,
                replan_reason=replan_reason,
            )
            detail_payload = dict(payload.get("replanDetail") or {})
            self._persist_detail(int(mission_plan_id), detail_payload)
            payloads.append(payload)
            self._state.triggered_keys.add(trigger_key)
            logs.append(
                f"[0401] imaging-schedule replan prepared "
                f"(aircraftID={aircraft_id}, currentWP={current_waypoint_id}, missionPlanID={mission_plan_id}, "
                f"reason={replan_reason}, roll={roll:.2f})"
            )

        return payloads, logs

    def _build_payload(
        self,
        *,
        timestamp_ms: int,
        mission_plan_id: int,
        source_plan_id: int,
        aircraft_id: int,
        current_waypoint_id: int,
        artifacts: SourceMissionArtifacts,
        roll: float,
        replan_reason: str,
    ) -> dict[str, Any]:
        input_ids = list(artifacts.input_mission_ids or [])
        if not input_ids:
            input_ids = [0]
        coordinate = dict((artifacts.current_waypoint or {}).get("coordinate") or {})
        detail_payload = {
            "trigger": "0401",
            "triggerType": TRIGGER_TYPE,
            "sourceMissionPlanID": int(source_plan_id),
            "aircraftID": int(aircraft_id),
            "inputMissionPackageID": int(artifacts.input_mission_package_id)
            if artifacts.input_mission_package_id is not None
            else None,
            "individualMissionPackageID": int(artifacts.individual_mission_package_id),
            "individualMissionID": int(artifacts.individual_mission_id),
            "pathID": int(artifacts.path_id),
            "currentWaypointID": int(current_waypoint_id),
            "currentWaypointCoordinate": {
                "latitude": _coerce_float(coordinate.get("latitude")),
                "longitude": _coerce_float(coordinate.get("longitude")),
                "altitude": _coerce_float(coordinate.get("altitude")),
            },
            "patternType": _coerce_int(artifacts.pattern_type),
            "sensorType": _coerce_int((artifacts.current_waypoint or {}).get("sensorType")),
            "operationMode": _coerce_int((artifacts.current_waypoint or {}).get("operationMode")),
            "triggerProbability": float(_imaging_schedule_config().get("trigger_probability", 0.20)),
            "triggerRoll": float(roll),
            "selectedReplanReason": str(replan_reason),
            "timestamp": int(timestamp_ms),
        }
        option_block = [
            {
                "optionID": 1,
                "optionName": OPTION_NAME,
                "missionPlanID": int(mission_plan_id),
            }
        ]
        return {
            "timestamp": int(timestamp_ms),
            "source": "MSM",
            "inputMissionPackageID": int(artifacts.input_mission_package_id)
            if artifacts.input_mission_package_id is not None
            else 0,
            "replanRequestTime": {"replanRequestTimestamp": int(timestamp_ms)},
            "replanLevel": int(REPLAN_LEVEL),
            "replanRequest": str(replan_reason),
            "replanReason": str(replan_reason),
            "inputMissionIDList": [{"inputMissionID": int(mission_id)} for mission_id in input_ids],
            "individualMissionIDList": [
                {"individualMissionID": int(artifacts.individual_mission_id)}
            ],
            "pendingOptionList": option_block,
            "replanDetail": detail_payload,
        }

    def _next_replan_reason(self) -> str:
        reasons = tuple(str(value) for value in REPLAN_REASON_SEQUENCE if str(value).strip())
        if not reasons:
            return REPLAN_REASON
        index = int(self._state.next_reason_index) % len(reasons)
        self._state.next_reason_index = int(self._state.next_reason_index) + 1
        return reasons[index]

    def _is_imaging_target(self, artifacts: SourceMissionArtifacts) -> bool:
        config = _imaging_schedule_config()
        operation_modes = {
            int(value)
            for value in (config.get("imaging_operation_modes") or [1, 2, 3, 4, 5])
            if _coerce_int(value) is not None
        }
        pattern_types = {
            int(value)
            for value in (config.get("imaging_pattern_types") or [3, 4, 5, 6, 7, 8, 9])
            if _coerce_int(value) is not None
        }
        waypoint = dict(artifacts.current_waypoint or {})
        sensor_type = _coerce_int(waypoint.get("sensorType"))
        if sensor_type is not None and sensor_type > 0:
            return True
        operation_mode = _coerce_int(waypoint.get("operationMode"))
        if operation_mode in operation_modes:
            return True
        return _coerce_int(artifacts.pattern_type) in pattern_types

    def _resolve_source_artifacts(
        self,
        *,
        source_plan_id: int,
        aircraft_id: int,
        current_waypoint_id: int,
    ) -> SourceMissionArtifacts | None:
        artifact_index = SourceArtifactIndex.from_source_plan(int(source_plan_id))
        if artifact_index is None:
            return None

        input_mission_package_id = artifact_index.input_mission_package_id
        imp_id = artifact_index.individual_package_id_for_aircraft(int(aircraft_id))
        if imp_id is None or imp_id <= 0:
            return None

        for mission in artifact_index.individual_missions(int(imp_id)):
            if not isinstance(mission, dict):
                continue
            path_id = _coerce_int(mission.get("pathID"))
            individual_mission_id = _coerce_int(mission.get("individualMissionID"))
            if path_id is None or path_id <= 0 or individual_mission_id is None or individual_mission_id <= 0:
                continue
            current_waypoint = artifact_index.waypoint_for_path(int(path_id), int(current_waypoint_id))
            if not isinstance(current_waypoint, dict):
                continue

            related = mission.get("relatedMission") or {}
            input_mission_ids: list[int] = []
            related_input_mission_id = _coerce_int(
                related.get("inputMissionID")
                if isinstance(related, dict)
                else None
            )
            if related_input_mission_id is not None and related_input_mission_id > 0:
                input_mission_ids = [int(related_input_mission_id)]
            if not input_mission_ids and input_mission_package_id is not None:
                input_plan = load_db_json("InputMissionPlan", input_mission_package_id)
                seen_ids: set[int] = set()
                for item in input_plan.get("inputMissionList") or []:
                    if not isinstance(item, dict):
                        continue
                    mission_id = _coerce_int(item.get("inputMissionID"))
                    if mission_id is None or mission_id <= 0 or mission_id in seen_ids:
                        continue
                    if bool(item.get("isDone")):
                        continue
                    seen_ids.add(int(mission_id))
                    input_mission_ids.append(int(mission_id))
            if not input_mission_ids:
                input_mission_ids = [int(value) for value in collect_input_mission_ids() if _coerce_int(value)]
            if not input_mission_ids:
                input_mission_ids = [0]

            info = mission.get("individualMissionInfo") or {}
            pattern_type = _coerce_int(info.get("patternType")) if isinstance(info, dict) else None
            return SourceMissionArtifacts(
                input_mission_package_id=input_mission_package_id,
                input_mission_ids=input_mission_ids,
                individual_mission_package_id=int(imp_id),
                individual_mission_id=int(individual_mission_id),
                path_id=int(path_id),
                current_waypoint=dict(current_waypoint),
                pattern_type=pattern_type,
            )
        return None

    def _persist_detail(self, mission_plan_id: int, detail_payload: dict[str, Any]) -> None:
        try:
            detail_path = imaging_schedule_replan_store.save_detail(int(mission_plan_id), detail_payload)
        except Exception:
            detail_path = None
        try:
            imaging_schedule_replan_store.save_event(
                "monitor_dispatch",
                {
                    "missionPlanID": int(mission_plan_id),
                    "detailPath": str(detail_path) if detail_path is not None else None,
                    "detail": dict(detail_payload or {}),
                },
            )
        except Exception:
            pass
