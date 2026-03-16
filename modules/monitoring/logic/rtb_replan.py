# -*- coding: utf-8 -*-
from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

from modules.common.option_codes import (
    DEFAULT_OPTION_CODE_SEQUENCE,
    ensure_option_code_sequence,
    option_code_to_label,
)
from modules.monitoring.logic.init_replan import allocate_mission_plan_ids, collect_input_mission_ids
from modules.monitoring.logic.mission_update import load_db_json


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _aircraft_display_label(aircraft_id: int | None) -> str:
    aid = _coerce_int(aircraft_id)
    if aid is None or aid <= 0:
        return "미상 항공기"
    if 1 <= aid <= 3:
        return f"유인기 {aid}번"
    if 4 <= aid <= 6:
        return f"무인기 {aid - 3}번"
    return f"항공기 {aid}번"


def _extract_input_package_id(mission_plan_id: int | None) -> int | None:
    if mission_plan_id is None:
        return None
    plan = load_db_json("MissionPlan", mission_plan_id)
    return _coerce_int(
        plan.get("inputMissionPackageID")
        or plan.get("InputMissionPackageID")
        or plan.get("inputMissionPackageId")
    )


def _pending_input_ids_from_package(package_id: int | None) -> list[int]:
    if package_id is None:
        return []
    payload = load_db_json("InputMissionPlan", package_id)
    missions = payload.get("inputMissionList") or []
    pending: list[int] = []
    seen: set[int] = set()
    for item in missions:
        if not isinstance(item, dict):
            continue
        if item.get("isDone"):
            continue
        mission_id = _coerce_int(item.get("inputMissionID"))
        if mission_id is None or mission_id <= 0 or mission_id in seen:
            continue
        seen.add(mission_id)
        pending.append(mission_id)
    return pending


@dataclass
class RtbReplanState:
    availability_overrides: dict[int, bool] = field(default_factory=dict)
    triggered_aircraft: set[int] = field(default_factory=set)


class RtbReplanCoordinator:
    REPLAN_LEVEL = 1
    RTB_FLIGHT_MODE = 5
    ABNORMAL_HEALTH_VALUE = 2
    UAV_IDS = (4, 5, 6)
    OPTION_CODES: tuple[int, ...] = DEFAULT_OPTION_CODE_SEQUENCE

    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._state = RtbReplanState()
        self._lock = threading.Lock()

    def get_availability_overrides(self) -> dict[int, bool]:
        return dict(self._state.availability_overrides)

    def on_agent_states(
        self,
        agent_states: Iterable[dict[str, Any]] | None,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
        aircraft_filter: Callable[[int], bool] | None = None,
        suppressed_aircraft: Iterable[int] | None = None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        with self._lock:
            logs: list[str] = []
            if not agent_states:
                return [], logs

            suppressed_ids: set[int] = set()
            for raw_id in suppressed_aircraft or ():
                aircraft_id = _coerce_int(raw_id)
                if aircraft_id is not None and aircraft_id > 0:
                    suppressed_ids.add(int(aircraft_id))

            rtb_state_by_aircraft: dict[int, dict[str, Any]] = {}
            for state in agent_states:
                if not isinstance(state, dict):
                    continue
                aircraft_id = _coerce_int(state.get("aircraft_id"))
                if aircraft_id is None or aircraft_id <= 0:
                    continue
                if not self._is_uav_state(aircraft_id, state.get("is_unmanned")):
                    continue
                trigger_type = self._classify_trigger_type(state)
                if not trigger_type:
                    continue
                if int(aircraft_id) in suppressed_ids:
                    logs.append(
                        "[0401] RTB replan suppressed: forced return already active "
                        f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id})"
                    )
                    continue
                if aircraft_filter is not None:
                    try:
                        if not bool(aircraft_filter(int(aircraft_id))):
                            continue
                    except Exception:
                        continue
                enriched_state = dict(state)
                enriched_state["_rtb_trigger_type"] = trigger_type
                rtb_state_by_aircraft[int(aircraft_id)] = enriched_state

            if not rtb_state_by_aircraft:
                return [], logs

            unique_aircraft = sorted(rtb_state_by_aircraft)
            for aircraft_id in unique_aircraft:
                self._state.availability_overrides[int(aircraft_id)] = False

            if system_mode not in (3, 4):
                logs.append(
                    f"[0401] RTB/abnormal-state detected but replan skipped: mode={system_mode} (need 3/4)"
                )
                return [], logs

            mission_ids, package_id = self._collect_pending_input_ids(current_mission_plan_id)
            payloads: list[dict[str, Any]] = []
            for aircraft_id in unique_aircraft:
                if aircraft_id in self._state.triggered_aircraft:
                    continue
                payload = self._build_replan_payload(
                    aircraft_id=int(aircraft_id),
                    state=rtb_state_by_aircraft.get(int(aircraft_id)) or {},
                    mission_ids=mission_ids,
                    package_id=package_id,
                )
                if payload is None:
                    logs.append(
                        "[0401] RTB replan skipped: missionPlanID allocation failed "
                        f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id})"
                    )
                    continue
                self._state.triggered_aircraft.add(int(aircraft_id))
                payloads.append(payload)
                detail = payload.get("replanDetail") if isinstance(payload, dict) else None
                cause = "unknown"
                trigger = "unknown"
                if isinstance(detail, dict):
                    cause = str(detail.get("rtbCause") or "unknown")
                    trigger = str(detail.get("triggerType") or "unknown")
                logs.append(
                    "[0401] RTB replan prepared "
                    f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id}, trigger={trigger}, cause={cause})"
                )

            return payloads, logs

    def _collect_pending_input_ids(
        self, current_mission_plan_id: int | None
    ) -> tuple[list[int], int | None]:
        package_id = _extract_input_package_id(current_mission_plan_id)
        pending = _pending_input_ids_from_package(package_id)
        if not pending:
            fallback: list[int] = []
            seen: set[int] = set()
            for item in collect_input_mission_ids():
                mission_id = _coerce_int(item)
                if mission_id is None or mission_id <= 0 or mission_id in seen:
                    continue
                seen.add(mission_id)
                fallback.append(mission_id)
            pending = fallback
        if not pending:
            pending = [0]
        return pending, package_id

    def _build_replan_payload(
        self,
        *,
        aircraft_id: int,
        state: dict[str, Any],
        mission_ids: list[int],
        package_id: int | None,
    ) -> dict[str, Any] | None:
        option_count = len(self.OPTION_CODES)
        option_codes = ensure_option_code_sequence(self.OPTION_CODES, option_count)
        mission_plan_ids = allocate_mission_plan_ids(option_count)
        if not mission_plan_ids:
            return None

        pending_options: list[dict[str, Any]] = []
        for code, mission_plan_id in zip(option_codes, mission_plan_ids):
            pending_options.append(
                {
                    "optionID": int(code),
                    "optionName": option_code_to_label(int(code)),
                    "missionPlanID": int(mission_plan_id),
                }
            )

        ts = int(self._now_ms())
        reason, cause = self._resolve_rtb_reason(aircraft_id=aircraft_id, state=state)
        actual_flight_mode = _coerce_int(state.get("flight_mode"))
        actual_health = _coerce_int(state.get("health"))
        trigger_type = str(state.get("_rtb_trigger_type") or self._classify_trigger_type(state) or "unexpectedRTB")
        return {
            "timestamp": ts,
            "source": "MSM",
            "inputMissionPackageID": int(package_id) if package_id is not None else 0,
            "replanRequestTime": {"replanRequestTimestamp": ts},
            "replanLevel": int(self.REPLAN_LEVEL),
            "replanRequest": reason,
            "inputMissionIDList": [{"inputMissionID": int(mission_id)} for mission_id in mission_ids],
            "pendingOptionList": pending_options,
            "replanDetail": {
                "trigger": "0401",
                "triggerType": trigger_type,
                "aircraftID": int(aircraft_id),
                "flightMode": actual_flight_mode,
                "health": actual_health,
                "rtbCause": cause,
                "fuelWarning": _coerce_int(state.get("fuel_warning")),
                "payloadHealth": _coerce_int(state.get("payload_health")),
                "reason": reason,
            },
        }

    def _classify_trigger_type(self, state: dict[str, Any] | None) -> str | None:
        flight_mode = _coerce_int((state or {}).get("flight_mode"))
        health = _coerce_int((state or {}).get("health"))
        payload_health = _coerce_int((state or {}).get("payload_health"))
        if flight_mode == self.RTB_FLIGHT_MODE:
            return "unexpectedRTB"
        if health == self.ABNORMAL_HEALTH_VALUE or payload_health == self.ABNORMAL_HEALTH_VALUE:
            return "abnormalHealthRTB"
        return None

    def _resolve_rtb_reason(
        self,
        *,
        aircraft_id: int,
        state: dict[str, Any] | None,
    ) -> tuple[str, str]:
        health = _coerce_int((state or {}).get("health"))
        payload_health = _coerce_int((state or {}).get("payload_health"))
        fuel_warning = _coerce_int((state or {}).get("fuel_warning"))
        suffix = f"{_aircraft_display_label(aircraft_id)} RTB"
        fuel_issue = fuel_warning is not None and int(fuel_warning) >= 2
        health_issue = health == self.ABNORMAL_HEALTH_VALUE
        payload_issue = payload_health == self.ABNORMAL_HEALTH_VALUE
        if fuel_issue and health_issue and payload_issue:
            return f"연료 부족 및 기체/임무장비 고장으로 인한 {suffix}", "fuel_health_payload"
        if fuel_issue and health_issue:
            return f"연료 부족 및 기체 고장으로 인한 {suffix}", "fuel_health"
        if fuel_issue and payload_issue:
            return f"연료 부족 및 임무장비 고장으로 인한 {suffix}", "fuel_payload"
        if fuel_issue:
            return f"연료 부족으로 인한 {suffix}", "fuel"
        if health_issue and payload_issue:
            return f"기체 및 임무장비 고장으로 인한 {suffix}", "health_payload"
        if health_issue:
            return f"기체 고장으로 인한 {suffix}", "health"
        if payload_issue:
            return f"임무장비 고장으로 인한 {suffix}", "payload"
        return f"비정상 상태로 인한 {suffix}", "unknown"

    def _is_uav_state(self, aircraft_id: int, is_unmanned: object) -> bool:
        if int(aircraft_id) in self.UAV_IDS:
            return True
        if isinstance(is_unmanned, bool):
            return bool(is_unmanned)
        candidate = _coerce_int(is_unmanned)
        return candidate == 1
