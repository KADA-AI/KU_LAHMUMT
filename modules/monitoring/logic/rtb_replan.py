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
    ) -> tuple[list[dict[str, Any]], list[str]]:
        with self._lock:
            logs: list[str] = []
            if not agent_states:
                return [], logs

            rtb_aircraft: list[int] = []
            for state in agent_states:
                if not isinstance(state, dict):
                    continue
                aircraft_id = _coerce_int(state.get("aircraft_id"))
                if aircraft_id is None or aircraft_id <= 0:
                    continue
                if not self._is_uav_state(aircraft_id, state.get("is_unmanned")):
                    continue
                flight_mode = _coerce_int(state.get("flight_mode"))
                if flight_mode != self.RTB_FLIGHT_MODE:
                    continue
                if aircraft_filter is not None:
                    try:
                        if not bool(aircraft_filter(int(aircraft_id))):
                            continue
                    except Exception:
                        continue
                rtb_aircraft.append(int(aircraft_id))

            if not rtb_aircraft:
                return [], logs

            unique_aircraft = sorted(set(rtb_aircraft))
            for aircraft_id in unique_aircraft:
                self._state.availability_overrides[int(aircraft_id)] = False

            if system_mode not in (3, 4):
                logs.append(f"[0401] RTB detected but replan skipped: mode={system_mode} (need 3/4)")
                return [], logs

            mission_ids, package_id = self._collect_pending_input_ids(current_mission_plan_id)
            payloads: list[dict[str, Any]] = []
            for aircraft_id in unique_aircraft:
                if aircraft_id in self._state.triggered_aircraft:
                    continue
                payload = self._build_replan_payload(
                    aircraft_id=int(aircraft_id),
                    mission_ids=mission_ids,
                    package_id=package_id,
                )
                if payload is None:
                    logs.append(
                        f"[0401] RTB replan skipped: missionPlanID allocation failed (aircraftID={aircraft_id})"
                    )
                    continue
                self._state.triggered_aircraft.add(int(aircraft_id))
                payloads.append(payload)
                logs.append(f"[0401] RTB replan prepared (aircraftID={aircraft_id})")

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
        reason = f"임무장비 고장으로 인한 {int(aircraft_id):02d}번 무인기 RTB"
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
                "triggerType": "unexpectedRTB",
                "aircraftID": int(aircraft_id),
                "flightMode": int(self.RTB_FLIGHT_MODE),
                "reason": reason,
            },
        }

    def _is_uav_state(self, aircraft_id: int, is_unmanned: object) -> bool:
        if int(aircraft_id) in self.UAV_IDS:
            return True
        if isinstance(is_unmanned, bool):
            return bool(is_unmanned)
        candidate = _coerce_int(is_unmanned)
        return candidate == 1
