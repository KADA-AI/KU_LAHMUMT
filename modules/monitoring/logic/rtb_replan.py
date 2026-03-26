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
from modules.monitoring.logic.replan_runtime_settings import get_rtb_settings


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
class PendingRtbTrigger:
    first_seen_ms: int
    trigger_type: str
    cause: str
    reason: str


@dataclass
class RtbReplanState:
    availability_overrides: dict[int, bool] = field(default_factory=dict)
    triggered_aircraft: set[int] = field(default_factory=set)
    pending_by_aircraft: dict[int, PendingRtbTrigger] = field(default_factory=dict)


class RtbReplanCoordinator:
    REPLAN_LEVEL = 1
    HEALTH_UNAVAILABLE_TRIGGER = "abnormalHealthUnavailable"
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

    @staticmethod
    def _config() -> dict[str, Any]:
        return get_rtb_settings()

    def on_agent_states(
        self,
        agent_states: Iterable[dict[str, Any]] | None,
        *,
        timestamp_ms: int | None = None,
        system_mode: int | None,
        current_mission_plan_id: int | None,
        aircraft_filter: Callable[[int], bool] | None = None,
        suppressed_aircraft: Iterable[int] | None = None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        with self._lock:
            logs: list[str] = []
            now_ts = int(timestamp_ms) if timestamp_ms is not None else int(self._now_ms())
            if not agent_states:
                self._clear_inactive_aircraft(set())
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
                trigger_type = self._classify_trigger_type(state, timestamp_ms=now_ts)
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
                reason, cause = self._resolve_rtb_reason(
                    aircraft_id=int(aircraft_id),
                    state=enriched_state,
                    trigger_type=trigger_type,
                    timestamp_ms=now_ts,
                )
                enriched_state["_rtb_trigger_type"] = trigger_type
                enriched_state["_rtb_reason"] = reason
                enriched_state["_rtb_cause"] = cause
                rtb_state_by_aircraft[int(aircraft_id)] = enriched_state

            self._clear_inactive_aircraft(set(rtb_state_by_aircraft))
            if not rtb_state_by_aircraft:
                return [], logs

            unique_aircraft = sorted(rtb_state_by_aircraft)
            for aircraft_id in unique_aircraft:
                self._state.availability_overrides[int(aircraft_id)] = False

            if system_mode not in (3, 4):
                for aircraft_id in unique_aircraft:
                    self._state.pending_by_aircraft.pop(int(aircraft_id), None)
                logs.append(
                    f"[0401] RTB/health-unavailable detected but replan skipped: mode={system_mode} (need 3/4)"
                )
                return [], logs

            mission_ids, package_id = self._collect_pending_input_ids(current_mission_plan_id)
            payloads: list[dict[str, Any]] = []
            for aircraft_id in unique_aircraft:
                if aircraft_id in self._state.triggered_aircraft:
                    continue
                active_state = rtb_state_by_aircraft.get(int(aircraft_id)) or {}
                trigger_type = str(active_state.get("_rtb_trigger_type") or "unexpectedRTB")
                cause = str(active_state.get("_rtb_cause") or "unknown")
                reason = str(active_state.get("_rtb_reason") or "")
                if self._is_immediate_trigger_type(trigger_type):
                    payload = self._build_replan_payload(
                        aircraft_id=int(aircraft_id),
                        state=active_state,
                        mission_ids=mission_ids,
                        package_id=package_id,
                        timestamp_ms=now_ts,
                        trigger_type=trigger_type,
                        reason=reason,
                        cause=cause,
                    )
                    if payload is None:
                        logs.append(
                            "[0401] health-unavailable replan skipped: missionPlanID allocation failed "
                            f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id})"
                        )
                        continue
                    self._state.triggered_aircraft.add(int(aircraft_id))
                    self._state.pending_by_aircraft.pop(int(aircraft_id), None)
                    payloads.append(payload)
                    logs.append(
                        "[0401] health-unavailable replan prepared "
                        f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id}, cause={cause})"
                    )
                    continue
                pending = self._state.pending_by_aircraft.get(int(aircraft_id))
                if (
                    pending is None
                    or pending.trigger_type != trigger_type
                    or pending.cause != cause
                ):
                    self._state.pending_by_aircraft[int(aircraft_id)] = PendingRtbTrigger(
                        first_seen_ms=now_ts,
                        trigger_type=trigger_type,
                        cause=cause,
                        reason=reason,
                    )
                    logs.append(
                        "[0401] RTB replan pending "
                        f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id}, "
                        f"trigger={trigger_type}, cause={cause}, hold={int(self._config().get('replan_hold_ms', 5000)) / 1000.0:.1f}s)"
                    )
                    continue
                if now_ts - int(pending.first_seen_ms) < int(self._config().get("replan_hold_ms", 5000)):
                    continue

                payload = self._build_replan_payload(
                    aircraft_id=int(aircraft_id),
                    state=active_state,
                    mission_ids=mission_ids,
                    package_id=package_id,
                    timestamp_ms=now_ts,
                    trigger_type=trigger_type,
                    reason=reason,
                    cause=cause,
                )
                if payload is None:
                    logs.append(
                        "[0401] RTB replan skipped: missionPlanID allocation failed "
                        f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id})"
                    )
                    continue
                self._state.triggered_aircraft.add(int(aircraft_id))
                self._state.pending_by_aircraft.pop(int(aircraft_id), None)
                payloads.append(payload)
                detail = payload.get("replanDetail") if isinstance(payload, dict) else None
                logged_cause = "unknown"
                logged_trigger = "unknown"
                if isinstance(detail, dict):
                    logged_cause = str(detail.get("rtbCause") or "unknown")
                    logged_trigger = str(detail.get("triggerType") or "unknown")
                logs.append(
                    "[0401] RTB replan prepared "
                    f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id}, "
                    f"trigger={logged_trigger}, cause={logged_cause})"
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
        timestamp_ms: int,
        trigger_type: str,
        reason: str,
        cause: str,
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

        actual_flight_mode = _coerce_int(state.get("flight_mode"))
        actual_health = _coerce_int(state.get("health"))
        return {
            "timestamp": int(timestamp_ms),
            "source": "MSM",
            "inputMissionPackageID": int(package_id) if package_id is not None else 0,
            "replanRequestTime": {"replanRequestTimestamp": int(timestamp_ms)},
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
                "datalinkConnected": state.get("datalink_connected"),
                "lastSignalTime": _coerce_int(state.get("last_signal_time")),
                "reason": reason,
            },
        }

    def _classify_trigger_type(
        self,
        state: dict[str, Any] | None,
        *,
        timestamp_ms: int | None,
    ) -> str | None:
        # 0401 payload/datalink faults only raise notices.
        # health==2 is an immediate unavailable/replan signal and has priority.
        _ = timestamp_ms
        config = self._config()
        health = _coerce_int((state or {}).get("health"))
        if health == int(config.get("abnormal_health_value", 2)):
            return self.HEALTH_UNAVAILABLE_TRIGGER
        flight_mode = _coerce_int((state or {}).get("flight_mode"))
        if flight_mode == int(config.get("unexpected_rtb_flight_mode", 5)):
            return "unexpectedRTB"
        return None

    def _resolve_rtb_reason(
        self,
        *,
        aircraft_id: int,
        state: dict[str, Any] | None,
        trigger_type: str | None,
        timestamp_ms: int | None,
    ) -> tuple[str, str]:
        config = self._config()
        health = _coerce_int((state or {}).get("health"))
        payload_health = _coerce_int((state or {}).get("payload_health"))
        fuel_warning = _coerce_int((state or {}).get("fuel_warning"))
        aircraft_label = _aircraft_display_label(aircraft_id)
        suffix = f"{aircraft_label} RTB"
        signal_issue = self._has_signal_loss(state, timestamp_ms=timestamp_ms)
        abnormal_health_value = int(config.get("abnormal_health_value", 2))
        health_issue = health == abnormal_health_value
        payload_issue = payload_health == abnormal_health_value
        fuel_issue = fuel_warning is not None and int(fuel_warning) >= int(config.get("fuel_warning_replan_level", 2))

        if str(trigger_type or "").strip() == self.HEALTH_UNAVAILABLE_TRIGGER and health_issue:
            return f"{aircraft_label} 기체 고장으로 인한 재계획", "health"
        if signal_issue:
            return f"통신 두절로 인한 {suffix}", "signal_loss"
        if health_issue:
            return f"무인기 고장으로 인한 {suffix}", "health"
        if payload_issue:
            return f"임무장비 고장으로 인한 {suffix}", "payload"
        if fuel_issue:
            return f"연료 부족으로 인한 {suffix}", "fuel"
        if str(trigger_type or "").strip() == "unexpectedRTB":
            return f"비정상 상태로 인한 {suffix}", "unexpected_rtb"
        return f"비정상 상태로 인한 {suffix}", "unknown"

    def _has_signal_loss(
        self,
        state: dict[str, Any] | None,
        *,
        timestamp_ms: int | None,
    ) -> bool:
        if (state or {}).get("datalink_connected") is False:
            return True
        current_ts = _coerce_int(timestamp_ms)
        last_signal_ts = _coerce_int((state or {}).get("last_signal_time"))
        if current_ts is None or last_signal_ts is None:
            return False
        grace_ms = int(self._config().get("signal_loss_grace_ms", 10000))
        return (current_ts - last_signal_ts) >= grace_ms

    @staticmethod
    def _is_immediate_trigger_type(trigger_type: str | None) -> bool:
        return str(trigger_type or "").strip() == RtbReplanCoordinator.HEALTH_UNAVAILABLE_TRIGGER

    def _clear_inactive_aircraft(self, active_aircraft: set[int]) -> None:
        tracked_ids = set(int(aid) for aid in self._state.availability_overrides.keys())
        tracked_ids.update(int(aid) for aid in self._state.pending_by_aircraft.keys())
        tracked_ids.update(int(aid) for aid in self._state.triggered_aircraft)
        for aircraft_id in tracked_ids:
            if int(aircraft_id) in active_aircraft:
                continue
            self._state.availability_overrides.pop(int(aircraft_id), None)
            self._state.pending_by_aircraft.pop(int(aircraft_id), None)
            self._state.triggered_aircraft.discard(int(aircraft_id))

    def _is_uav_state(self, aircraft_id: int, is_unmanned: object) -> bool:
        if int(aircraft_id) in self.UAV_IDS:
            return True
        if isinstance(is_unmanned, bool):
            return bool(is_unmanned)
        candidate = _coerce_int(is_unmanned)
        return candidate == 1
