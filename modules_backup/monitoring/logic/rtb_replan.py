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


def _coerce_bool(value: object) -> bool | None:
    if isinstance(value, bool):
        return bool(value)
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return bool(int(value))
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
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
    last_notice_bucket: int = 0


@dataclass
class RtbReplanState:
    availability_overrides: dict[int, bool] = field(default_factory=dict)
    triggered_aircraft: set[int] = field(default_factory=set)
    pending_by_aircraft: dict[int, PendingRtbTrigger] = field(default_factory=dict)


class RtbReplanCoordinator:
    REPLAN_LEVEL = 1
    RTB_TRIGGER = "unexpectedRTB"
    HEALTH_UNAVAILABLE_TRIGGER = "abnormalHealthUnavailable"
    PAYLOAD_UNAVAILABLE_TRIGGER = "payloadHealthUnavailable"
    COMMUNICATION_UNAVAILABLE_TRIGGER = "communicationLossUnavailable"
    NOTICE_INTERVAL_MS = 10000
    FAULT_UNAVAILABLE_PRIORITY: tuple[str, ...] = (
        COMMUNICATION_UNAVAILABLE_TRIGGER,
        HEALTH_UNAVAILABLE_TRIGGER,
        PAYLOAD_UNAVAILABLE_TRIGGER,
    )
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
    ) -> tuple[list[dict[str, Any]], list[str], list[str]]:
        with self._lock:
            logs: list[str] = []
            notices: list[str] = []
            now_ts = int(timestamp_ms) if timestamp_ms is not None else int(self._now_ms())
            if not agent_states:
                self._clear_inactive_aircraft(set())
                return [], logs, notices

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
                return [], logs, notices

            unique_aircraft = self._sort_aircraft_by_trigger_priority(rtb_state_by_aircraft)
            for aircraft_id in unique_aircraft:
                active_state = rtb_state_by_aircraft.get(int(aircraft_id)) or {}
                trigger_type = str(active_state.get("_rtb_trigger_type") or self.RTB_TRIGGER)
                if self._sets_availability_before_dispatch(trigger_type):
                    self._state.availability_overrides[int(aircraft_id)] = False

            if system_mode not in (3, 4):
                for aircraft_id in unique_aircraft:
                    self._state.pending_by_aircraft.pop(int(aircraft_id), None)
                logs.append(
                    f"[0401] RTB/health-unavailable detected but replan skipped: mode={system_mode} (need 3/4)"
                )
                return [], logs, notices

            mission_ids, package_id = self._collect_pending_input_ids(current_mission_plan_id)
            payloads: list[dict[str, Any]] = []
            for aircraft_id in unique_aircraft:
                active_state = rtb_state_by_aircraft.get(int(aircraft_id)) or {}
                trigger_type = str(active_state.get("_rtb_trigger_type") or self.RTB_TRIGGER)
                cause = str(active_state.get("_rtb_cause") or "unknown")
                reason = str(active_state.get("_rtb_reason") or "")
                if aircraft_id in self._state.triggered_aircraft:
                    if self._sets_availability_on_dispatch(trigger_type):
                        self._state.availability_overrides[int(aircraft_id)] = False
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
                        f"[0401] {self._replan_log_label(trigger_type)} replan pending "
                        f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id}, "
                        f"trigger={trigger_type}, cause={cause}, "
                        f"hold={self._hold_ms_for_trigger_type(trigger_type) / 1000.0:.1f}s)"
                    )
                    pending = self._state.pending_by_aircraft.get(int(aircraft_id))
                if pending is None:
                    continue
                hold_ms = self._hold_ms_for_trigger_type(trigger_type)
                elapsed_ms = max(0, now_ts - int(pending.first_seen_ms))
                progress_notice = self._build_pending_progress_notice(
                    aircraft_id=int(aircraft_id),
                    pending=pending,
                    elapsed_ms=elapsed_ms,
                    hold_ms=hold_ms,
                )
                if progress_notice:
                    notices.append(progress_notice)
                if elapsed_ms < hold_ms:
                    continue

                if self._sets_availability_on_dispatch(trigger_type):
                    self._state.availability_overrides[int(aircraft_id)] = False

                payload = self._build_replan_payload(
                    aircraft_id=int(aircraft_id),
                    state=active_state,
                    mission_ids=mission_ids,
                    package_id=package_id,
                    current_mission_plan_id=current_mission_plan_id,
                    timestamp_ms=now_ts,
                    trigger_type=trigger_type,
                    reason=reason,
                    cause=cause,
                )
                if payload is None:
                    logs.append(
                        f"[0401] {self._replan_log_label(trigger_type)} replan skipped: "
                        "missionPlanID allocation failed "
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
                    f"[0401] {self._replan_log_label(trigger_type)} replan prepared "
                    f"({_aircraft_display_label(aircraft_id)}, aircraftID={aircraft_id}, "
                    f"trigger={logged_trigger}, cause={logged_cause})"
                )

            return payloads, logs, notices

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
        current_mission_plan_id: int | None,
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
                "sourceMissionPlanID": int(current_mission_plan_id) if current_mission_plan_id is not None else None,
                "currentMissionPlanID": int(current_mission_plan_id) if current_mission_plan_id is not None else None,
                "aircraftID": int(aircraft_id),
                "flightMode": actual_flight_mode,
                "health": actual_health,
                "rtbCause": cause,
                "fuelWarning": _coerce_int(state.get("fuel_warning")),
                "payloadHealth": _coerce_int(state.get("payload_health")),
                "datalinkConnected": state.get("datalink_connected"),
                "commandAircraftID": self._command_aircraft_id(state),
                "commandDatalinkConnected": self._command_datalink_connected(state),
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
        config = self._config()
        # Keep the legacy RTB path distinct: RTB still applies availability before
        # dispatch, while non-RTB faults wait for fault_unavailable_hold_ms.
        flight_mode = _coerce_int((state or {}).get("flight_mode"))
        if flight_mode == int(config.get("unexpected_rtb_flight_mode", 5)):
            return self.RTB_TRIGGER

        abnormal_health_value = int(config.get("abnormal_health_value", 2))
        health = _coerce_int((state or {}).get("health"))
        payload_health = _coerce_int((state or {}).get("payload_health"))
        detected_faults: dict[str, bool] = {
            self.COMMUNICATION_UNAVAILABLE_TRIGGER: self._has_signal_loss(state, timestamp_ms=timestamp_ms),
            self.HEALTH_UNAVAILABLE_TRIGGER: health == abnormal_health_value,
            self.PAYLOAD_UNAVAILABLE_TRIGGER: payload_health == abnormal_health_value,
        }
        for trigger_type in self.FAULT_UNAVAILABLE_PRIORITY:
            if detected_faults.get(trigger_type):
                return trigger_type

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

        trigger = str(trigger_type or "").strip()
        if trigger == self.HEALTH_UNAVAILABLE_TRIGGER and health_issue:
            return f"{aircraft_label} 고장 재계획", "health"
        if trigger == self.COMMUNICATION_UNAVAILABLE_TRIGGER and signal_issue:
            return f"{aircraft_label} 통신두절", "signal_loss"
        if trigger == self.PAYLOAD_UNAVAILABLE_TRIGGER and payload_issue:
            return f"{aircraft_label} 장비고장", "payload"
        if signal_issue:
            return "통신두절 RTB", "signal_loss"
        if health_issue:
            return "무인기고장 RTB", "health"
        if payload_issue:
            return "장비고장 RTB", "payload"
        if fuel_issue:
            return "연료부족 RTB", "fuel"
        if trigger == self.RTB_TRIGGER:
            return "비정상 RTB", "unexpected_rtb"
        return "비정상 RTB", "unknown"

    def _has_signal_loss(
        self,
        state: dict[str, Any] | None,
        *,
        timestamp_ms: int | None,
    ) -> bool:
        return self._command_datalink_connected(state) is False

    def _command_aircraft_id(self, state: dict[str, Any] | None) -> int | None:
        for key in ("leader_aircraft_id", "command_aircraft_id"):
            candidate = _coerce_int((state or {}).get(key))
            if candidate is not None and 1 <= int(candidate) <= 3:
                return int(candidate)
        configured = _coerce_int(self._config().get("command_aircraft_id", 1))
        if configured is not None and 1 <= int(configured) <= 3:
            return int(configured)
        return None

    def _command_datalink_connected(self, state: dict[str, Any] | None) -> bool | None:
        command_id = self._command_aircraft_id(state)
        if command_id is None:
            return None
        pair_statuses = (state or {}).get("datalink_connected_by_manned")
        if not isinstance(pair_statuses, dict):
            return None
        for key in (command_id, str(command_id)):
            if key in pair_statuses:
                return _coerce_bool(pair_statuses.get(key))
        return None

    def _hold_ms_for_trigger_type(self, trigger_type: str | None) -> int:
        key = str(trigger_type or "").strip()
        config = self._config()
        if key == self.RTB_TRIGGER:
            return max(0, int(config.get("replan_hold_ms", 5000)))
        return max(0, int(config.get("fault_unavailable_hold_ms", 55000)))

    def _build_pending_progress_notice(
        self,
        *,
        aircraft_id: int,
        pending: PendingRtbTrigger,
        elapsed_ms: int,
        hold_ms: int,
    ) -> str | None:
        if str(pending.trigger_type or "").strip() == self.RTB_TRIGGER:
            return None
        if hold_ms <= 0 or elapsed_ms >= hold_ms:
            return None
        bucket = max(0, int(elapsed_ms) // int(self.NOTICE_INTERVAL_MS))
        if bucket <= 0 or bucket <= int(pending.last_notice_bucket):
            return None
        pending.last_notice_bucket = int(bucket)
        elapsed_s = max(0, int(elapsed_ms) // 1000)
        remaining_s = max(0, (int(hold_ms) - int(elapsed_ms) + 999) // 1000)
        situation = self._progress_situation_label(pending.trigger_type)
        return f"{int(aircraft_id):04d} {situation} 재계획 {remaining_s:02d}초"

    def _progress_situation_label(self, trigger_type: str | None) -> str:
        key = str(trigger_type or "").strip()
        if key == self.COMMUNICATION_UNAVAILABLE_TRIGGER:
            return "통신두절"
        if key == self.HEALTH_UNAVAILABLE_TRIGGER:
            return "무인기 고장"
        if key == self.PAYLOAD_UNAVAILABLE_TRIGGER:
            return "임무장비 고장"
        return "비가용"

    def _sets_availability_before_dispatch(self, trigger_type: str | None) -> bool:
        return str(trigger_type or "").strip() == self.RTB_TRIGGER

    def _sets_availability_on_dispatch(self, trigger_type: str | None) -> bool:
        return str(trigger_type or "").strip() in {
            self.RTB_TRIGGER,
            self.HEALTH_UNAVAILABLE_TRIGGER,
            self.PAYLOAD_UNAVAILABLE_TRIGGER,
            self.COMMUNICATION_UNAVAILABLE_TRIGGER,
        }

    def _trigger_priority_rank(self, trigger_type: str | None) -> int:
        key = str(trigger_type or "").strip()
        if key == self.RTB_TRIGGER:
            return 0
        try:
            return 1 + self.FAULT_UNAVAILABLE_PRIORITY.index(key)
        except ValueError:
            return len(self.FAULT_UNAVAILABLE_PRIORITY) + 10

    def _sort_aircraft_by_trigger_priority(
        self, rtb_state_by_aircraft: dict[int, dict[str, Any]]
    ) -> list[int]:
        return sorted(
            (int(aircraft_id) for aircraft_id in rtb_state_by_aircraft),
            key=lambda aircraft_id: (
                self._trigger_priority_rank(
                    (rtb_state_by_aircraft.get(int(aircraft_id)) or {}).get("_rtb_trigger_type")
                ),
                int(aircraft_id),
            ),
        )

    def _replan_log_label(self, trigger_type: str | None) -> str:
        if str(trigger_type or "").strip() == self.RTB_TRIGGER:
            return "RTB"
        return "unavailable"

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
