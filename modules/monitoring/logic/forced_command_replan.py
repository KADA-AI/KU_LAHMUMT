# -*- coding: utf-8 -*-
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

from modules.common.option_codes import (
    DEFAULT_OPTION_CODE_SEQUENCE,
    ensure_option_code_sequence,
    option_code_to_label,
)
from modules.monitoring.logic.init_replan import allocate_mission_plan_ids, collect_input_mission_ids
from modules.monitoring.logic.mission_update import (
    build_uav_mission_view,
    load_db_json,
    parse_payload,
)


HOLD_DELAY_SECONDS = 30.0
HOLD_DELAY_REASON = "강제대기 후 30초 경과"

REPLAN_REASON_BY_TYPE: dict[int, str] = {
    1: "강제대기로 인한 재계획",
    2: "강제귀환으로 인한 재계획",
    3: "강제임무복귀로 인한 재계획",
}

# Keep option codes aligned with the rest of the monitoring replan flows,
# but render with the exact labels requested by the user.
OPTION_CODES: tuple[int, ...] = DEFAULT_OPTION_CODE_SEQUENCE
OPTION_LABEL_OVERRIDE: dict[int, str] = {
    6: "정찰/시간 균형",
    4: "정찰 특화",
    5: "최소 시간",
}


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _extract_forced_fields(body: dict[str, Any]) -> tuple[int | None, int | None, int | None, str | None]:
    ts = None
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in body:
            ts = _coerce_int(body.get(key))
            break

    aircraft_id = None
    for key in ("aircraftID", "AircraftID", "aircraftId", "aircraft_id"):
        if key in body:
            aircraft_id = _coerce_int(body.get(key))
            break

    mandatory_type = None
    for key in ("mandatoryType", "MandatoryType", "mandatory_type"):
        if key in body:
            mandatory_type = _coerce_int(body.get(key))
            break

    source = None
    for key in ("source", "Source", "sourceModuleName", "SourceModuleName"):
        if key in body:
            try:
                source = str(body.get(key))
            except Exception:
                source = None
            break

    return ts, aircraft_id, mandatory_type, source


def _plan_aircraft_ids(mission_plan_id: int | None) -> list[int]:
    if mission_plan_id is None:
        return []
    plan = load_db_json("MissionPlan", mission_plan_id)
    aircraft_list = plan.get("aircraftList") or []
    ids: list[int] = []
    for entry in aircraft_list:
        if not isinstance(entry, dict):
            continue
        aid = _coerce_int(entry.get("aircraftID") or entry.get("AircraftID"))
        if aid is not None:
            ids.append(int(aid))
    return sorted({int(v) for v in ids})


def _extract_input_package_id(mission_plan_id: int | None) -> int | None:
    if mission_plan_id is None:
        return None
    plan = load_db_json("MissionPlan", mission_plan_id)
    package_id = _coerce_int(
        plan.get("inputMissionPackageID")
        or plan.get("InputMissionPackageID")
        or plan.get("inputMissionPackageId")
    )
    return package_id


@dataclass
class HoldState:
    aircraft_id: int
    deadline_monotonic: float
    command_timestamp: int | None
    due_logged: bool = False


@dataclass
class ForcedCommandState:
    option_id_counter: int = 0
    availability_overrides: dict[int, bool] = field(default_factory=dict)
    forced_unavailable_aircraft: set[int] = field(default_factory=set)
    hold_state_by_aircraft: dict[int, HoldState] = field(default_factory=dict)
    permanent_return_aircraft: set[int] = field(default_factory=set)
    last_signature: tuple[int, int, int | None] | None = None
    last_signature_monotonic: float = 0.0


class ForcedCommandReplanCoordinator:
    """Handle 0802 MandatoryCommand inputs and emit ver2-style 0902 requests."""

    REPLAN_LEVEL = 1
    SIGNATURE_DEDUP_SECONDS = 0.6

    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        now_monotonic: Callable[[], float] = time.monotonic,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._now_monotonic = now_monotonic
        self._log = logger
        self._state = ForcedCommandState()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def get_availability_overrides(self) -> dict[int, bool]:
        return dict(self._state.availability_overrides)

    def get_rtb_suppressed_aircraft(self) -> set[int]:
        return set(self._state.permanent_return_aircraft)

    def is_permanent_return(self, aircraft_id: int | None) -> bool:
        if aircraft_id is None:
            return False
        try:
            aid = int(aircraft_id)
        except Exception:
            return False
        return aid in self._state.permanent_return_aircraft

    def on_forced_command(
        self,
        payload: object | None,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        logs: list[str] = []
        body = parse_payload(payload)
        if not body:
            return [], logs

        ts, aircraft_id, mandatory_type, source = _extract_forced_fields(body)
        if aircraft_id is None or mandatory_type is None:
            logs.append("[0802] skipped: aircraftID/mandatoryType missing")
            return [], logs

        now_mono = float(self._now_monotonic())
        signature = (int(aircraft_id), int(mandatory_type), ts)
        if self._is_duplicate_signature(signature, now_mono):
            return [], logs
        self._state.last_signature = signature
        self._state.last_signature_monotonic = now_mono

        reason_text = REPLAN_REASON_BY_TYPE.get(int(mandatory_type))
        if reason_text:
            logs.append(
                f"[0802] mandatoryType={mandatory_type} received"
                f" (aircraftID={aircraft_id}, source={source or '-'})"
            )

        if int(mandatory_type) == 2:
            self._state.permanent_return_aircraft.add(int(aircraft_id))

        self._apply_availability_override(int(aircraft_id), int(mandatory_type))

        # mandatoryType=1 -> hold delay only (no immediate replan)
        if int(mandatory_type) == 1:
            deadline = now_mono + float(HOLD_DELAY_SECONDS)
            self._state.hold_state_by_aircraft[int(aircraft_id)] = HoldState(
                aircraft_id=int(aircraft_id),
                deadline_monotonic=deadline,
                command_timestamp=ts,
            )
            logs.append(
                "[0802] 강제대기 수신 -> 30초 유예 후 재계획 검토"
                f" (aircraftID={aircraft_id})"
            )
            return [], logs

        # mandatoryType=3 within hold window -> cancel hold and skip replan
        if int(mandatory_type) == 3:
            hold = self._state.hold_state_by_aircraft.get(int(aircraft_id))
            if hold is not None and now_mono < float(hold.deadline_monotonic):
                self._state.hold_state_by_aircraft.pop(int(aircraft_id), None)
                logs.append(
                    "[0802] 강제임무복귀가 강제대기 유예 내 수신 -> 재계획 생략"
                    f" (aircraftID={aircraft_id})"
                )
                return [], logs
            # Outside the hold window, any lingering hold state should be cleared.
            self._state.hold_state_by_aircraft.pop(int(aircraft_id), None)

        if int(mandatory_type) == 2:
            # 강제귀환 should also cancel any pending hold state.
            self._state.hold_state_by_aircraft.pop(int(aircraft_id), None)

        # mandatoryType=2 -> immediate replan only in active modes
        if int(mandatory_type) == 2:
            if system_mode not in (3, 4):
                logs.append(f"[0802] replan skipped: mode={system_mode} (need 3/4)")
                return [], logs
            payload_0902 = self._build_replan_payload(
                mandatory_type=int(mandatory_type),
                aircraft_id=int(aircraft_id),
                command_timestamp=ts,
                current_mission_plan_id=current_mission_plan_id,
                reason_override=reason_text,
            )
            return ([payload_0902] if payload_0902 else []), logs

        # mandatoryType=3 -> resume only (no immediate replan)
        return [], logs

    def poll_due_holds(
        self,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
        availability_check: Callable[[int], bool | None] | None = None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        """Emit delayed replan requests when 강제대기 유예가 만료된 경우."""
        logs: list[str] = []
        if not self._state.hold_state_by_aircraft:
            return [], logs

        now_mono = float(self._now_monotonic())
        dispatch: list[dict[str, Any]] = []

        for aircraft_id, hold in list(self._state.hold_state_by_aircraft.items()):
            if now_mono < float(hold.deadline_monotonic):
                continue
            if not hold.due_logged:
                logs.append(
                    "[0802] 강제대기 유예 만료 -> 재계획 검토"
                    f" (aircraftID={aircraft_id})"
                )
                hold.due_logged = True

            if system_mode not in (3, 4):
                continue

            if availability_check is not None:
                try:
                    available = availability_check(int(aircraft_id))
                except Exception:
                    available = None
                if available is True:
                    logs.append(
                        "[0802] hold expired but aircraft available -> skip delayed replan"
                        f" (aircraftID={aircraft_id})"
                    )
                    self._state.hold_state_by_aircraft.pop(int(aircraft_id), None)
                    continue

            payload_0902 = self._build_replan_payload(
                mandatory_type=1,
                aircraft_id=int(aircraft_id),
                command_timestamp=hold.command_timestamp,
                current_mission_plan_id=current_mission_plan_id,
                reason_override=HOLD_DELAY_REASON,
            )
            if payload_0902:
                dispatch.append(payload_0902)

            # Hold-specific state can be cleared after the delayed replan fires.
            self._state.hold_state_by_aircraft.pop(int(aircraft_id), None)

        return dispatch, logs

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _is_duplicate_signature(
        self,
        signature: tuple[int, int, int | None],
        now_mono: float,
    ) -> bool:
        last = self._state.last_signature
        if last is None or last != signature:
            return False
        return (now_mono - float(self._state.last_signature_monotonic)) < float(
            self.SIGNATURE_DEDUP_SECONDS
        )

    def _apply_availability_override(self, aircraft_id: int, mandatory_type: int) -> None:
        if mandatory_type in (1, 2):
            self._state.availability_overrides[int(aircraft_id)] = False
            self._state.forced_unavailable_aircraft.add(int(aircraft_id))
            return
        if mandatory_type == 3:
            self._state.availability_overrides[int(aircraft_id)] = True
            self._state.forced_unavailable_aircraft.discard(int(aircraft_id))

    def _allocate_option_ids(self, count: int) -> list[int]:
        allocated: list[int] = []
        for _ in range(max(int(count), 0)):
            self._state.option_id_counter += 1
            allocated.append(int(self._state.option_id_counter))
        return allocated

    def _allocate_plan_ids(self, count: int, *, seed: int) -> list[int]:
        plan_ids = allocate_mission_plan_ids(count)
        if len(plan_ids) >= count:
            return [int(v) for v in plan_ids[:count]]
        next_id = int(seed)
        while len(plan_ids) < count:
            plan_ids.append(int(next_id))
            next_id += 1
        return [int(v) for v in plan_ids[:count]]

    def _option_label(self, code: int) -> str:
        if int(code) in OPTION_LABEL_OVERRIDE:
            return str(OPTION_LABEL_OVERRIDE[int(code)])
        return str(option_code_to_label(code))

    def _collect_input_ids(
        self,
        *,
        current_mission_plan_id: int | None,
        excluded_aircraft: set[int],
    ) -> tuple[list[int], set[int]]:
        plan_aircraft_ids = _plan_aircraft_ids(current_mission_plan_id)
        view = build_uav_mission_view(
            current_mission_plan_id,
            uav_ids=plan_aircraft_ids or (4, 5, 6),
        )

        excluded_input_ids: set[int] = set()
        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aid = _coerce_int(entry.get("aircraft_id"))
            if aid is None or int(aid) not in excluded_aircraft:
                continue
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                input_id = _coerce_int(mission.get("input_id"))
                if input_id is not None and input_id > 0:
                    excluded_input_ids.add(int(input_id))

        pending_ids: list[int] = []
        all_ids: list[int] = []
        for mission in view.get("input_missions") or []:
            if not isinstance(mission, dict):
                continue
            input_id = _coerce_int(mission.get("input_mission_id"))
            if input_id is None or input_id <= 0:
                continue
            all_ids.append(int(input_id))
            if not bool(mission.get("is_done")):
                pending_ids.append(int(input_id))

        candidates = pending_ids or all_ids
        filtered: list[int] = []
        unique_candidates: list[int] = []
        seen: set[int] = set()
        for mid in candidates:
            if mid in seen:
                continue
            seen.add(mid)
            mid_int = int(mid)
            unique_candidates.append(mid_int)
            if mid_int in excluded_input_ids:
                continue
            filtered.append(mid_int)

        # IMPORTANT: Some plans assign every input mission to every aircraft.
        # In that case, excluding a single forced aircraft would eliminate all
        # missions and cause mission_ids=[0], which breaks the pipeline.
        if not filtered and unique_candidates:
            filtered = list(unique_candidates)
            if excluded_input_ids and self._log:
                try:
                    self._log(
                        "[0802] exclusion removed all missions -> falling back to pending mission IDs"
                    )
                except Exception:
                    pass

        if not filtered:
            fallback_ids = collect_input_mission_ids()
            unique_fallback: list[int] = []
            seen_fb: set[int] = set()
            for mid in fallback_ids:
                mid_int = int(mid)
                if mid_int in seen_fb:
                    continue
                seen_fb.add(mid_int)
                unique_fallback.append(mid_int)
                if mid_int in excluded_input_ids:
                    continue
                filtered.append(mid_int)
            if not filtered and unique_fallback:
                filtered = list(unique_fallback)

        if not filtered:
            filtered = [0]

        return filtered, excluded_input_ids

    def _build_replan_payload(
        self,
        *,
        mandatory_type: int,
        aircraft_id: int,
        command_timestamp: int | None,
        current_mission_plan_id: int | None,
        reason_override: str | None,
    ) -> dict[str, Any] | None:
        ts = int(self._now_ms())
        option_count = len(OPTION_CODES)
        plan_seed = 700_000_000 + (ts % 1_000)
        plan_ids = self._allocate_plan_ids(option_count, seed=plan_seed)
        option_ids = self._allocate_option_ids(option_count)
        option_codes = ensure_option_code_sequence(OPTION_CODES, option_count)

        pending_options: list[dict[str, Any]] = []
        for idx, code in enumerate(option_codes):
            pending_options.append(
                {
                    "optionID": int(option_ids[idx]),
                    "optionName": self._option_label(int(code)),
                    "missionPlanID": int(plan_ids[idx]),
                }
            )

        mission_ids, excluded_input_ids = self._collect_input_ids(
            current_mission_plan_id=current_mission_plan_id,
            excluded_aircraft=set(self._state.forced_unavailable_aircraft),
        )
        input_models = [{"inputMissionID": int(mid)} for mid in mission_ids]

        package_id = _extract_input_package_id(current_mission_plan_id)

        reason_text = reason_override or REPLAN_REASON_BY_TYPE.get(int(mandatory_type))
        if not reason_text:
            reason_text = f"강제명령({int(mandatory_type)})에 의한 재계획"

        payload: dict[str, Any] = {
            "timestamp": ts,
            "source": "MSM",
            "inputMissionPackageID": int(package_id) if package_id is not None else 0,
            "replanRequestTime": {"replanRequestTimestamp": ts},
            "replanLevel": int(self.REPLAN_LEVEL),
            "replanRequest": str(reason_text),
            "inputMissionIDList": input_models,
            "pendingOptionList": pending_options,
        }
        return payload
