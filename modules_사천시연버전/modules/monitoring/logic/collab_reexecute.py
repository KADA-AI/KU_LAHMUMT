# -*- coding: utf-8 -*-
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

from modules.common.option_codes import (
    DEFAULT_OPTION_CODE_SEQUENCE,
    ensure_option_code_sequence,
    option_code_to_label,
)
from modules.monitoring.logic.init_replan import allocate_mission_plan_ids
from modules.monitoring.logic.mission_update import parse_payload


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_bool(value: object) -> bool:
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "y"}:
            return True
        if text in {"0", "false", "no", "n", ""}:
            return False
    try:
        return bool(int(value))  # type: ignore[arg-type]
    except Exception:
        return bool(value)


def _extract_input_key(plan: dict[str, Any]) -> tuple[int | None, int | None]:
    package_id = _coerce_int(
        plan.get("inputMissionPackageID") or plan.get("InputMissionPackageID")
    )
    timestamp = _coerce_int(
        plan.get("timestamp")
        or plan.get("Timestamp")
        or plan.get("timeStamp")
        or plan.get("TimeStamp")
    )
    return package_id, timestamp


def _extract_input_ids(plan: dict[str, Any]) -> list[int]:
    mission_list = plan.get("inputMissionList") or plan.get("InputMissionList") or []
    ids_all: list[int] = []
    ids_pending: list[int] = []
    for item in mission_list:
        if not isinstance(item, dict):
            continue
        input_id = _coerce_int(item.get("inputMissionID") or item.get("InputMissionID"))
        if input_id is None:
            continue
        ids_all.append(input_id)
        is_done = _coerce_bool(item.get("isDone") or item.get("IsDone"))
        if not is_done:
            ids_pending.append(input_id)
    if ids_pending:
        return list(dict.fromkeys(ids_pending))
    if ids_all:
        return list(dict.fromkeys(ids_all))
    return []


def _mission_list(plan: dict[str, Any]) -> list[dict[str, Any]]:
    mission_list = plan.get("inputMissionList") or plan.get("InputMissionList") or []
    if not isinstance(mission_list, list):
        return []
    return [item for item in mission_list if isinstance(item, dict)]


def _mission_id(mission: dict[str, Any]) -> int | None:
    return _coerce_int(mission.get("inputMissionID") or mission.get("InputMissionID"))


def _mission_done(mission: dict[str, Any]) -> bool:
    if "isDone" in mission:
        return _coerce_bool(mission.get("isDone"))
    if "IsDone" in mission:
        return _coerce_bool(mission.get("IsDone"))
    return False


def _mission_clone_fingerprint(mission: dict[str, Any]) -> str:
    body = {
        str(key): value
        for key, value in mission.items()
        if str(key).lower() not in {"inputmissionid", "isdone"}
    }
    try:
        return json.dumps(
            body,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except Exception:
        return str(body)


def _detect_reexecute_clone_pair(plan: dict[str, Any]) -> tuple[int, int] | None:
    missions = _mission_list(plan)
    mission_ids = [
        int(value)
        for value in (_mission_id(item) for item in missions)
        if value is not None
    ]
    max_mission_id = max(mission_ids) if mission_ids else None
    for idx in range(len(missions) - 1):
        original = missions[idx]
        cloned = missions[idx + 1]
        original_id = _mission_id(original)
        cloned_id = _mission_id(cloned)
        if original_id is None or cloned_id is None or int(original_id) == int(cloned_id):
            continue
        if max_mission_id is not None and int(cloned_id) != int(max_mission_id):
            continue
        if not _mission_done(original) or _mission_done(cloned):
            continue
        if _mission_clone_fingerprint(original) == _mission_clone_fingerprint(cloned):
            return int(original_id), int(cloned_id)
    return None


def _detect_reexecute_clone_input_id(plan: dict[str, Any]) -> int | None:
    pair = _detect_reexecute_clone_pair(plan)
    if pair is None:
        return None
    return int(pair[1])


def _prioritize_clone_input_ids(
    clone_input_id: int | None,
    input_ids: Iterable[int],
) -> list[int]:
    ordered: list[int] = []
    if clone_input_id is not None:
        ordered.append(int(clone_input_id))
    for value in input_ids:
        try:
            input_id = int(value)
        except Exception:
            continue
        if input_id not in ordered:
            ordered.append(input_id)
    return ordered


def _fingerprint_plan(plan: dict[str, Any]) -> str:
    """Return a stable fingerprint for the full 0201 payload."""
    try:
        text = json.dumps(
            plan,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except Exception:
        text = str(plan)
    data = text.encode("utf-8", "ignore")
    return hashlib.sha1(data).hexdigest()


@dataclass
class ReexecuteState:
    pending: bool = False
    inflight: bool = False
    required_new_key: tuple[int | None, int | None] | None = None
    last_input_key: tuple[int | None, int | None] | None = None
    last_dispatched_key: tuple[int | None, int | None] | None = None
    required_new_signature: str | None = None
    last_input_signature: str | None = None
    last_dispatched_signature: str | None = None
    wait_logged: bool = False
    last_input_ids: list[int] = field(default_factory=list)
    last_reexecute_clone_input_id: int | None = None
    last_reexecute_source_input_id: int | None = None
    last_reexecute_clone_key: tuple[int | None, int | None] | None = None
    last_reexecute_clone_signature: str | None = None
    late_execute_ack_key: tuple[int | None, int | None] | None = None
    option_id_counter: int = 0


class CollabReexecuteCoordinator:
    """Execute=2 -> wait for refreshed 0201 -> dispatch 0902 replan request."""

    # Keep option codes aligned with the rest of the system (ICD / DSS tabs).
    OPTION_CODES: tuple[int, ...] = DEFAULT_OPTION_CODE_SEQUENCE
    REPLAN_REASON = "협업기저임무 재수행 요청"

    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._state = ReexecuteState()

    # ------------------------------------------------------------------
    # Public entry points
    # ------------------------------------------------------------------
    def is_active(self) -> bool:
        """Return True only while waiting for a refreshed 0201."""
        # NOTE: We intentionally do not treat "inflight" as blocking.
        # Once a reexecute-triggered 0902 has been dispatched, we should not
        # suppress the "0201 refresh during execution" logic.
        return bool(self._state.pending)

    def has_dispatched_input_plan(self, payload: object | None) -> bool:
        """Return True when payload is the 0201 already consumed by reexecute."""
        plan = parse_payload(payload)
        if not plan:
            return False
        if self._state.last_dispatched_key is None:
            return False
        if self._state.last_dispatched_signature is None:
            return False
        current_key = _extract_input_key(plan)
        current_signature = _fingerprint_plan(plan)
        return (
            current_key == self._state.last_dispatched_key
            and current_signature == self._state.last_dispatched_signature
        )

    def on_execute(self, execute: int | None) -> list[str]:
        logs: list[str] = []
        if execute is None:
            return logs
        if int(execute) == 2:
            if (
                self._state.last_reexecute_clone_input_id is not None
                and self._state.last_reexecute_clone_key is not None
                and self._state.last_reexecute_clone_key == self._state.last_input_key
                and self._state.last_reexecute_clone_key == self._state.last_dispatched_key
                and self._state.late_execute_ack_key != self._state.last_reexecute_clone_key
            ):
                self._state.pending = False
                self._state.wait_logged = False
                self._state.late_execute_ack_key = self._state.last_reexecute_clone_key
                package_id, timestamp = self._state.last_reexecute_clone_key
                logs.append(
                    "[REEXEC] execute=2 received after refreshed 0201 clone -> "
                    "already dispatched reexecute 0902"
                    f" (package={package_id}, timestamp={timestamp}, "
                    f"inputMissionID={self._state.last_reexecute_clone_input_id})"
                )
                return logs
            self._state.pending = True
            self._state.inflight = False
            self._state.wait_logged = False
            self._state.required_new_key = self._state.last_input_key
            self._state.required_new_signature = self._state.last_input_signature
            self._state.last_dispatched_signature = None
            package_id, timestamp = self._state.required_new_key or (None, None)
            logs.append(
                "[REEXEC] execute=2 received -> waiting for next 0201 arrival"
                f" (package={package_id}, timestamp={timestamp})"
            )
            return logs

        if self._state.pending or self._state.inflight:
            logs.append(
                f"[REEXEC] execute={int(execute)} received -> cancel reexecute-wait mode"
            )
        self._state.pending = False
        self._state.inflight = False
        self._state.wait_logged = False
        self._state.required_new_key = self._state.last_input_key
        self._state.required_new_signature = self._state.last_input_signature
        return logs

    def on_input_plan(
        self,
        payload: object | None,
        *,
        has_new_arrival: bool = True,
    ) -> tuple[dict[str, Any] | None, list[str]]:
        logs: list[str] = []
        plan = parse_payload(payload)
        if not plan:
            return None, logs

        package_id, timestamp = _extract_input_key(plan)
        current_key = (package_id, timestamp)
        self._state.last_input_key = current_key
        current_signature = _fingerprint_plan(plan)
        self._state.last_input_signature = current_signature

        input_ids = _extract_input_ids(plan)
        if input_ids:
            self._state.last_input_ids = input_ids

        clone_pair = _detect_reexecute_clone_pair(plan)
        clone_source_input_id = int(clone_pair[0]) if clone_pair is not None else None
        clone_input_id = int(clone_pair[1]) if clone_pair is not None else None
        if clone_input_id is not None:
            self._state.last_reexecute_clone_input_id = int(clone_input_id)
            self._state.last_reexecute_source_input_id = clone_source_input_id
            self._state.last_reexecute_clone_key = current_key
            self._state.last_reexecute_clone_signature = current_signature

        if not self._state.pending or self._state.inflight:
            if (
                clone_input_id is not None
                and bool(has_new_arrival)
                and not (
                    self._state.last_dispatched_signature is not None
                    and current_signature == self._state.last_dispatched_signature
                    and self._state.last_dispatched_key is not None
                    and current_key == self._state.last_dispatched_key
                )
            ):
                reexecute_input_ids = _prioritize_clone_input_ids(clone_input_id, input_ids)
                replan_payload = self._build_replan_payload(
                    plan,
                    package_id,
                    reexecute_input_ids,
                    source_input_id=clone_source_input_id,
                )
                if replan_payload is None:
                    return None, logs
                self._state.last_dispatched_key = current_key
                self._state.last_dispatched_signature = current_signature
                self._state.pending = False
                self._state.inflight = True
                self._state.required_new_key = current_key
                self._state.required_new_signature = current_signature
                self._state.wait_logged = False
                self._state.late_execute_ack_key = None
                logs.append(
                    "[REEXEC] refreshed 0201 clone detected before execute=2 listener -> "
                    "dispatching 0902 replan request"
                    f" (package={package_id}, timestamp={timestamp}, inputMissionID={int(clone_input_id)})"
                )
                return replan_payload, logs
            return None, logs

        if not has_new_arrival:
            if not self._state.wait_logged:
                logs.append(
                    "[REEXEC] execute=2 mode active -> waiting for next 0201 arrival"
                )
                self._state.wait_logged = True
            return None, logs

        if (
            self._state.last_dispatched_signature is not None
            and current_signature == self._state.last_dispatched_signature
            and self._state.last_dispatched_key is not None
            and current_key == self._state.last_dispatched_key
        ):
            return None, logs

        reexecute_input_ids = _prioritize_clone_input_ids(clone_input_id, input_ids)
        replan_payload = self._build_replan_payload(
            plan,
            package_id,
            reexecute_input_ids,
            source_input_id=clone_source_input_id,
        )
        if replan_payload is None:
            return None, logs

        self._state.last_dispatched_key = current_key
        self._state.last_dispatched_signature = current_signature
        self._state.pending = False
        self._state.inflight = True
        self._state.required_new_key = current_key
        self._state.required_new_signature = current_signature
        self._state.wait_logged = False
        self._state.late_execute_ack_key = None
        logs.append(
            "[REEXEC] next 0201 arrival detected -> dispatching 0902 replan request"
            f" (package={package_id}, timestamp={timestamp})"
        )
        return replan_payload, logs

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _allocate_option_ids(self, count: int) -> list[int]:
        ids: list[int] = []
        for _ in range(max(int(count), 0)):
            self._state.option_id_counter += 1
            ids.append(int(self._state.option_id_counter))
        return ids

    def _allocate_plan_ids(self, count: int, *, seed: int) -> list[int]:
        allocated = allocate_mission_plan_ids(count)
        if len(allocated) >= count:
            return [int(v) for v in allocated[:count]]
        # Fallback: derive deterministic IDs from the provided seed.
        next_id = int(seed)
        while len(allocated) < count:
            allocated.append(int(next_id))
            next_id += 1
        return [int(v) for v in allocated[:count]]

    def _build_replan_payload(
        self,
        plan: dict[str, Any],
        package_id: int | None,
        input_ids: Iterable[int],
        *,
        source_input_id: int | None = None,
    ) -> dict[str, Any] | None:
        ts = int(self._now_ms())
        option_count = len(self.OPTION_CODES)
        plan_seed = 700_000_000 + (ts % 1_000)
        plan_ids = self._allocate_plan_ids(option_count, seed=plan_seed)
        option_ids = self._allocate_option_ids(option_count)
        option_codes = ensure_option_code_sequence(self.OPTION_CODES, option_count)

        pending_options: list[dict[str, Any]] = []
        for idx, code in enumerate(option_codes):
            label = option_code_to_label(code)
            pending_options.append(
                {
                    "optionID": int(option_ids[idx]),
                    "optionName": str(label),
                    "missionPlanID": int(plan_ids[idx]),
                }
            )

        mission_ids = list(input_ids) or list(self._state.last_input_ids)
        if not mission_ids and package_id is not None:
            mission_ids = [int(package_id)]
        if not mission_ids:
            mission_ids = [0]
        input_models = [{"inputMissionID": int(mid)} for mid in mission_ids]

        payload: dict[str, Any] = {
            "timestamp": ts,
            "source": "MSM",
            "inputMissionPackageID": int(package_id) if package_id is not None else 0,
            "replanRequestTime": {"replanRequestTimestamp": ts},
            "replanLevel": 1,
            "replanRequest": self.REPLAN_REASON,
            "inputMissionIDList": input_models,
            "pendingOptionList": pending_options,
            "replanDetail": {
                "trigger": "0201",
                "triggerType": "collabReexecuteInputRefresh",
                "inputMissionPackageID": int(package_id) if package_id is not None else 0,
            },
        }
        if source_input_id is not None and int(source_input_id) > 0:
            payload["replanDetail"]["reexecuteSourceInputMissionID"] = int(source_input_id)
        return payload

