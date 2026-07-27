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
from modules.monitoring.logic.replan_runtime_settings import get_input_refresh_settings


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

    def _ordered_unique(values: Iterable[int]) -> list[int]:
        seen: set[int] = set()
        ordered: list[int] = []
        for raw in values:
            try:
                value = int(raw)
            except Exception:
                continue
            if value in seen:
                continue
            seen.add(value)
            ordered.append(value)
        return ordered

    if ids_pending:
        return _ordered_unique(ids_pending)
    if ids_all:
        return _ordered_unique(ids_all)
    return []


def _fingerprint_plan(plan: dict[str, Any]) -> str:
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
class InputRefreshState:
    last_input_key: tuple[int | None, int | None] | None = None
    last_input_signature: str | None = None
    last_dispatched_signature: str | None = None
    last_dispatched_ms: int | None = None
    last_input_ids: list[int] = field(default_factory=list)
    option_id_counter: int = 0


class InputRefreshReplanCoordinator:
    """Handle new 0201 input plans during mission execution by dispatching 0902."""

    # Use the same option codes as other 0902 flows.
    OPTION_CODES: tuple[int, ...] = DEFAULT_OPTION_CODE_SEQUENCE
    REPLAN_REASON = "협업기저임무 재입력에 대한 재계획"
    REPLAN_LEVEL = 3

    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._state = InputRefreshState()

    def on_input_plan(
        self,
        payload: object | None,
        *,
        system_mode: int | None,
        blocked: bool,
        current_mission_plan_id: int | None = None,
        current_input_mission_id: int | None = None,
        replan_reason: str | None = None,
        replan_detail: dict[str, Any] | None = None,
    ) -> tuple[dict[str, Any] | None, list[str]]:
        logs: list[str] = []
        config = get_input_refresh_settings()
        if blocked:
            return None, logs
        if system_mode not in (3, 4):
            return None, logs
        plan = parse_payload(payload)
        if not plan:
            return None, logs

        package_id, timestamp = _extract_input_key(plan)
        current_key = (package_id, timestamp)
        current_signature = _fingerprint_plan(plan)
        self._state.last_input_key = current_key
        self._state.last_input_signature = current_signature

        input_ids = _extract_input_ids(plan)
        if input_ids:
            self._state.last_input_ids = input_ids

        now_ms = int(self._now_ms())
        if (
            self._state.last_dispatched_signature is not None
            and current_signature == self._state.last_dispatched_signature
        ):
            last_ms = self._state.last_dispatched_ms
            duplicate_window_ms = int(config.get("duplicate_window_ms", 400))
            if last_ms is not None and (now_ms - last_ms) < duplicate_window_ms:
                return None, logs

        replan_payload = self._build_replan_payload(
            package_id,
            input_ids,
            current_mission_plan_id=current_mission_plan_id,
            current_input_mission_id=current_input_mission_id,
            replan_reason=replan_reason,
            replan_detail=replan_detail,
        )
        if replan_payload is None:
            return None, logs

        self._state.last_dispatched_signature = current_signature
        self._state.last_dispatched_ms = now_ms
        logs.append(
            "[REINPUT] new 0201 detected during mission execution -> dispatching 0902"
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
        next_id = int(seed)
        while len(allocated) < count:
            allocated.append(int(next_id))
            next_id += 1
        return [int(v) for v in allocated[:count]]

    def _build_replan_payload(
        self,
        package_id: int | None,
        input_ids: Iterable[int],
        *,
        current_mission_plan_id: int | None = None,
        current_input_mission_id: int | None = None,
        replan_reason: str | None = None,
        replan_detail: dict[str, Any] | None = None,
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

        resolved_reason = str(replan_reason or self.REPLAN_REASON).strip() or self.REPLAN_REASON
        detail_payload: dict[str, Any] = {
            "trigger": "0201",
            "triggerType": "inputRefresh",
            "inputMissionPackageID": int(package_id) if package_id is not None else 0,
        }
        if isinstance(replan_detail, dict):
            detail_payload.update(replan_detail)
        current_input_id = _coerce_int(current_input_mission_id)
        if current_input_id is not None and int(current_input_id) > 0:
            detail_payload["currentInputMissionID"] = int(current_input_id)
            detail_payload["preserveCurrentMissionProgress"] = True

        payload: dict[str, Any] = {
            "timestamp": ts,
            "source": "MSM",
            "inputMissionPackageID": int(package_id) if package_id is not None else 0,
            "replanRequestTime": {"replanRequestTimestamp": ts},
            "replanLevel": int(self.REPLAN_LEVEL),
            "replanRequest": resolved_reason,
            "inputMissionIDList": input_models,
            "pendingOptionList": pending_options,
            "replanDetail": detail_payload,
        }
        source_plan_id = _coerce_int(current_mission_plan_id)
        if source_plan_id is not None and int(source_plan_id) > 0:
            payload["sourceMissionPlanID"] = int(source_plan_id)
            payload["currentMissionPlanID"] = int(source_plan_id)
            payload["replanDetail"]["sourceMissionPlanID"] = int(source_plan_id)
            payload["replanDetail"]["currentMissionPlanID"] = int(source_plan_id)
        return payload
