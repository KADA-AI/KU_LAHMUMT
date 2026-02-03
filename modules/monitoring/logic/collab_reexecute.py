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
        return sorted(dict.fromkeys(ids_pending))
    if ids_all:
        return sorted(dict.fromkeys(ids_all))
    return []


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

    def on_execute(self, execute: int | None) -> list[str]:
        logs: list[str] = []
        if execute is None:
            return logs
        if int(execute) == 2:
            self._state.pending = True
            self._state.inflight = False
            self._state.wait_logged = False
            self._state.required_new_key = self._state.last_input_key
            self._state.required_new_signature = self._state.last_input_signature
            self._state.last_dispatched_signature = None
            package_id, timestamp = self._state.required_new_key or (None, None)
            logs.append(
                "[REEXEC] execute=2 received -> waiting for updated 0201"
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

    def on_input_plan(self, payload: object | None) -> tuple[dict[str, Any] | None, list[str]]:
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

        if not self._state.pending or self._state.inflight:
            return None, logs

        required_key = self._state.required_new_key
        if (
            required_key is not None
            and current_key == required_key
            and self._state.required_new_signature is not None
            and current_signature == self._state.required_new_signature
        ):
            if not self._state.wait_logged:
                logs.append(
                    "[REEXEC] execute=2 mode active -> still waiting for a newer 0201"
                )
                self._state.wait_logged = True

        if (
            self._state.last_dispatched_signature is not None
            and current_signature == self._state.last_dispatched_signature
        ):
            return None, logs

        replan_payload = self._build_replan_payload(plan, package_id, input_ids)
        if replan_payload is None:
            return None, logs

        self._state.last_dispatched_key = current_key
        self._state.last_dispatched_signature = current_signature
        self._state.pending = False
        self._state.inflight = True
        self._state.required_new_key = current_key
        self._state.required_new_signature = current_signature
        self._state.wait_logged = False
        logs.append(
            "[REEXEC] updated 0201 detected -> dispatching 0902 replan request"
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
        }
        return payload

