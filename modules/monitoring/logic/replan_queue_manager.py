from __future__ import annotations

import copy
import hashlib
import json
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from modules.common import db_paths
from modules.monitoring.logic.mission_update import parse_payload
from modules.monitoring.logic.replan_runtime_settings import load_replan_settings

_SUPPRESS_FLAG_NAME = "suppress_option_request.json"


def _suppress_flag_path() -> Path:
    return db_paths.get_db_subpath("DSS_Internal", _SUPPRESS_FLAG_NAME)


def write_suppress_option_flag(
    *,
    target_id: int | None,
    target_key: str | None,
    queue_id: int | None = None,
    signature: str | None = None,
    plan_ids: list[int] | None = None,
    created_ms: int | None = None,
) -> Path:
    path = _suppress_flag_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "active": True,
        "target_id": target_id,
        "target_key": target_key,
        "queue_id": queue_id,
        "signature": signature,
        "plan_ids": list(plan_ids or []),
        "created_ms": created_ms,
        "reason": "Target 정보 변경으로 인한 재계획 중단",
    }
    path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8")
    return path


def clear_suppress_option_flag() -> None:
    path = _suppress_flag_path()
    try:
        path.unlink(missing_ok=True)
    except Exception:
        pass


def read_and_clear_suppress_option_flag() -> dict[str, Any] | None:
    path = _suppress_flag_path()
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        data = None
    try:
        path.unlink(missing_ok=True)
    except Exception:
        pass
    if isinstance(data, dict) and data.get("active"):
        return data
    return None


TARGET_TYPE_LABELS: dict[int, str] = {
    1: "전차",
    2: "장갑차",
    3: "방사포",
    4: "곡사포",
    5: "고정고사포",
    6: "군인",
}

SOURCE_LABELS: dict[str, str] = {
    "auto_init": "초기 재계획",
    "dl_risk": "DL 위험도",
    "input_refresh": "입력임무 갱신",
    "collab_reexecute": "재실행",
    "reexecute_0201": "재실행",
    "prior_mission": "선행임무",
    "rtb": "RTB",
    "path_deviation": "경로이탈",
    "quality_speed": "품질-속도",
    "imaging_schedule": "촬영 스케줄",
    "target_detection": "표적탐지",
    "forced_command": "강제명령",
    "forced_hold_due": "강제대기 만료",
    "next_collab": "다음 협업임무",
    "post_attack_rejoin": "공격 후 복귀",
    "manual": "수동",
}

STAGE_LABELS: dict[str, str] = {
    "dispatching": "전송중",
    "waiting_output": "계획 산출 대기",
    "planning_started": "재계획 수행중",
    "planning_finished": "재계획 산출 완료",
    "options_requested": "옵션 요청 전송",
    "options_sent": "옵션 정보 전송",
    "plan_update_sent": "수행임무 갱신",
    "decision_received": "결과 수신",
    "dispatch_failed": "전송 실패",
    "option_suppressed": "옵션 생략(표적 추가)",
    "no_replan_needed": "재계획 불필요",
    "timed_out": "시간 초과",
}


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _normalize_text(value: object) -> str:
    return " ".join(str(value or "").strip().split())


def _extract_plan_ids(payload: dict[str, Any]) -> list[int]:
    out: list[int] = []
    seen: set[int] = set()
    option_blocks = payload.get("pendingOptionList") or payload.get("optionList") or []
    for item in option_blocks:
        if not isinstance(item, dict):
            continue
        plan_id = _coerce_int(item.get("missionPlanID"))
        if plan_id is None or plan_id <= 0 or plan_id in seen:
            continue
        seen.add(plan_id)
        out.append(plan_id)
    if out:
        return out
    for raw in payload.get("missionPlanIDList") or []:
        if isinstance(raw, dict):
            plan_id = _coerce_int(raw.get("missionPlanID"))
        else:
            plan_id = _coerce_int(raw)
        if plan_id is None or plan_id <= 0 or plan_id in seen:
            continue
        seen.add(plan_id)
        out.append(plan_id)
    if out:
        return out
    root_plan_id = _coerce_int(payload.get("missionPlanID") or payload.get("missionPlanId"))
    if root_plan_id is not None and root_plan_id > 0:
        return [root_plan_id]
    return out


def _extract_input_ids(payload: dict[str, Any]) -> list[int]:
    out: list[int] = []
    seen: set[int] = set()
    for item in payload.get("inputMissionIDList") or []:
        if not isinstance(item, dict):
            continue
        mission_id = _coerce_int(item.get("inputMissionID"))
        if mission_id is None or mission_id <= 0 or mission_id in seen:
            continue
        seen.add(mission_id)
        out.append(mission_id)
    return out


def _extract_detail(payload: dict[str, Any]) -> dict[str, Any]:
    detail = payload.get("replanDetail")
    return dict(detail) if isinstance(detail, dict) else {}


def _is_attack_close_detail(detail: dict[str, Any]) -> bool:
    return str(detail.get("triggerType") or "").strip() == "attackClosedDestroyed"


def _is_attack_close_payload(payload: dict[str, Any]) -> bool:
    return _is_attack_close_detail(_extract_detail(payload))


def _is_attack_close_item(item: Any) -> bool:
    payload = getattr(item, "payload", None)
    return isinstance(payload, dict) and _is_attack_close_payload(payload)


def _is_normal_target_detection_item(item: Any) -> bool:
    return str(getattr(item, "source_tag", "") or "").strip() == "target_detection" and not _is_attack_close_item(item)


def _detail_signature_payload(detail: dict[str, Any]) -> dict[str, Any]:
    cleaned = {}
    for key, value in detail.items():
        if key in {"snapshot", "raw", "rawEntry", "timestamp", "timeStamp"}:
            continue
        cleaned[key] = value
    return cleaned


def _extract_target_type(detail: dict[str, Any]) -> int | None:
    value = _coerce_int(detail.get("targetType"))
    if value is not None and value > 0:
        return value
    return None


def _extract_attack_target_ids(detail: dict[str, Any]) -> list[int]:
    seen: set[int] = set()
    out: list[int] = []

    for raw in detail.get("attackTargetIDs") or []:
        target_id = _coerce_int(raw)
        if target_id is None or target_id <= 0 or target_id in seen:
            continue
        seen.add(target_id)
        out.append(target_id)

    for field in ("targetBundle", "attackTargetList"):
        for item in detail.get(field) or []:
            if not isinstance(item, dict):
                continue
            target_id = _coerce_int(item.get("targetID") or item.get("targetId"))
            if target_id is None or target_id <= 0 or target_id in seen:
                continue
            seen.add(target_id)
            out.append(target_id)

    primary_target_id = _coerce_int(detail.get("targetID") or detail.get("targetId"))
    if primary_target_id is not None and primary_target_id > 0 and primary_target_id not in seen:
        seen.add(primary_target_id)
        out.append(primary_target_id)

    out.sort()
    return out


def _build_signature(source_tag: str, payload: dict[str, Any]) -> str:
    detail = _extract_detail(payload)
    trigger_name = _normalize_text(detail.get("trigger") or detail.get("triggerType"))
    if trigger_name == "0402":
        signature_payload = {
            "source_tag": str(source_tag or "manual"),
            "replan_level": _coerce_int(payload.get("replanLevel")),
            "trigger": "0402",
            "input_ids": _extract_input_ids(payload),
            "source_plan_id": _coerce_int(
                detail.get("sourceMissionPlanID")
                or payload.get("sourceMissionPlanID")
            ),
            "current_plan_id": _coerce_int(
                detail.get("currentMissionPlanID")
                or payload.get("currentMissionPlanID")
            ),
            "bundle_target_ids": _extract_attack_target_ids(detail),
            "bundle_count": _coerce_int(detail.get("attackTargetCount") or detail.get("targetBundleCount")),
            "follow_up": bool(detail.get("followUpAttackMode")),
        }
    else:
        signature_payload = {
            "source_tag": str(source_tag or "manual"),
            "replan_level": _coerce_int(payload.get("replanLevel")),
            "reason": _normalize_text(payload.get("replanRequest") or payload.get("replanReason")),
            "trigger": trigger_name,
            "aircraft_id": _coerce_int(detail.get("aircraftID") or detail.get("aircraftId")),
            "target_id": _coerce_int(detail.get("targetID") or detail.get("targetId")),
            "target_type": _coerce_int(detail.get("targetType")),
            "watcher_id": _coerce_int(detail.get("watcherID") or detail.get("watcherId")),
            "prior_mission_id": _coerce_int(detail.get("priorMissionID")),
            "mandatory_type": _coerce_int(detail.get("mandatoryType")),
            "input_ids": _extract_input_ids(payload),
            "detail": _detail_signature_payload(detail),
        }
    raw = json.dumps(signature_payload, ensure_ascii=False, sort_keys=True, default=str)
    return hashlib.sha1(raw.encode("utf-8", "ignore")).hexdigest()


def _source_label(source_tag: str, detail: dict[str, Any]) -> str:
    if str(detail.get("triggerType") or "").strip() == "attackClosedDestroyed":
        return "공격 후 복귀"
    if str(detail.get("triggerType") or "").strip() == "priorClosedResume":
        return "선행임무 후 복귀"
    normalized = str(source_tag or "").strip()
    if normalized:
        return SOURCE_LABELS.get(normalized, normalized)
    trigger = str(detail.get("trigger") or detail.get("triggerType") or "").strip()
    return trigger or SOURCE_LABELS["manual"]


def _extract_signal_plan_ids(signal_name: str, payload: object | None) -> list[int]:
    body = parse_payload(payload)
    if not body:
        return []
    if signal_name in {"0901", "0701"}:
        return _extract_plan_ids(body)
    if signal_name in {"0903", "0702"}:
        plan_id = _coerce_int(body.get("missionPlanID") or body.get("missionPlanId"))
        return [plan_id] if plan_id is not None and plan_id > 0 else []
    return []


def _extract_signal_stage(signal_name: str, payload: object | None) -> str | None:
    body = parse_payload(payload)
    if signal_name == "0305":
        status = _coerce_int(body.get("missionPlanningStatus"))
        if status == 1:
            return "planning_started"
        if status == 2:
            return "planning_finished"
        return None
    if signal_name == "0901":
        return "options_requested"
    if signal_name == "0701":
        return "options_sent"
    if signal_name == "0903":
        return "plan_update_sent"
    if signal_name == "0702":
        return "decision_received"
    return None


def _is_failure_completion_0305(payload: object | None) -> bool:
    body = parse_payload(payload)
    if not isinstance(body, dict):
        return False
    if _coerce_int(body.get("missionPlanningStatus")) != 2:
        return False
    reason = _normalize_text(body.get("replanReason") or body.get("reason") or "")
    reason_lower = reason.lower()
    return (
        "\uc2e4\ud328" in reason
        or "\uc624\ub958" in reason
        or "fail" in reason_lower
        or "error" in reason_lower
    )


def _is_noop_completion_0305(payload: object | None) -> bool:
    body = parse_payload(payload)
    if not isinstance(body, dict):
        return False
    if _coerce_int(body.get("missionPlanningStatus")) != 2:
        return False
    reason = _normalize_text(body.get("replanReason") or body.get("reason") or "")
    reason_lower = reason.lower()
    return (
        "\ubd88\ud544\uc694" in reason
        or "\uae30\uc874 \uc784\ubb34" in reason
        or "replan_not_needed" in reason_lower
        or "no replan" in reason_lower
    )


@dataclass
class _QueueItem:
    queue_id: int
    source_tag: str
    source_label: str
    payload: dict[str, Any]
    signature: str
    reason: str
    replan_level: int | None
    trigger_name: str
    stage: str
    status: str
    created_ms: int
    ready_at_ms: int
    plan_ids: list[int]
    input_ids: list[int]
    target_id: int | None
    target_type: int | None
    watcher_id: int | None
    aircraft_id: int | None
    prior_mission_id: int | None
    mandatory_type: int | None
    duplicates: int = 0
    suppress_options: bool = False
    options_delivered: bool = False
    dispatched_ms: int | None = None
    completed_ms: int | None = None
    timeout_at_ms: int | None = None
    last_signal: str | None = None
    completion_signal: str | None = None

    def public_dict(self, now_ms: int | None = None) -> dict[str, Any]:
        age_ms = None
        active_ms = None
        if now_ms is not None:
            age_ms = max(0, int(now_ms) - int(self.created_ms))
            if self.dispatched_ms is not None:
                active_end_ms = int(self.completed_ms) if self.completed_ms is not None else int(now_ms)
                active_ms = max(0, active_end_ms - int(self.dispatched_ms))
        return {
            "queue_id": int(self.queue_id),
            "source_tag": str(self.source_tag),
            "source_label": str(self.source_label),
            "reason": str(self.reason),
            "replan_level": self.replan_level,
            "trigger_name": str(self.trigger_name),
            "stage": str(self.stage),
            "stage_label": STAGE_LABELS.get(self.stage, self.stage),
            "status": str(self.status),
            "created_ms": int(self.created_ms),
            "ready_at_ms": int(self.ready_at_ms),
            "dispatched_ms": self.dispatched_ms,
            "completed_ms": self.completed_ms,
            "timeout_at_ms": self.timeout_at_ms,
            "age_ms": age_ms,
            "active_ms": active_ms,
            "plan_ids": list(self.plan_ids),
            "input_ids": list(self.input_ids),
            "target_id": self.target_id,
            "target_type": self.target_type,
            "target_type_label": TARGET_TYPE_LABELS.get(int(self.target_type or 0), "-")
            if self.target_type is not None
            else "-",
            "watcher_id": self.watcher_id,
            "aircraft_id": self.aircraft_id,
            "prior_mission_id": self.prior_mission_id,
            "mandatory_type": self.mandatory_type,
            "duplicates": int(self.duplicates),
            "suppress_options": bool(self.suppress_options),
            "options_delivered": bool(self.options_delivered),
            "last_signal": self.last_signal,
            "completion_signal": self.completion_signal,
        }


class ReplanQueueManager:
    def __init__(
        self,
        *,
        now_fn: Callable[[], int] | None = None,
        logger: Callable[[str], None] | None = None,
        settings_getter: Callable[[], dict[str, Any]] = load_replan_settings,
    ) -> None:
        self._now_fn = now_fn or (lambda: 0)
        self._logger = logger
        self._settings_getter = settings_getter
        self._lock = threading.Lock()
        self._next_id = 1
        self._active: _QueueItem | None = None
        self._queue: deque[_QueueItem] = deque()
        self._history: deque[_QueueItem] = deque()
        self._copy_metrics: dict[str, dict[str, float | int]] = {}

    def build_snapshot(self) -> dict[str, Any]:
        return self.snapshot(now_ms=self._now_fn())

    def snapshot(self, *, now_ms: int | None = None) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_locked(now_ms=now_ms)

    def enqueue(
        self,
        *,
        payload: dict[str, Any],
        source_tag: str | None,
        now_ms: int,
        include_dispatch_payload: bool = True,
    ) -> dict[str, Any]:
        with self._lock:
            item = self._build_item_locked(payload=payload, source_tag=source_tag, now_ms=now_ms)
            duplicate = self._find_duplicate_locked(item.signature)
            if duplicate is not None:
                duplicate.duplicates += 1
                duplicate.last_signal = "duplicate"
                return self._result_locked(
                    now_ms=now_ms,
                    events=[
                        {
                            "type": "duplicate",
                            "item": duplicate.public_dict(now_ms=now_ms),
                        }
                    ],
                )

            merged_item = self._merge_target_detection_queue_item_locked(item=item, now_ms=now_ms)
            if merged_item is not None:
                merged_public, removed_count = merged_item
                events = [
                    {
                        "type": "merged",
                        "item": merged_public.public_dict(now_ms=now_ms),
                        "removed_count": int(removed_count),
                    }
                ]
                suppressed_item = self._ensure_target_detection_suppression_locked(now_ms=now_ms)
                if suppressed_item is not None:
                    events.append(
                        {
                            "type": "suppress_options_set",
                            "item": suppressed_item.public_dict(now_ms=now_ms),
                        }
                    )
                return self._result_locked(now_ms=now_ms, events=events)

            preempt_event = self._preempt_active_target_detection_for_attack_close_locked(
                item=item,
                now_ms=now_ms,
            )
            if preempt_event is not None and int(item.ready_at_ms) <= int(now_ms):
                item.status = "dispatching"
                item.stage = "dispatching"
                self._active = item
                return self._result_locked(
                    now_ms=now_ms,
                    events=[
                        preempt_event,
                        {
                            "type": "dispatching",
                            "item": item.public_dict(now_ms=now_ms),
                        },
                    ],
                    dispatch=item,
                    include_dispatch_payload=include_dispatch_payload,
                )

            if self._active is None and not self._queue and int(item.ready_at_ms) <= int(now_ms):
                item.status = "dispatching"
                item.stage = "dispatching"
                self._active = item
                return self._result_locked(
                    now_ms=now_ms,
                    events=[
                        {
                            "type": "dispatching",
                            "item": item.public_dict(now_ms=now_ms),
                        }
                    ],
                    dispatch=item,
                    include_dispatch_payload=include_dispatch_payload,
                )

            item.status = "queued"
            item.stage = "queued"
            self._queue_append_locked(item)

            events: list[dict[str, Any]] = [
                {
                    "type": "queued",
                    "item": item.public_dict(now_ms=now_ms),
                }
            ]
            suppressed_item = self._ensure_target_detection_suppression_locked(now_ms=now_ms)
            if suppressed_item is not None:
                events.append(
                    {
                        "type": "suppress_options_set",
                        "item": suppressed_item.public_dict(now_ms=now_ms),
                    }
                )
            return self._result_locked(now_ms=now_ms, events=events)

    def confirm_dispatch(self, *, queue_id: int, success: bool, now_ms: int) -> dict[str, Any]:
        with self._lock:
            active = self._active
            if active is None or int(active.queue_id) != int(queue_id):
                return self._result_locked(now_ms=now_ms)

            if success:
                active.status = "active"
                active.stage = "waiting_output"
                active.dispatched_ms = int(now_ms)
                active.timeout_at_ms = int(now_ms) + int(self._timeout_ms_locked())
                return self._result_locked(
                    now_ms=now_ms,
                    events=[
                        {
                            "type": "active",
                            "item": active.public_dict(now_ms=now_ms),
                        }
                    ],
                )

            return self._complete_active_locked(
                status="dispatch_failed",
                stage="dispatch_failed",
                now_ms=now_ms,
                completion_signal="0902",
            )

    def handle_signal(self, *, signal_name: str, payload: object | None, now_ms: int) -> dict[str, Any]:
        signal_name = str(signal_name or "").strip().upper()
        with self._lock:
            active = self._active
            if active is None:
                return self._update_history_signal_locked(
                    signal_name=signal_name,
                    payload=payload,
                    now_ms=now_ms,
                )

            if not self._signal_matches_active_locked(signal_name, payload, active):
                return self._update_history_signal_locked(
                    signal_name=signal_name,
                    payload=payload,
                    now_ms=now_ms,
                )

            stage = _extract_signal_stage(signal_name, payload)
            if not stage:
                return self._result_locked(now_ms=now_ms)

            if signal_name == "0701":
                active.options_delivered = True
            active.last_signal = signal_name
            active.stage = stage
            if signal_name == "0305" and _is_failure_completion_0305(payload):
                return self._complete_active_locked(
                    status="dispatch_failed",
                    stage="dispatch_failed",
                    now_ms=now_ms,
                    completion_signal=signal_name,
                )
            if signal_name == "0305" and _is_noop_completion_0305(payload):
                return self._complete_active_locked(
                    status="completed",
                    stage="no_replan_needed",
                    now_ms=now_ms,
                    completion_signal=signal_name,
                )
            release_on_option_info = self._release_on_option_info_locked()
            should_complete = signal_name in {"0903", "0702"} or (
                signal_name == "0701" and release_on_option_info
            )
            if should_complete:
                return self._complete_active_locked(
                    status="completed",
                    stage=stage,
                    now_ms=now_ms,
                    completion_signal=signal_name,
                )

            return self._result_locked(
                now_ms=now_ms,
                events=[
                    {
                        "type": "signal",
                        "signal_name": signal_name,
                        "item": active.public_dict(now_ms=now_ms),
                    }
                ],
            )

    def poll(self, *, now_ms: int) -> dict[str, Any]:
        with self._lock:
            active = self._active
            if active is None:
                next_item = self._promote_ready_item_locked(now_ms=now_ms)
                if next_item is None:
                    return self._result_locked(now_ms=now_ms)
                return self._result_locked(
                    now_ms=now_ms,
                    events=[
                        {
                            "type": "dispatching",
                            "item": next_item.public_dict(now_ms=now_ms),
                        }
                    ],
                    dispatch=next_item,
                )
            if active.timeout_at_ms is None:
                return self._result_locked(now_ms=now_ms)
            if int(now_ms) < int(active.timeout_at_ms):
                return self._result_locked(now_ms=now_ms)
            return self._complete_active_locked(
                status="timed_out",
                stage="timed_out",
                now_ms=now_ms,
                completion_signal="timeout",
            )

    def enqueue_payloads(self, payloads: list[dict[str, Any]], *, source: str) -> list[str]:
        logs: list[str] = []
        now_ms = int(self._now_fn())
        ordered_payloads = self._order_payloads_for_source(payloads or [], source=source)
        for payload in ordered_payloads:
            result = self.enqueue(
                payload=payload,
                source_tag=source,
                now_ms=now_ms,
                include_dispatch_payload=False,
            )
            logs.extend(self._logs_from_result(result))
        return logs

    def try_activate_next(self) -> tuple[int, dict[str, Any]] | None:
        with self._lock:
            if self._active is not None and self._active.status == "dispatching":
                return int(self._active.queue_id), self._deepcopy_locked(
                    self._active.payload,
                    label="activate_payload",
                )
            if self._active is not None or not self._queue:
                return None
            next_item = self._promote_ready_item_locked(now_ms=int(self._now_fn()))
            if next_item is None:
                return None
            return int(next_item.queue_id), self._deepcopy_locked(
                next_item.payload,
                label="activate_payload",
            )

    def mark_dispatch_result(self, queue_id: int, *, sent: bool, error: str | None = None) -> list[str]:
        result = self.confirm_dispatch(queue_id=queue_id, success=bool(sent), now_ms=int(self._now_fn()))
        logs = self._logs_from_result(result)
        if error and not sent:
            logs.append(f"[RQUEUE] dispatch failed: queueID={queue_id}, error={error}")
        return logs

    def handle_0305(self, payload: object | None) -> tuple[bool, list[str]]:
        result = self.handle_signal(signal_name="0305", payload=payload, now_ms=int(self._now_fn()))
        return bool(result.get("dispatch")), self._logs_from_result(result)

    def handle_0701(self, payload: object | None) -> tuple[bool, list[str]]:
        result = self.handle_signal(signal_name="0701", payload=payload, now_ms=int(self._now_fn()))
        return bool(result.get("dispatch")), self._logs_from_result(result)

    def handle_0903(self, payload: object | None) -> tuple[bool, list[str]]:
        result = self.handle_signal(signal_name="0903", payload=payload, now_ms=int(self._now_fn()))
        return bool(result.get("dispatch")), self._logs_from_result(result)

    def handle_0702(self, payload: object | None) -> tuple[bool, list[str]]:
        result = self.handle_signal(signal_name="0702", payload=payload, now_ms=int(self._now_fn()))
        return bool(result.get("dispatch")), self._logs_from_result(result)

    def handle_0001(self, payload: object | None) -> tuple[bool, list[str]]:
        result = self._handle_failure_notice(payload=payload, now_ms=int(self._now_fn()))
        return bool(result.get("dispatch")), self._logs_from_result(result)

    def poll_due_transitions(self) -> tuple[bool, list[str]]:
        result = self.poll(now_ms=int(self._now_fn()))
        return bool(result.get("dispatch")), self._logs_from_result(result)

    def drop_queued_target_detection_targets(self, target_ids: list[int] | set[int] | tuple[int, ...]) -> list[str]:
        normalized_ids: set[int] = set()
        for value in target_ids or []:
            try:
                target_id = int(value)
            except Exception:
                continue
            if target_id > 0:
                normalized_ids.add(target_id)
        if not normalized_ids:
            return []

        with self._lock:
            if not self._queue:
                return []

            kept: list[_QueueItem] = []
            removed: list[_QueueItem] = []
            for item in self._queue:
                item_target_ids: set[int] = set()
                primary_id = _coerce_int(item.target_id)
                if primary_id is not None:
                    item_target_ids.add(int(primary_id))
                if _is_normal_target_detection_item(item):
                    try:
                        item_target_ids.update(_extract_attack_target_ids(_extract_detail(item.payload)))
                    except Exception:
                        pass
                if _is_normal_target_detection_item(item) and item_target_ids.intersection(normalized_ids):
                    removed.append(item)
                    continue
                kept.append(item)

            if not removed:
                return []

            self._queue = deque(kept)
            details = ", ".join(
                f"#{item.queue_id} targetID={item.target_id} ({item.reason})" for item in removed[:8]
            )
            if len(removed) > 8:
                details += f" 외 {len(removed) - 8}건"
            return [f"[RQUEUE] dropped queued target detections: {details}"]

    def _logs_from_result(self, result: dict[str, Any]) -> list[str]:
        logs: list[str] = []
        for event in result.get("events") or []:
            if not isinstance(event, dict):
                continue
            item = event.get("item") or {}
            queue_id = item.get("queue_id")
            source_label = item.get("source_label") or "재계획"
            reason = item.get("reason") or "-"
            stage_label = item.get("stage_label") or item.get("stage") or "-"
            event_type = str(event.get("type") or "")
            if event_type == "queued":
                logs.append(f"[RQUEUE] queued #{queue_id} {source_label}: {reason}")
            elif event_type == "dispatching":
                logs.append(f"[RQUEUE] dispatching #{queue_id} {source_label}: {reason}")
            elif event_type == "active":
                logs.append(f"[RQUEUE] active #{queue_id} {source_label}: {reason}")
            elif event_type == "duplicate":
                logs.append(f"[RQUEUE] duplicate suppressed #{queue_id} {source_label}: {reason}")
            elif event_type == "merged":
                removed_count = _coerce_int(event.get("removed_count")) or 0
                logs.append(
                    f"[RQUEUE] merged queued #{queue_id} {source_label}: {reason} "
                    f"(staleMerged={removed_count})"
                )
            elif event_type == "resume_after_post_attack":
                logs.append(
                    f"[RQUEUE] 0001 resume attack planning after post-attack apply -> "
                    f"#{queue_id} {source_label}: {reason}"
                )
            elif event_type == "signal":
                signal_name = event.get("signal_name") or "-"
                logs.append(f"[RQUEUE] signal {signal_name} -> #{queue_id} {stage_label}")
            elif event_type == "suppress_options_set":
                target_id = item.get("target_id")
                logs.append(
                    f"[RQUEUE] suppress_options set on active #{queue_id} "
                    f"(targetID={target_id}): 추가 표적 감지로 옵션 생략 예정"
                )
            elif event_type == "completed":
                completion_signal = item.get("completion_signal") or "-"
                active_ms = item.get("active_ms")
                active_suffix = f", active_ms={active_ms}" if active_ms is not None else ""
                logs.append(
                    f"[RQUEUE] completed #{queue_id} {source_label}: {reason} "
                    f"({stage_label}, signal={completion_signal}{active_suffix})"
                )
            elif event_type == "history_signal":
                signal_name = event.get("signal_name") or "-"
                logs.append(f"[RQUEUE] history {signal_name} -> #{queue_id} {stage_label}")
        return logs

    def _timeout_ms_locked(self) -> int:
        cfg = self._queue_cfg_locked()
        return max(1000, _coerce_int(cfg.get("active_timeout_ms")) or 45000)

    def _history_limit_locked(self) -> int:
        cfg = self._queue_cfg_locked()
        return max(5, _coerce_int(cfg.get("history_limit")) or 30)

    def _target_dispatch_delay_ms_locked(self) -> int:
        cfg = self._queue_cfg_locked()
        return max(0, _coerce_int(cfg.get("target_dispatch_delay_ms")) or 0)

    def _release_on_option_info_locked(self) -> bool:
        cfg = self._queue_cfg_locked()
        return bool(cfg.get("release_on_option_info", False))

    def _suppress_active_target_options_on_new_detection_locked(self) -> bool:
        cfg = self._queue_cfg_locked()
        return bool(cfg.get("suppress_active_target_options_on_new_detection", False))

    def _initial_ready_at_ms_locked(self, *, source_tag: str, now_ms: int) -> int:
        if str(source_tag) != "target_detection":
            return int(now_ms)
        delay_ms = int(self._target_dispatch_delay_ms_locked())
        if delay_ms <= 0 or self._active is not None:
            return int(now_ms)
        existing_ready = [
            int(item.ready_at_ms)
            for item in self._queue
            if str(item.source_tag) == "target_detection"
        ]
        if existing_ready:
            return max(int(now_ms), min(existing_ready))
        return int(now_ms) + delay_ms

    def _find_duplicate_locked(self, signature: str) -> _QueueItem | None:
        if self._active is not None and self._active.signature == signature:
            return self._active
        for item in self._queue:
            if item.signature == signature:
                return item
        return None

    def _signal_matches_active_locked(
        self,
        signal_name: str,
        payload: object | None,
        active: _QueueItem,
    ) -> bool:
        if signal_name == "0305":
            return True
        signal_plan_ids = _extract_signal_plan_ids(signal_name, payload)
        if not signal_plan_ids:
            return False
        if not active.plan_ids:
            return True
        return bool(set(signal_plan_ids) & set(active.plan_ids))

    def _queue_append_locked(self, item: _QueueItem) -> None:
        items = list(self._queue)
        items.append(item)
        items.sort(key=self._queue_sort_key_locked)
        self._queue = deque(items)

    def _merge_target_detection_queue_item_locked(
        self,
        *,
        item: _QueueItem,
        now_ms: int,
    ) -> tuple[_QueueItem, int] | None:
        if not _is_normal_target_detection_item(item):
            return None
        queued_target_items = [
            queued for queued in self._queue if _is_normal_target_detection_item(queued)
        ]
        if not queued_target_items:
            return None

        survivor = queued_target_items[-1]
        ready_at_ms = min(
            [int(item.ready_at_ms), *[int(queued.ready_at_ms) for queued in queued_target_items]]
        )
        removed_count = max(0, len(queued_target_items) - 1)

        survivor.payload = self._deepcopy_locked(item.payload, label="merge_target_payload")
        survivor.signature = str(item.signature)
        survivor.reason = str(item.reason)
        survivor.replan_level = item.replan_level
        survivor.trigger_name = str(item.trigger_name)
        survivor.source_label = str(item.source_label)
        survivor.created_ms = int(now_ms)
        survivor.ready_at_ms = int(ready_at_ms)
        survivor.plan_ids = list(item.plan_ids)
        survivor.input_ids = list(item.input_ids)
        survivor.target_id = item.target_id
        survivor.target_type = item.target_type
        survivor.watcher_id = item.watcher_id
        survivor.aircraft_id = item.aircraft_id
        survivor.prior_mission_id = item.prior_mission_id
        survivor.mandatory_type = item.mandatory_type
        survivor.status = "queued"
        survivor.stage = "queued"
        survivor.suppress_options = False
        survivor.options_delivered = False
        survivor.dispatched_ms = None
        survivor.completed_ms = None
        survivor.timeout_at_ms = None
        survivor.last_signal = None
        survivor.completion_signal = None

        kept: list[_QueueItem] = []
        for queued in self._queue:
            if not _is_normal_target_detection_item(queued):
                kept.append(queued)
                continue
            if queued is survivor:
                continue
        kept.append(survivor)
        kept.sort(key=self._queue_sort_key_locked)
        self._queue = deque(kept)
        return survivor, removed_count

    def _queue_sort_key_locked(self, item: _QueueItem) -> tuple[int, int, int, int]:
        ready_at_ms = int(item.ready_at_ms or item.created_ms)
        if _is_normal_target_detection_item(item):
            return (
                ready_at_ms,
                1,
                self._target_priority_rank_locked(item.target_type),
                int(item.queue_id),
            )
        return (
            ready_at_ms,
            0,
            int(item.queue_id),
            int(item.queue_id),
        )

    def _target_priority_rank_locked(self, target_type: int | None) -> int:
        priority_order = self._target_priority_order_locked()
        rank_map = {value: index for index, value in enumerate(priority_order)}
        try:
            target_value = int(target_type) if target_type is not None else 0
        except Exception:
            target_value = 0
        return rank_map.get(target_value, len(rank_map) + 100)

    def _promote_ready_item_locked(self, *, now_ms: int) -> _QueueItem | None:
        if self._active is not None or not self._queue:
            return None
        next_item = self._queue[0]
        if int(next_item.ready_at_ms) > int(now_ms):
            return None
        next_item = self._queue.popleft()
        next_item.status = "dispatching"
        next_item.stage = "dispatching"
        self._active = next_item
        suppressed_item = self._ensure_target_detection_suppression_locked(now_ms=now_ms)
        if suppressed_item is not None and callable(self._logger):
            try:
                for line in self._logs_from_result(
                    {
                        "events": [
                            {
                                "type": "suppress_options_set",
                                "item": suppressed_item.public_dict(now_ms=now_ms),
                            }
                        ]
                    }
                ):
                    self._logger(line)
            except Exception:
                pass
        return next_item

    def _ensure_target_detection_suppression_locked(self, *, now_ms: int) -> _QueueItem | None:
        if not self._suppress_active_target_options_on_new_detection_locked():
            return None
        outstanding: list[_QueueItem] = []
        if self._active is not None and _is_normal_target_detection_item(self._active):
            outstanding.append(self._active)
        outstanding.extend(
            item for item in self._queue if _is_normal_target_detection_item(item)
        )
        if len(outstanding) < 2:
            return None
        candidate = outstanding[0]
        if bool(candidate.options_delivered) or str(candidate.stage or "") in {"options_sent", "decision_received"}:
            return None
        if bool(candidate.suppress_options):
            return None
        detail = candidate.payload.get("replanDetail") or {}
        try:
            write_suppress_option_flag(
                target_id=candidate.target_id,
                target_key=str(detail.get("targetKey") or "") or None,
                queue_id=int(candidate.queue_id),
                signature=str(candidate.signature),
                plan_ids=list(candidate.plan_ids),
                created_ms=int(now_ms),
            )
        except Exception:
            return None
        candidate.suppress_options = True
        return candidate

    def _signal_matches_history_locked(
        self,
        signal_name: str,
        payload: object | None,
        item: _QueueItem,
    ) -> bool:
        if signal_name == "0305":
            return False
        signal_plan_ids = _extract_signal_plan_ids(signal_name, payload)
        if not signal_plan_ids or not item.plan_ids:
            return False
        return bool(set(signal_plan_ids) & set(item.plan_ids))

    def _update_history_signal_locked(
        self,
        *,
        signal_name: str,
        payload: object | None,
        now_ms: int,
    ) -> dict[str, Any]:
        stage = _extract_signal_stage(signal_name, payload)
        if not stage:
            return self._result_locked(now_ms=now_ms)
        for item in self._history:
            if not self._signal_matches_history_locked(signal_name, payload, item):
                continue
            item.last_signal = signal_name
            item.stage = stage
            if signal_name in {"0702", "0903"}:
                item.completion_signal = signal_name
                item.completed_ms = int(now_ms)
            return self._result_locked(
                now_ms=now_ms,
                events=[
                    {
                        "type": "history_signal",
                        "signal_name": signal_name,
                        "item": item.public_dict(now_ms=now_ms),
                    }
                ],
            )
        return self._result_locked(now_ms=now_ms)

    def _preempt_active_target_detection_for_attack_close_locked(
        self,
        *,
        item: _QueueItem,
        now_ms: int,
    ) -> dict[str, Any] | None:
        active = self._active
        if active is None:
            return None
        if not _is_attack_close_item(item):
            return None
        if not _is_normal_target_detection_item(active):
            return None
        if str(active.stage or "") not in {"planning_finished", "options_requested", "options_sent"}:
            return None

        active.status = "option_suppressed"
        active.stage = "option_suppressed"
        active.completed_ms = int(now_ms)
        active.timeout_at_ms = None
        active.completion_signal = "post_attack_preempt"
        self._history.appendleft(active)
        while len(self._history) > self._history_limit_locked():
            self._history.pop()
        self._active = None
        return {
            "type": "completed",
            "item": active.public_dict(now_ms=now_ms),
        }

    def _handle_failure_notice(self, *, payload: object | None, now_ms: int) -> dict[str, Any]:
        body = parse_payload(payload)
        if not isinstance(body, dict):
            return self._result_locked(now_ms=now_ms)
        source = str(body.get("source") or "").strip().upper()
        contents = _normalize_text(body.get("contents") or "")
        contents_lower = contents.lower()
        if source != "MMR":
            return self._result_locked(now_ms=now_ms)
        is_noop_notice = (
            "재계획 불필요" in contents
            or "replan_not_needed" in contents_lower
            or "no replan" in contents_lower
        )
        is_suppress_notice = (
            is_noop_notice
            or "재계획 중단" in contents
            or "target 정보 변경" in contents_lower
        )
        is_failure_notice = (
            ("임무계획" in contents and "실패" in contents)
            or ("mission" in contents_lower and ("fail" in contents_lower or "error" in contents_lower))
            or ("planning" in contents_lower and ("fail" in contents_lower or "error" in contents_lower))
        )
        if not is_failure_notice and not is_suppress_notice:
            return self._result_locked(now_ms=now_ms)
        if self._active is None:
            return self._result_locked(now_ms=now_ms)
        if is_noop_notice:
            return self._complete_active_locked(
                status="completed",
                stage="no_replan_needed",
                now_ms=now_ms,
                completion_signal="0001",
            )
        if is_suppress_notice:
            return self._complete_active_locked(
                status="option_suppressed",
                stage="option_suppressed",
                now_ms=now_ms,
                completion_signal="0001",
            )
        return self._complete_active_locked(
            status="dispatch_failed",
            stage="dispatch_failed",
            now_ms=now_ms,
            completion_signal="0001",
        )

    def _complete_active_locked(
        self,
        *,
        status: str,
        stage: str,
        now_ms: int,
        completion_signal: str,
    ) -> dict[str, Any]:
        events: list[dict[str, Any]] = []
        active = self._active
        if active is None:
            return self._result_locked(now_ms=now_ms)

        active.status = str(status)
        active.stage = str(stage)
        active.completed_ms = int(now_ms)
        active.timeout_at_ms = None
        active.completion_signal = str(completion_signal)
        if bool(active.suppress_options):
            clear_suppress_option_flag()
        self._history.appendleft(active)
        while len(self._history) > self._history_limit_locked():
            self._history.pop()
        events.append(
            {
                "type": "completed",
                "item": active.public_dict(now_ms=now_ms),
            }
        )
        self._active = None

        next_item: _QueueItem | None = None
        next_item = self._promote_ready_item_locked(now_ms=now_ms)
        if next_item is not None:
            if _is_attack_close_item(active) and _is_normal_target_detection_item(next_item):
                events.append(
                    {
                        "type": "resume_after_post_attack",
                        "item": next_item.public_dict(now_ms=now_ms),
                    }
                )
            events.append(
                {
                    "type": "dispatching",
                    "item": next_item.public_dict(now_ms=now_ms),
                }
            )
        return self._result_locked(now_ms=now_ms, events=events, dispatch=next_item)

    def _build_item_locked(
        self,
        *,
        payload: dict[str, Any],
        source_tag: str | None,
        now_ms: int,
    ) -> _QueueItem:
        source_value = str(source_tag or "").strip() or "manual"
        payload_copy = self._deepcopy_locked(payload, label="enqueue_payload")
        detail = _extract_detail(payload_copy)
        queue_id = int(self._next_id)
        self._next_id += 1
        return _QueueItem(
            queue_id=queue_id,
            source_tag=source_value,
            source_label=_source_label(source_value, detail),
            payload=payload_copy,
            signature=_build_signature(source_value, payload_copy),
            reason=_normalize_text(payload_copy.get("replanRequest") or payload_copy.get("replanReason") or "-"),
            replan_level=_coerce_int(payload_copy.get("replanLevel")),
            trigger_name=_normalize_text(detail.get("trigger") or detail.get("triggerType") or source_value),
            stage="queued",
            status="queued",
            created_ms=int(now_ms),
            ready_at_ms=self._initial_ready_at_ms_locked(source_tag=source_value, now_ms=int(now_ms)),
            plan_ids=_extract_plan_ids(payload_copy),
            input_ids=_extract_input_ids(payload_copy),
            target_id=_coerce_int(detail.get("targetID") or detail.get("targetId")),
            target_type=_extract_target_type(detail),
            watcher_id=_coerce_int(detail.get("watcherID") or detail.get("watcherId")),
            aircraft_id=_coerce_int(detail.get("aircraftID") or detail.get("aircraftId")),
            prior_mission_id=_coerce_int(detail.get("priorMissionID")),
            mandatory_type=_coerce_int(detail.get("mandatoryType")),
        )

    def _result_locked(
        self,
        *,
        now_ms: int,
        events: list[dict[str, Any]] | None = None,
        dispatch: _QueueItem | None = None,
        include_dispatch_payload: bool = True,
    ) -> dict[str, Any]:
        payload = None
        if dispatch is not None and include_dispatch_payload:
            payload = self._deepcopy_locked(dispatch.payload, label="result_dispatch_payload")
        return {
            "snapshot": self._snapshot_locked(now_ms=now_ms),
            "events": list(events or []),
            "dispatch": {
                "queue_id": int(dispatch.queue_id),
                "payload": payload,
                "item": dispatch.public_dict(now_ms=now_ms),
            }
            if dispatch is not None
            else None,
        }

    def _snapshot_locked(self, *, now_ms: int | None = None) -> dict[str, Any]:
        queue_cfg = self._queue_cfg_locked()
        return {
            "blocked": self._active is not None,
            "active": self._active.public_dict(now_ms=now_ms) if self._active is not None else None,
            "queued": [item.public_dict(now_ms=now_ms) for item in self._queue],
            "history": [item.public_dict(now_ms=now_ms) for item in self._history],
            "stats": {
                "queued_count": len(self._queue),
                "history_count": len(self._history),
                "active_timeout_ms": max(1000, _coerce_int(queue_cfg.get("active_timeout_ms")) or 45000),
                "history_limit": max(5, _coerce_int(queue_cfg.get("history_limit")) or 30),
                "target_dispatch_delay_ms": max(0, _coerce_int(queue_cfg.get("target_dispatch_delay_ms")) or 0),
                "release_on_option_info": bool(queue_cfg.get("release_on_option_info", False)),
                "suppress_active_target_options_on_new_detection": bool(
                    queue_cfg.get("suppress_active_target_options_on_new_detection", False)
                ),
                "copy_metrics": self._copy_metrics_snapshot_locked(),
            },
            "target_priority_order": self._target_priority_order_locked(),
        }

    def _deepcopy_locked(self, value: Any, *, label: str) -> Any:
        started_at = time.perf_counter()
        copied = copy.deepcopy(value)
        elapsed_ms = max(0.0, (time.perf_counter() - started_at) * 1000.0)
        metrics = self._copy_metrics.setdefault(
            str(label or "copy"),
            {"count": 0, "total_ms": 0.0, "max_ms": 0.0},
        )
        metrics["count"] = int(metrics.get("count", 0)) + 1
        metrics["total_ms"] = float(metrics.get("total_ms", 0.0)) + float(elapsed_ms)
        metrics["max_ms"] = max(float(metrics.get("max_ms", 0.0)), float(elapsed_ms))
        return copied

    def _copy_metrics_snapshot_locked(self) -> dict[str, dict[str, float | int]]:
        result: dict[str, dict[str, float | int]] = {}
        for label, metrics in self._copy_metrics.items():
            count = int(metrics.get("count", 0) or 0)
            total_ms = float(metrics.get("total_ms", 0.0) or 0.0)
            max_ms = float(metrics.get("max_ms", 0.0) or 0.0)
            result[str(label)] = {
                "count": count,
                "total_ms": round(total_ms, 3),
                "avg_ms": round(total_ms / count, 3) if count else 0.0,
                "max_ms": round(max_ms, 3),
            }
        return result

    def _settings_locked(self) -> dict[str, Any]:
        cfg = self._settings_getter() or {}
        if isinstance(cfg.get("replan_queue"), dict) or isinstance(cfg.get("target_detection"), dict):
            return dict(cfg)
        return {
            "replan_queue": dict(cfg) if isinstance(cfg, dict) else {},
            "target_detection": {},
        }

    def _queue_cfg_locked(self) -> dict[str, Any]:
        cfg = self._settings_locked().get("replan_queue")
        return dict(cfg) if isinstance(cfg, dict) else {}

    def _target_cfg_locked(self) -> dict[str, Any]:
        cfg = self._settings_locked().get("target_detection")
        return dict(cfg) if isinstance(cfg, dict) else {}

    def _target_priority_order_locked(self) -> list[int]:
        raw = self._target_cfg_locked().get("target_type_priority") or []
        out: list[int] = []
        seen: set[int] = set()
        for value in raw:
            parsed = _coerce_int(value)
            if parsed is None or parsed <= 0 or parsed in seen:
                continue
            seen.add(parsed)
            out.append(parsed)
        return out

    def _order_payloads_for_source(
        self,
        payloads: list[dict[str, Any]],
        *,
        source: str,
    ) -> list[dict[str, Any]]:
        normalized = [dict(item) for item in payloads if isinstance(item, dict)]
        if str(source or "").strip() != "target_detection":
            return normalized

        priority_order = self._target_priority_order_locked()
        priority_rank = {target_type: index for index, target_type in enumerate(priority_order)}
        fallback_rank = len(priority_rank) + 100
        decorated: list[tuple[int, int, int, dict[str, Any]]] = []
        for index, payload in enumerate(normalized):
            if _is_attack_close_payload(payload):
                decorated.append((-1, index, 0, payload))
                continue
            detail = _extract_detail(payload)
            target_type = _extract_target_type(detail)
            target_id = _coerce_int(detail.get("targetID") or detail.get("targetId")) or 0
            rank = priority_rank.get(int(target_type or 0), fallback_rank)
            decorated.append((rank, index, target_id, payload))
        decorated.sort(key=lambda item: (item[0], item[1], item[2]))
        return [payload for _rank, _index, _target_id, payload in decorated]


__all__ = ["ReplanQueueManager", "TARGET_TYPE_LABELS", "SOURCE_LABELS", "STAGE_LABELS"]
