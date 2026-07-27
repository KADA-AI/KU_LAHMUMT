"""Helpers for 0902 replan request parsing and normalization."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from typing import Any, Mapping


_JSON_OBJECT_RE = re.compile(r"{.*}", flags=re.S)


@dataclass(frozen=True, slots=True)
class ReplanRequestSelection:
    plan_ids: list[int] = field(default_factory=list)
    option_names: list[str] = field(default_factory=list)
    mission_ids: list[int] = field(default_factory=list)
    detail: Any = None
    detail_trigger_type: str = ""


@dataclass(frozen=True, slots=True)
class ReplanDelayPolicy:
    runtime_setting_key: str | None
    default_delay_ms: int


def parse_replan_payload(raw: bytes | bytearray | str | Mapping[str, Any] | None) -> dict[str, Any] | None:
    if not raw:
        return None
    if isinstance(raw, Mapping):
        return dict(raw)
    if isinstance(raw, str):
        text = raw
    else:
        try:
            text = bytes(raw).decode("utf-8", "ignore")
        except Exception:
            return None
    try:
        match = _JSON_OBJECT_RE.search(text)
        if not match:
            return None
        payload = json.loads(match.group(0))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def _append_int(target: list[int], value: Any) -> bool:
    try:
        target.append(int(value))
        return True
    except Exception:
        return False


def extract_replan_request_selection(payload: Mapping[str, Any]) -> ReplanRequestSelection:
    detail = payload.get("replanDetail") if isinstance(payload, Mapping) else None
    detail_trigger_type = ""
    if isinstance(detail, Mapping):
        detail_trigger_type = str(detail.get("triggerType") or "").strip()

    plan_ids: list[int] = []
    option_names: list[str] = []
    option_payload = payload.get("optionList") or payload.get("pendingOptionList") or []
    if isinstance(option_payload, list):
        for item in option_payload:
            if not isinstance(item, Mapping):
                continue
            if not _append_int(plan_ids, item.get("missionPlanID")):
                continue
            name = item.get("optionName")
            if name:
                option_names.append(str(name))

    if not plan_ids:
        mission_plan_ids = payload.get("missionPlanIDList") or []
        if isinstance(mission_plan_ids, list):
            for item in mission_plan_ids:
                value = item.get("missionPlanID") if isinstance(item, Mapping) else item
                _append_int(plan_ids, value)

    if not plan_ids and isinstance(detail, Mapping):
        try:
            detail_plan_id = int(detail.get("missionPlanID"))
        except Exception:
            detail_plan_id = None
        if detail_plan_id is not None and detail_plan_id > 0:
            plan_ids.append(int(detail_plan_id))

    mission_ids: list[int] = []
    input_mission_ids = payload.get("inputMissionIDList") or []
    if isinstance(input_mission_ids, list):
        for item in input_mission_ids:
            try:
                mission_ids.append(int(item.get("inputMissionID")))
            except Exception:
                continue

    return ReplanRequestSelection(
        plan_ids=plan_ids,
        option_names=option_names,
        mission_ids=mission_ids,
        detail=detail,
        detail_trigger_type=detail_trigger_type,
    )


def replan_delay_policy(payload: Mapping[str, Any]) -> ReplanDelayPolicy:
    detail = payload.get("replanDetail") if isinstance(payload, Mapping) else None
    if not isinstance(detail, Mapping):
        return ReplanDelayPolicy(None, 100)
    trigger = str(detail.get("trigger") or "").strip()
    trigger_type = str(detail.get("triggerType") or "").strip()
    if trigger_type == "collabReexecuteInputRefresh":
        return ReplanDelayPolicy("replan_collab_reexecute_schedule_delay_ms", 30)
    if trigger_type == "nextCollaborativeMission":
        # Monitoring persists the complete next-collaborative detail before it
        # sends 0902.  Keep a 1 ms Qt timer boundary so rapid requests retain
        # the existing coalescing/replay ordering without the generic 100 ms
        # anti-race delay.
        return ReplanDelayPolicy("replan_next_collab_schedule_delay_ms", 1)
    if trigger == "0402" or trigger_type == "attackClosedDestroyed":
        return ReplanDelayPolicy(None, 0)
    if trigger != "0401":
        return ReplanDelayPolicy(None, 100)
    if trigger_type in {"communicationLossRTB", "abnormalHealthRTB", "unexpectedRTB"}:
        return ReplanDelayPolicy(None, 55_000)
    return ReplanDelayPolicy(None, 100)
