"""Plan and replan metric classification helpers."""

from __future__ import annotations

import json
from collections.abc import Callable
from typing import Any


ReasonPredicate = Callable[[object | None], bool]


def count_replan_options(ctx: object | None, payload: object | None = None) -> int:
    for source in (payload, ctx):
        if not isinstance(source, dict):
            continue
        for key in (
            "optionList",
            "pendingOptionList",
            "options",
            "optionCodes",
            "missionPlanOptions",
            "candidateMissionPlans",
        ):
            value = source.get(key)
            if isinstance(value, list) and value:
                return len(value)
        for key in ("missionPlanIDList", "missionPlanIDs", "plan_ids", "option_names"):
            value = source.get(key)
            if isinstance(value, list) and value:
                return len(value)
    return 0


def classify_replan_timing_context(
    ctx: object | None,
    payload: object | None = None,
    *,
    is_path_deviation_reason_text: ReasonPredicate | None = None,
    is_quality_speed_reason_text: ReasonPredicate | None = None,
    is_imaging_schedule_reason_text: ReasonPredicate | None = None,
) -> dict[str, Any]:
    path_deviation_predicate = is_path_deviation_reason_text or (lambda _value: False)
    quality_speed_predicate = is_quality_speed_reason_text or (lambda _value: False)
    imaging_schedule_predicate = is_imaging_schedule_reason_text or (lambda _value: False)

    context = ctx if isinstance(ctx, dict) else {}
    data = payload if isinstance(payload, dict) else {}
    detail = context.get("replan_detail")
    if not isinstance(detail, dict):
        detail = data.get("replanDetail")
    if not isinstance(detail, dict):
        detail = {}

    reason = str(
        context.get("reason")
        or data.get("replanRequest")
        or data.get("replanReason")
        or ""
    ).strip()
    reason_l = reason.lower()
    try:
        level = int(context.get("replan_level", context.get("replanLevel", data.get("replanLevel", 0))) or 0)
    except Exception:
        level = 0
    detail_trigger = str(detail.get("triggerType") or "").strip()
    detail_trigger_l = detail_trigger.lower()
    detail_event = str(detail.get("trigger") or "").strip()
    detail_event_l = detail_event.lower()
    option_count = count_replan_options(context, data)
    force_direct = bool(context.get("force_direct_update"))

    detail_text = ""
    try:
        detail_text = json.dumps(detail, ensure_ascii=False, sort_keys=True).lower()
    except Exception:
        detail_text = str(detail or "").lower()

    if detail_trigger == "nextCollaborativeMission" or "nextcollaborative" in detail_trigger_l or "collab" in reason_l:
        trigger = "next_collab_direct"
    elif detail_trigger == "priorClosedResume":
        trigger = "prior_post_rejoin_direct"
    elif detail_trigger == "attackClosedDestroyed":
        trigger = "post_attack_direct"
    elif level == 5 or "dl" in reason_l or "risk" in reason_l or "risk" in detail_text:
        trigger = "dl_risk_level5"
    elif detail_trigger == "pathDeviation" or path_deviation_predicate(reason):
        trigger = "path_deviation_direct"
    elif detail_trigger == "qualityMonitorSep" or quality_speed_predicate(reason):
        trigger = "quality_speed_direct"
    elif detail_trigger == "imagingScheduleDeviation" or imaging_schedule_predicate(reason):
        trigger = "imaging_schedule_direct"
    elif detail_event == "0402" or "attack" in reason_l or "attacktarget" in detail_text or "0402" in detail_event_l:
        trigger = "attack_2_option" if option_count <= 2 else "attack_option"
    elif level == 4 or "prior" in reason_l:
        trigger = "prior_level4_direct"
    elif option_count >= 3:
        trigger = "general_3_option"
    elif force_direct:
        trigger = "direct_unknown"
    else:
        trigger = "general_or_unknown"

    return {
        "trigger": trigger,
        "replanLevel": int(level),
        "optionCount": int(option_count),
        "forceDirect": int(force_direct),
    }
