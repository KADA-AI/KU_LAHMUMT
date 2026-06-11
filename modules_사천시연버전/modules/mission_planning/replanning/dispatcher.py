"""Replan dispatch route predicates.

The GUI still owns pipeline execution and handled/fallback semantics. This
module centralizes route classification so trigger-specific code can move under
`replanning/triggers/*` without changing the public launcher first.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any


POST_ATTACK_REJOIN_ROUTE = "post_attack_rejoin"
PRIOR_POST_REJOIN_ROUTE = "prior_post_rejoin"
ATTACK_ROUTE = "attack"

SPECIALIZED_DISPATCH_ORDER = (
    POST_ATTACK_REJOIN_ROUTE,
    "next_collab",
    "imaging_schedule",
    "path_deviation",
    "prior",
)

_ATTACK_REASON_KEYWORDS = (
    "\uacf5\uaca9 \ud2b9\ud654",
    "\uacf5\uaca9\ud2b9\ud654",
    "\uacf5\uaca9\ucd94\ucc9c",
)
_ATTACK_DETAIL_KEYWORDS = (
    "\uacf5\uaca9 \ud2b9\ud654",
    "\uacf5\uaca9\ud2b9\ud654",
)


def _detail_value(detail: Any, key: str) -> str:
    if not isinstance(detail, Mapping):
        return ""
    return str(detail.get(key) or "").strip()


def is_post_attack_rejoin_detail(detail: Any) -> bool:
    if not isinstance(detail, Mapping):
        return False
    if _detail_value(detail, "trigger") != "0402":
        return False
    return _detail_value(detail, "triggerType") == "attackClosedDestroyed"


def is_prior_post_rejoin_detail(detail: Any) -> bool:
    if not isinstance(detail, Mapping):
        return False
    if _detail_value(detail, "trigger") != "0401":
        return False
    return _detail_value(detail, "triggerType") == "priorClosedResume"


def _has_any_keyword(value: object | None, keywords: tuple[str, ...]) -> bool:
    text = str(value or "")
    return any(keyword in text for keyword in keywords)


def should_use_attack_pipeline(ctx: Mapping[str, Any]) -> bool:
    reason_text = str(ctx.get("reason") or "").strip()
    if _has_any_keyword(reason_text, _ATTACK_REASON_KEYWORDS):
        return True

    detail = ctx.get("replan_detail")
    if isinstance(detail, Mapping):
        if is_post_attack_rejoin_detail(detail):
            return False
        trigger = detail.get("trigger")
        if isinstance(trigger, str) and trigger.strip() == "0402":
            return True
        for key in ("ReplanReason", "reason", "label", "mode"):
            if _has_any_keyword(detail.get(key), _ATTACK_DETAIL_KEYWORDS):
                return True

    option_names = ctx.get("option_names") or []
    if isinstance(option_names, Sequence) and not isinstance(option_names, (str, bytes)):
        for name in option_names:
            if _has_any_keyword(name, _ATTACK_REASON_KEYWORDS):
                return True

    return False


def should_use_post_attack_rejoin_pipeline(ctx: Mapping[str, Any]) -> bool:
    return is_post_attack_rejoin_detail(ctx.get("replan_detail"))


def should_use_prior_post_rejoin_pipeline(ctx: Mapping[str, Any]) -> bool:
    return is_prior_post_rejoin_detail(ctx.get("replan_detail"))
