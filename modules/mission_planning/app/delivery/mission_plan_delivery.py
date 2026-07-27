"""Pure helpers for mission-plan post-delivery scheduling."""

from __future__ import annotations

import os
from typing import Any, Mapping


def env_int_clamped(
    name: str,
    default: int,
    minimum: int,
    maximum: int,
    environ: Mapping[str, str] | None = None,
) -> int:
    env = os.environ if environ is None else environ
    try:
        value = int(env.get(name, str(default)))
    except Exception:
        value = int(default)
    return max(int(minimum), min(int(maximum), int(value)))


def post_0301_delivery_delays(
    *,
    plan_count: int,
    force_direct: bool,
    environ: Mapping[str, str] | None = None,
) -> tuple[int, int, str]:
    count = max(int(plan_count or 0), 1)
    if force_direct:
        grace_ms = env_int_clamped(
            "REPLAN_FORCE_DIRECT_POST_0301_GRACE_MS",
            0,
            0,
            1800,
            environ,
        )
        timeout_ms = env_int_clamped(
            "REPLAN_FORCE_DIRECT_POST_0301_TIMEOUT_MS",
            1200,
            grace_ms + 100,
            5000,
            environ,
        )
        return int(grace_ms), int(timeout_ms), "direct_0903"

    grace_base_ms = env_int_clamped("REPLAN_OPTION_POST_0301_GRACE_MS", 0, 0, 1800, environ)
    grace_extra_ms = env_int_clamped("REPLAN_OPTION_POST_0301_PER_EXTRA_PLAN_MS", 0, 0, 500, environ)
    timeout_extra_ms = env_int_clamped("REPLAN_OPTION_POST_0301_TIMEOUT_EXTRA_MS", 20, 0, 3000, environ)
    grace_ms = int(grace_base_ms) + max(0, count - 1) * int(grace_extra_ms)
    fallback_ms = int(grace_ms) + int(timeout_extra_ms)
    return int(grace_ms), int(fallback_ms), "option_or_0901"


def sort_plan_delivery_entries(
    plan_ids: object | None,
    option_names: object | None,
) -> tuple[list[int], list[str]]:
    names = list(option_names or [])
    entries: list[tuple[int, str, int]] = []
    seen: set[int] = set()
    for idx, raw_plan_id in enumerate(plan_ids or []):
        try:
            plan_id = int(raw_plan_id)
        except Exception:
            continue
        if plan_id <= 0 or plan_id in seen:
            continue
        seen.add(plan_id)
        raw_name = names[idx] if idx < len(names) else None
        option_name = str(raw_name) if raw_name is not None else f"option{len(entries) + 1}"
        entries.append((plan_id, option_name, idx))
    entries.sort(key=lambda item: (item[0], item[2]))
    return [item[0] for item in entries], [item[1] for item in entries]


def normalize_post_delivery_waypoint_mark(payload: Any) -> dict[str, Any] | None:
    if not isinstance(payload, dict):
        return None
    try:
        max_waypoint_id = int(payload.get("max_waypoint_id") or 0)
    except Exception:
        max_waypoint_id = 0
    try:
        variants = int(payload.get("variants") or 0)
    except Exception:
        variants = 0
    if max_waypoint_id <= 0 and variants <= 0:
        return None
    return {
        "max_waypoint_id": int(max_waypoint_id),
        "variants": max(0, int(variants)),
        "reason": str(payload.get("reason") or "post_delivery_waypoint_mark"),
    }


def merge_post_delivery_waypoint_mark(existing: Any, incoming: Any) -> dict[str, Any] | None:
    current = normalize_post_delivery_waypoint_mark(existing) or {}
    new_value = normalize_post_delivery_waypoint_mark(incoming) or {}
    if not current and not new_value:
        return None
    return {
        "max_waypoint_id": max(
            int(current.get("max_waypoint_id") or 0),
            int(new_value.get("max_waypoint_id") or 0),
        ),
        "variants": int(current.get("variants") or 0) + int(new_value.get("variants") or 0),
        "reason": str(new_value.get("reason") or current.get("reason") or "post_delivery_waypoint_mark"),
    }


def normalize_post_delivery_snapshot_carry_forward(payload: Any) -> dict[str, Any] | None:
    if not isinstance(payload, dict):
        return None
    raw_items = payload.get("items")
    if raw_items is None:
        raw_items = [payload]
    if isinstance(raw_items, (str, bytes)) or not isinstance(raw_items, (list, tuple)):
        return None
    items: list[dict[str, Any]] = []
    for raw in raw_items:
        if not isinstance(raw, dict):
            continue
        try:
            source_plan_id = int(raw.get("sourceMissionPlanID") or raw.get("source_plan_id") or 0)
            target_plan_id = int(raw.get("targetMissionPlanID") or raw.get("target_plan_id") or 0)
        except Exception:
            continue
        if source_plan_id <= 0 or target_plan_id <= 0:
            continue
        try:
            variant_no = int(raw.get("variant") or raw.get("variant_no") or 0)
        except Exception:
            variant_no = 0
        items.append(
            {
                "sourceMissionPlanID": int(source_plan_id),
                "targetMissionPlanID": int(target_plan_id),
                "variant": max(0, int(variant_no)),
                "reason": str(raw.get("reason") or payload.get("reason") or "post_delivery_snapshot_carry_forward"),
            }
        )
    if not items:
        return None
    return {
        "items": items,
        "reason": str(payload.get("reason") or "post_delivery_snapshot_carry_forward"),
    }


def merge_post_delivery_snapshot_carry_forward(existing: Any, incoming: Any) -> dict[str, Any] | None:
    current = normalize_post_delivery_snapshot_carry_forward(existing) or {}
    new_value = normalize_post_delivery_snapshot_carry_forward(incoming) or {}
    raw_items = list(current.get("items") or []) + list(new_value.get("items") or [])
    if not raw_items:
        return None
    merged: list[dict[str, Any]] = []
    seen: set[tuple[int, int, str]] = set()
    for item in raw_items:
        try:
            source_plan_id = int(item.get("sourceMissionPlanID") or 0)
            target_plan_id = int(item.get("targetMissionPlanID") or 0)
        except Exception:
            continue
        reason = str(item.get("reason") or "").strip()
        key = (source_plan_id, target_plan_id, reason)
        if source_plan_id <= 0 or target_plan_id <= 0 or key in seen:
            continue
        seen.add(key)
        merged.append(dict(item))
    if not merged:
        return None
    return {
        "items": merged,
        "reason": str(new_value.get("reason") or current.get("reason") or "post_delivery_snapshot_carry_forward"),
    }
