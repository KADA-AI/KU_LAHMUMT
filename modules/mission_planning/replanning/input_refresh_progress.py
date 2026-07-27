from __future__ import annotations

from typing import Any, Iterable


def _positive_int(value: object | None) -> int | None:
    try:
        parsed = int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


def parallel_snapshot_safety_reasons(
    *,
    snapshot_apply_result: dict[str, Any] | None,
    collapse_apply_result: dict[str, Any] | None,
    filtered_payload_materialized: bool,
) -> list[str]:
    """Return parallel-planning blockers for progress-mutated 0201 input.

    Snapshot/collapse mutations are safe for parallel variants once the
    resulting 0201 has been written to the common filtered input artifact.
    Every worker then reads the same immutable payload.  If that write failed,
    keep the legacy sequential fallback because only the in-memory payload has
    the mutations.
    """

    if bool(filtered_payload_materialized):
        return []

    snapshot = snapshot_apply_result if isinstance(snapshot_apply_result, dict) else {}
    collapse = collapse_apply_result if isinstance(collapse_apply_result, dict) else {}
    reasons: list[str] = []
    try:
        if int(snapshot.get("applied") or 0) > 0 or int(snapshot.get("marked_done") or 0) > 0:
            reasons.append("remaining_snapshot_mutated")
    except (TypeError, ValueError):
        pass
    try:
        if int(collapse.get("groupCount") or 0) > 0:
            reasons.append("mission_collapse_mutated")
    except (TypeError, ValueError):
        pass
    return reasons


def _detail_rows(*containers: dict[str, Any]) -> Iterable[dict[str, Any]]:
    for container in containers:
        if not isinstance(container, dict):
            continue
        detail = container.get("replan_detail")
        if isinstance(detail, dict):
            yield detail


def is_input_refresh_context(*containers: dict[str, Any]) -> bool:
    return any(
        str(detail.get("trigger") or "").strip() == "0201"
        and str(detail.get("triggerType") or "").strip() == "inputRefresh"
        for detail in _detail_rows(*containers)
    )


def input_refresh_current_input_id(*containers: dict[str, Any]) -> int | None:
    for detail in _detail_rows(*containers):
        if (
            str(detail.get("trigger") or "").strip() != "0201"
            or str(detail.get("triggerType") or "").strip() != "inputRefresh"
        ):
            continue
        current_input_id = _positive_int(detail.get("currentInputMissionID"))
        if current_input_id is not None:
            return int(current_input_id)
    return None


def attach_input_refresh_current_input_id(
    current_input_id: int,
    *containers: dict[str, Any],
) -> int:
    resolved = _positive_int(current_input_id)
    if resolved is None:
        return 0
    updated = 0
    for detail in _detail_rows(*containers):
        if (
            str(detail.get("trigger") or "").strip() != "0201"
            or str(detail.get("triggerType") or "").strip() != "inputRefresh"
        ):
            continue
        detail["currentInputMissionID"] = int(resolved)
        detail["preserveCurrentMissionProgress"] = True
        updated += 1
    return int(updated)


def infer_started_input_mission_id(
    snapshot: dict[str, Any] | None,
    payload: dict[str, Any] | None,
) -> int | None:
    """Infer the active mission only when one exact, started mission exists.

    A zero-progress pending mission is intentionally not guessed.  Applying a
    snapshot to the wrong future mission is worse than skipping the fallback.
    """

    if not isinstance(snapshot, dict) or not isinstance(payload, dict):
        return None
    payload_ids = {
        input_id
        for mission in (payload.get("inputMissionList") or [])
        if isinstance(mission, dict)
        for input_id in [_positive_int(mission.get("inputMissionID"))]
        if input_id is not None
    }
    if not payload_ids:
        return None

    candidates: set[int] = set()
    for entry in snapshot.get("missions") or []:
        if not isinstance(entry, dict) or bool(entry.get("isDone")):
            continue
        input_id = _positive_int(entry.get("inputMissionID"))
        if input_id is None or int(input_id) not in payload_ids:
            continue
        mission_type = str(entry.get("missionType") or "").strip().lower()
        if mission_type not in {"line", "area"}:
            continue
        try:
            coverage_percent = float(entry.get("coveragePercent") or 0.0)
        except (TypeError, ValueError):
            coverage_percent = 0.0
        if coverage_percent > 0.0:
            candidates.add(int(input_id))
    if len(candidates) != 1:
        return None
    return next(iter(candidates))


def input_refresh_snapshot_whitelist(
    *,
    ctx: dict[str, Any],
    staged: dict[str, Any],
    mission_whitelist: set[int] | None = None,
) -> set[int] | None:
    """Return a current-only scope for inputRefresh, or None for other triggers."""

    if not is_input_refresh_context(ctx, staged):
        return None
    current_input_id = input_refresh_current_input_id(ctx, staged)
    if current_input_id is None:
        return {-1}
    allowed = {int(value) for value in (mission_whitelist or set())}
    if allowed and int(current_input_id) not in allowed:
        return {-1}
    return {int(current_input_id)}
