"""Persistent per-scenario replan timing history.

The history stores three request-local monotonic intervals: 0902 callback
entry to the successfully sent running 0305, running 0305 to the successfully
sent completion 0305, and their end-to-end total.  Wall-clock timestamps are
kept only for correlation.  A stable timing ID makes retries idempotent without
discarding two distinct requests that legitimately took the same duration.
"""

from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from modules.common import db_paths


HISTORY_FILENAME = "replan_timing_history.json"
_LOCK = threading.RLock()


def get_replan_timing_history_path() -> Path:
    """Return the timing history path for the active scenario database."""

    return db_paths.get_db_subpath("DSS_Internal", HISTORY_FILENAME)


def _utc_iso(epoch_ms: int | None = None) -> str:
    if epoch_ms is None:
        value = datetime.now(timezone.utc)
    else:
        value = datetime.fromtimestamp(max(0, int(epoch_ms)) / 1000.0, tz=timezone.utc)
    return value.isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _nonnegative_float(value: Any, *, required: bool) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        if required:
            raise ValueError("elapsed time must be a finite non-negative number")
        return None
    if number < 0.0 or number != number or number in (float("inf"), float("-inf")):
        if required:
            raise ValueError("elapsed time must be a finite non-negative number")
        return None
    return round(number, 3)


def _int_list(values: Sequence[Any] | None) -> list[int]:
    result: list[int] = []
    for value in values or ():
        try:
            normalized = int(value)
        except (TypeError, ValueError):
            continue
        if normalized not in result:
            result.append(normalized)
    return result


def _plain_metadata(metadata: Mapping[str, Any] | None) -> dict[str, Any]:
    """Keep metadata JSON-safe without letting arbitrary runtime state leak in."""

    if not isinstance(metadata, Mapping):
        return {}
    result: dict[str, Any] = {}
    for key, value in metadata.items():
        name = str(key)
        if value is None or isinstance(value, (str, int, float, bool)):
            result[name] = value
        elif isinstance(value, (list, tuple)):
            result[name] = [item for item in value if item is None or isinstance(item, (str, int, float, bool))]
    return result


def _load_history(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return loaded if isinstance(loaded, dict) else {}


def _metric_summary(records: list[dict[str, Any]], field: str) -> dict[str, Any]:
    values: list[float] = []
    for record in records:
        value = _nonnegative_float(record.get(field), required=False)
        if value is not None:
            values.append(value)
    if not values:
        return {
            "sampleCount": 0,
            "totalMs": 0.0,
            "averageMs": 0.0,
            "minMs": None,
            "maxMs": None,
            "lastMs": None,
        }
    total = sum(values)
    return {
        "sampleCount": len(values),
        "totalMs": round(total, 3),
        "averageMs": round(total / len(values), 3),
        "minMs": round(min(values), 3),
        "maxMs": round(max(values), 3),
        "lastMs": round(values[-1], 3),
    }


def _build_summary(records: list[dict[str, Any]]) -> dict[str, Any]:
    elapsed = _metric_summary(records, "elapsedMs")
    planning = _metric_summary(records, "planningElapsedMs")
    success_count = sum(1 for record in records if record.get("status") == "success")
    failure_count = sum(1 for record in records if record.get("status") == "failed")
    return {
        "completedCount": len(records),
        "successCount": success_count,
        "failureCount": failure_count,
        "totalElapsedMs": elapsed["totalMs"],
        "averageElapsedMs": elapsed["averageMs"],
        "minElapsedMs": elapsed["minMs"],
        "maxElapsedMs": elapsed["maxMs"],
        "lastElapsedMs": elapsed["lastMs"],
        "planningSampleCount": planning["sampleCount"],
        "totalPlanningElapsedMs": planning["totalMs"],
        "averagePlanningElapsedMs": planning["averageMs"],
        "minPlanningElapsedMs": planning["minMs"],
        "maxPlanningElapsedMs": planning["maxMs"],
        "lastPlanningElapsedMs": planning["lastMs"],
    }


def _build_by_trigger(records: list[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        trigger = str(record.get("trigger") or "unknown")
        grouped.setdefault(trigger, []).append(record)
    return {trigger: _build_summary(group) for trigger, group in sorted(grouped.items())}


def _write_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = path.with_name(f".{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
    temp_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temp_path.replace(path)


def record_replan_timing(
    *,
    elapsed_ms: Any,
    planning_elapsed_ms: Any = None,
    queue_elapsed_ms: Any = None,
    status: str = "success",
    reason: str = "",
    trigger: str = "",
    trigger_type: str = "",
    replan_level: Any = None,
    option_count: Any = None,
    timing_id: Any = None,
    transaction_id: Any = None,
    source_plan_ids: Sequence[Any] | None = None,
    result_plan_ids: Sequence[Any] | None = None,
    started_at_ms: Any = None,
    running_at_ms: Any = None,
    completed_at_ms: Any = None,
    metadata: Mapping[str, Any] | None = None,
    path: str | Path | None = None,
) -> dict[str, Any]:
    """Append one completed replan and return its updated cumulative summary."""

    elapsed = _nonnegative_float(elapsed_ms, required=True)
    planner_elapsed = _nonnegative_float(planning_elapsed_ms, required=False)
    queue_elapsed = _nonnegative_float(queue_elapsed_ms, required=False)
    normalized_status = "failed" if str(status).strip().lower() in {"failed", "failure", "error"} else "success"
    try:
        completed_wall_ms = int(completed_at_ms)
    except (TypeError, ValueError):
        completed_wall_ms = int(datetime.now(timezone.utc).timestamp() * 1000)
    try:
        started_wall_ms = int(started_at_ms)
    except (TypeError, ValueError):
        started_wall_ms = completed_wall_ms - int(round(float(elapsed or 0.0)))
    try:
        running_wall_ms = int(running_at_ms)
    except (TypeError, ValueError):
        running_wall_ms = None

    normalized_timing_id = str(timing_id or "").strip()

    destination = Path(path) if path is not None else get_replan_timing_history_path()
    with _LOCK:
        history = _load_history(destination)
        records = history.get("records")
        if not isinstance(records, list):
            records = []
        records = [record for record in records if isinstance(record, dict)]
        if normalized_timing_id:
            existing = next(
                (
                    record
                    for record in records
                    if str(record.get("replanTimingId") or "").strip() == normalized_timing_id
                ),
                None,
            )
            if existing is not None:
                summary = _build_summary(records)
                return {
                    "path": str(destination),
                    "record": existing,
                    "summary": summary,
                    "byTrigger": _build_by_trigger(records),
                    "duplicate": True,
                }
        try:
            last_sequence = max(int(record.get("sequence") or 0) for record in records)
        except ValueError:
            last_sequence = 0

        record: dict[str, Any] = {
            "sequence": last_sequence + 1,
            "startedAt": _utc_iso(started_wall_ms),
            "startedAtEpochMs": started_wall_ms,
            "completedAt": _utc_iso(completed_wall_ms),
            "completedAtEpochMs": completed_wall_ms,
            "status": normalized_status,
            "reason": str(reason or ""),
            "trigger": str(trigger or "unknown"),
            "triggerType": str(trigger_type or ""),
            "elapsedMs": elapsed,
            "elapsedSeconds": round(float(elapsed or 0.0) / 1000.0, 6),
            "queueElapsedMs": queue_elapsed,
            "queueElapsedSeconds": (
                round(float(queue_elapsed) / 1000.0, 6) if queue_elapsed is not None else None
            ),
            "planningElapsedMs": planner_elapsed,
            "planningElapsedSeconds": (
                round(float(planner_elapsed) / 1000.0, 6) if planner_elapsed is not None else None
            ),
            "sourceMissionPlanIDs": _int_list(source_plan_ids),
            "resultMissionPlanIDs": _int_list(result_plan_ids),
        }
        if normalized_timing_id:
            record["replanTimingId"] = normalized_timing_id
        if running_wall_ms is not None:
            record["runningSentAt"] = _utc_iso(running_wall_ms)
            record["runningSentAtEpochMs"] = running_wall_ms
        try:
            record["replanLevel"] = int(replan_level) if replan_level is not None else None
        except (TypeError, ValueError):
            record["replanLevel"] = None
        try:
            record["optionCount"] = int(option_count) if option_count is not None else None
        except (TypeError, ValueError):
            record["optionCount"] = None
        if transaction_id not in (None, ""):
            record["replanTransactionId"] = str(transaction_id)
        clean_metadata = _plain_metadata(metadata)
        if clean_metadata:
            record["metadata"] = clean_metadata

        records.append(record)
        summary = _build_summary(records)
        payload = {
            "schemaVersion": 2,
            "metricDefinition": "0902_received_to_0305_status_2_sent",
            "timeUnit": "milliseconds",
            "updatedAt": _utc_iso(completed_wall_ms),
            "summary": summary,
            "byTrigger": _build_by_trigger(records),
            "records": records,
        }
        _write_atomic(destination, payload)
        return {
            "path": str(destination),
            "record": record,
            "summary": summary,
            "byTrigger": payload["byTrigger"],
        }


__all__ = [
    "HISTORY_FILENAME",
    "get_replan_timing_history_path",
    "record_replan_timing",
]
