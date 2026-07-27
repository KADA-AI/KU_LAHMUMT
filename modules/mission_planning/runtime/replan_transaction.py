from __future__ import annotations

import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable

from modules.mission_planning.runtime.json_io import serialize_json_payload, write_json_bytes

try:
    from modules.common import replan_perf
except Exception:  # pragma: no cover - fallback for direct script execution
    replan_perf = None  # type: ignore


@dataclass(frozen=True)
class StagedJsonWrite:
    path: Path
    payload: bytes
    label: str = ""

    @property
    def payload_bytes(self) -> int:
        return len(self.payload)

    def summary(self) -> dict[str, Any]:
        return {
            "path": str(self.path),
            "name": self.path.name,
            "label": self.label,
            "payloadBytes": int(self.payload_bytes),
        }


def _workers_for_entry_count(entry_count: int, requested_workers: int | None = None) -> int:
    if entry_count <= 1:
        return 1
    if requested_workers is not None:
        try:
            value = int(requested_workers)
        except Exception:
            value = 1
    else:
        raw = os.getenv("DSS_REPLAN_TRANSACTION_WORKERS", "").strip()
        try:
            value = int(float(raw)) if raw else 1
        except Exception:
            value = 1
    return max(1, min(value, int(entry_count)))


def _json_write_workers_for_entry_count(entry_count: int) -> int:
    raw = os.getenv("DSS_REPLAN_JSON_WRITE_WORKERS", "").strip()
    if not raw:
        return 1
    try:
        value = int(float(raw))
    except Exception:
        return 1
    if value <= 1 or entry_count <= 1:
        return 1
    return max(1, min(value, int(entry_count)))


def _path_identity_key(path: Path) -> str:
    try:
        return os.path.normcase(os.path.abspath(os.fspath(path)))
    except Exception:
        return str(path)


def _has_duplicate_paths(entries: Iterable[StagedJsonWrite]) -> bool:
    seen: set[str] = set()
    for entry in entries:
        key = _path_identity_key(entry.path)
        if key in seen:
            return True
        seen.add(key)
    return False


def _perf_add_elapsed(name: str, started: float, **extra: Any) -> None:
    if replan_perf is None:
        return
    try:
        replan_perf.add_elapsed(name, started, **extra)
    except Exception:
        return


def _mark_written_flight_paths(
    entries: list[tuple[Path, Any]],
    results: list[dict[str, Any]],
) -> None:
    """Advance the waypoint high-water signature after known FlightPath writes.

    The allocator otherwise has to reopen every FlightPath JSON when the
    directory signature changes.  All waypoint IDs in these payloads are
    already available here, so recording their maximum is equivalent and
    avoids that full-directory rescan on the next reservation.
    """

    max_waypoint_id: int | None = None
    wrote_flight_path = False
    for (path, payload), result in zip(entries, results):
        if not bool(result.get("written")) or path.parent.name.casefold() != "flightpath":
            continue
        wrote_flight_path = True
        if not isinstance(payload, dict):
            continue
        for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
            waypoints = payload.get(key)
            if not isinstance(waypoints, list):
                continue
            for waypoint in waypoints:
                if not isinstance(waypoint, dict):
                    continue
                try:
                    waypoint_id = int(waypoint.get("waypointID"))
                except (TypeError, ValueError):
                    continue
                if waypoint_id <= 0:
                    continue
                if max_waypoint_id is None or waypoint_id > max_waypoint_id:
                    max_waypoint_id = waypoint_id
    if not wrote_flight_path:
        return
    try:
        from modules.mission_planning.engine.mission_generation.id_allocation.allocator import (
            mark_waypoint_files_written,
        )

        mark_waypoint_files_written(max_waypoint_id=max_waypoint_id)
    except Exception:
        # This is a performance hint only.  The allocator keeps its existing
        # full-scan fallback if the signature cannot be recorded.
        return


class ReplanTransaction:
    """Stage JSON artifact writes and commit them as one explicit step.

    The class is intentionally small: it does not change ID allocation or
    validation behavior. It only separates serialization/staging from file
    mutation so trigger adapters can move expensive or risky writes to a final
    commit boundary.
    """

    def __init__(
        self,
        transaction_id: str,
        *,
        skip_if_unchanged: bool = True,
        log: Callable[[str], None] | None = None,
    ) -> None:
        self.transaction_id = str(transaction_id or "replan-transaction")
        self.skip_if_unchanged = bool(skip_if_unchanged)
        self.log = log
        self._entries: list[StagedJsonWrite] = []

    @property
    def entries(self) -> tuple[StagedJsonWrite, ...]:
        return tuple(self._entries)

    def stage_json(
        self,
        path: str | Path,
        data: Any,
        *,
        pretty: bool = True,
        ensure_ascii: bool = False,
        sort_keys: bool = False,
        label: str = "",
    ) -> StagedJsonWrite:
        started = time.perf_counter()
        target = Path(path)
        payload = serialize_json_payload(
            target,
            data,
            pretty=pretty,
            ensure_ascii=ensure_ascii,
            sort_keys=sort_keys,
        )
        entry = StagedJsonWrite(target, payload, str(label or ""))
        self._entries.append(entry)
        _perf_add_elapsed(
            "mission_planning.replan_transaction.stage_json",
            started,
            payload_bytes=len(payload),
            entries=len(self._entries),
        )
        return entry

    def stage_json_batch(
        self,
        entries: Iterable[tuple[str | Path, Any]],
        *,
        pretty: bool = True,
        ensure_ascii: bool = False,
        sort_keys: bool = False,
        label: str | Callable[[Path, Any], str] = "",
        max_workers: int | None = None,
    ) -> list[StagedJsonWrite]:
        started = time.perf_counter()
        normalized_entries = [(Path(path), data) for path, data in entries]
        worker_count = _workers_for_entry_count(len(normalized_entries), max_workers)
        if worker_count > 1:
            seen: set[str] = set()
            for path, _data in normalized_entries:
                key = _path_identity_key(path)
                if key in seen:
                    worker_count = 1
                    break
                seen.add(key)

        def _label_for(path: Path, data: Any) -> str:
            if callable(label):
                try:
                    return str(label(path, data) or "")
                except Exception:
                    return ""
            return str(label or "")

        def _stage_one(item: tuple[int, Path, Any]) -> tuple[int, StagedJsonWrite]:
            index, target, data = item
            payload = serialize_json_payload(
                target,
                data,
                pretty=pretty,
                ensure_ascii=ensure_ascii,
                sort_keys=sort_keys,
            )
            return index, StagedJsonWrite(target, payload, _label_for(target, data))

        staged: list[StagedJsonWrite | None] = [None] * len(normalized_entries)
        indexed_entries = [
            (index, path, data)
            for index, (path, data) in enumerate(normalized_entries)
        ]
        if worker_count > 1:
            with ThreadPoolExecutor(max_workers=worker_count, thread_name_prefix="replan_tx_stage") as executor:
                futures = [executor.submit(_stage_one, item) for item in indexed_entries]
                for future in as_completed(futures):
                    index, entry = future.result()
                    staged[index] = entry
        else:
            for item in indexed_entries:
                index, entry = _stage_one(item)
                staged[index] = entry

        ordered = [entry for entry in staged if isinstance(entry, StagedJsonWrite)]
        self._entries.extend(ordered)
        _perf_add_elapsed(
            "mission_planning.replan_transaction.stage_json_batch",
            started,
            entries=len(ordered),
            payload_bytes=sum(entry.payload_bytes for entry in ordered),
            workers=int(worker_count),
        )
        return ordered

    def stage_json_bytes(
        self,
        path: str | Path,
        payload: bytes,
        *,
        label: str = "",
    ) -> StagedJsonWrite:
        if not isinstance(payload, (bytes, bytearray)):
            raise TypeError("payload must be bytes")
        entry = StagedJsonWrite(Path(path), bytes(payload), str(label or ""))
        self._entries.append(entry)
        return entry

    def clear(self) -> None:
        self._entries.clear()

    def summary(self) -> dict[str, Any]:
        entries = list(self._entries)
        return {
            "transactionId": self.transaction_id,
            "entryCount": len(entries),
            "payloadBytes": int(sum(entry.payload_bytes for entry in entries)),
            "duplicatePaths": bool(_has_duplicate_paths(entries)),
            "entries": [entry.summary() for entry in entries],
        }

    def dry_run(self) -> dict[str, Any]:
        entries = list(self._entries)
        existing = 0
        for entry in entries:
            try:
                if entry.path.exists():
                    existing += 1
            except Exception:
                pass
        return {
            "transactionId": self.transaction_id,
            "dryRun": True,
            "entryCount": len(entries),
            "existingPathCount": int(existing),
            "newPathCount": int(len(entries) - existing),
            "payloadBytes": int(sum(entry.payload_bytes for entry in entries)),
            "entries": [entry.summary() for entry in entries],
        }

    def commit(
        self,
        *,
        max_workers: int | None = None,
        dry_run: bool = False,
        skip_if_unchanged: bool | None = None,
    ) -> dict[str, Any]:
        if dry_run:
            return self.dry_run()
        started = time.perf_counter()
        entries = list(self._entries)
        worker_count = _workers_for_entry_count(len(entries), max_workers)
        if worker_count > 1 and _has_duplicate_paths(entries):
            worker_count = 1
        effective_skip = self.skip_if_unchanged if skip_if_unchanged is None else bool(skip_if_unchanged)

        results: list[dict[str, Any] | None] = [None] * len(entries)
        errors = 0
        first_error: BaseException | None = None

        def _write_one(index: int, entry: StagedJsonWrite) -> tuple[int, dict[str, Any]]:
            row = entry.summary()
            row["existedBefore"] = bool(entry.path.exists())
            written = write_json_bytes(
                entry.path,
                entry.payload,
                skip_if_unchanged=effective_skip,
            )
            row["written"] = bool(written)
            row["skipped"] = not bool(written)
            return index, row

        if worker_count > 1:
            with ThreadPoolExecutor(max_workers=worker_count, thread_name_prefix="replan_tx") as executor:
                futures = {
                    executor.submit(_write_one, index, entry): index
                    for index, entry in enumerate(entries)
                }
                for future in as_completed(futures):
                    index = futures[future]
                    try:
                        result_index, row = future.result()
                        results[result_index] = row
                    except Exception as exc:
                        errors += 1
                        if first_error is None:
                            first_error = exc
                        entry = entries[index]
                        results[index] = {
                            **entry.summary(),
                            "written": False,
                            "skipped": False,
                            "error": str(exc),
                        }
        else:
            for index, entry in enumerate(entries):
                try:
                    _result_index, row = _write_one(index, entry)
                    results[index] = row
                except Exception as exc:
                    errors += 1
                    if first_error is None:
                        first_error = exc
                    results[index] = {
                        **entry.summary(),
                        "written": False,
                        "skipped": False,
                        "error": str(exc),
                    }
                    break

        ordered_results = [row for row in results if isinstance(row, dict)]
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        written_count = sum(1 for row in ordered_results if row.get("written"))
        skipped_count = sum(1 for row in ordered_results if row.get("skipped"))
        report = {
            "transactionId": self.transaction_id,
            "dryRun": False,
            "entryCount": len(entries),
            "written": int(written_count),
            "skipped": int(skipped_count),
            "errors": int(errors),
            "workers": int(worker_count),
            "elapsedMs": round(elapsed_ms, 3),
            "payloadBytes": int(sum(entry.payload_bytes for entry in entries)),
            "entries": ordered_results,
        }
        _perf_add_elapsed(
            "mission_planning.replan_transaction.commit",
            started,
            entries=len(entries),
            written=int(written_count),
            skipped=int(skipped_count),
            errors=int(errors),
            workers=int(worker_count),
        )
        if self.log:
            self.log(
                "[REPLAN][TRANSACTION] "
                f"id={self.transaction_id} entries={len(entries)} "
                f"written={int(written_count)} skipped={int(skipped_count)} "
                f"errors={int(errors)} elapsedMs={elapsed_ms:.3f}"
            )
        if first_error is not None:
            raise first_error
        return report


def write_json_transaction_batch(
    entries: Iterable[tuple[Path, Any]],
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
    skip_if_unchanged: bool = True,
    log: Callable[[str], None] | None = None,
    transaction_id: str | None = None,
) -> list[dict[str, Any]]:
    normalized_entries = [(Path(path), data) for path, data in entries]
    tx = ReplanTransaction(
        transaction_id or f"replan-json-batch-{os.getpid()}-{int(time.time() * 1000)}",
        skip_if_unchanged=skip_if_unchanged,
    )
    worker_count = _json_write_workers_for_entry_count(len(normalized_entries))
    tx.stage_json_batch(
        normalized_entries,
        pretty=pretty,
        ensure_ascii=ensure_ascii,
        sort_keys=sort_keys,
        label=lambda path, _payload: path.parent.name,
        max_workers=worker_count,
    )
    report = tx.commit(
        max_workers=worker_count,
        dry_run=False,
        skip_if_unchanged=skip_if_unchanged,
    )
    results = [
        {
            "path": str(row.get("path") or ""),
            "name": str(row.get("name") or Path(str(row.get("path") or "")).name),
            "written": bool(row.get("written")),
            "skipped": bool(row.get("skipped")),
            **({"error": row.get("error")} if row.get("error") else {}),
        }
        for row in (report.get("entries") or [])
        if isinstance(row, dict)
    ]
    _mark_written_flight_paths(normalized_entries, results)
    if log:
        for row in results:
            path = row.get("path")
            if row.get("error"):
                log(f"[JSON][WRITE][ERR] {path}: {row.get('error')}")
            else:
                status = "written" if row.get("written") else "unchanged"
                log(f"[JSON][WRITE] {status}: {path}")
    return results


def write_json_transaction(
    path: str | Path,
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
    skip_if_unchanged: bool = True,
    log: Callable[[str], None] | None = None,
    transaction_id: str | None = None,
) -> bool:
    results = write_json_transaction_batch(
        [(Path(path), data)],
        pretty=pretty,
        ensure_ascii=ensure_ascii,
        sort_keys=sort_keys,
        skip_if_unchanged=skip_if_unchanged,
        log=log,
        transaction_id=transaction_id or f"replan-json-single-{os.getpid()}-{int(time.time() * 1000)}",
    )
    if not results:
        return False
    return bool(results[0].get("written"))
