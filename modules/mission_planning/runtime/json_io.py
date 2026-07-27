# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import json
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable, Iterable

try:
    from modules.common import replan_perf
except Exception:
    import sys as _sys

    _COMMON_DIR = next(
        (
            parent / "common"
            for parent in Path(__file__).resolve().parents
            if (parent / "common" / "replan_perf.py").exists()
        ),
        None,
    )
    if _COMMON_DIR is not None and str(_COMMON_DIR) not in _sys.path:
        _sys.path.insert(0, str(_COMMON_DIR))
    import replan_perf  # type: ignore

try:
    import orjson as _orjson
except Exception:  # pragma: no cover - optional dependency
    _orjson = None

try:
    import ujson as _ujson
except Exception:  # pragma: no cover - optional dependency
    _ujson = None


def _normalize_flight_path_payload(path: Path, data: Any) -> Any:
    if not _is_flight_path_target(path, data) or "Source" not in data:
        return data

    normalized: dict[str, Any] = {}
    has_lower_source = "source" in data
    for key, value in data.items():
        if key == "Source":
            if not has_lower_source:
                normalized["source"] = value
            continue
        normalized[key] = value
    return normalized


def _is_flight_path_target(path: Path, data: Any) -> bool:
    if not isinstance(path, Path) or not isinstance(data, dict):
        return False
    if not any(isinstance(data.get(key), list) for key in ("waypointList", "uavWaypointList", "lahWaypointList")):
        return False
    if path.parent.name == "FlightPath":
        return True
    return path.stem.startswith("FlightPath")


def _apply_uav_speed_weight_to_flight_path_payload(path: Path, data: Any) -> Any:
    if not _is_flight_path_target(path, data) or not isinstance(data, dict):
        return data
    try:
        aircraft_id = int(data.get("aircraftID", 0) or 0)
    except Exception:
        aircraft_id = 0
    if aircraft_id not in (4, 5, 6):
        return data

    try:
        from ..MissionPlanner.runtime_settings import (
            UAV_SPEED_MAX_MPS,
            UAV_SPEED_MIN_MPS,
            apply_runtime_uav_speed_weight_mps,
            get_runtime_uav_speed_weight,
        )
    except Exception:
        try:
            from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
                UAV_SPEED_MAX_MPS,
                UAV_SPEED_MIN_MPS,
                apply_runtime_uav_speed_weight_mps,
                get_runtime_uav_speed_weight,
            )
        except Exception:
            apply_runtime_uav_speed_weight_mps = None  # type: ignore
            get_runtime_uav_speed_weight = None  # type: ignore
            UAV_SPEED_MIN_MPS = 0.0  # type: ignore
            UAV_SPEED_MAX_MPS = float("inf")  # type: ignore

    if get_runtime_uav_speed_weight is None:
        return data
    use_direct_runtime_apply = (
        apply_runtime_uav_speed_weight_mps is not None
        and getattr(apply_runtime_uav_speed_weight_mps, "__name__", "") != "apply_runtime_uav_speed_weight_mps"
    )

    try:
        speed_weight = float(get_runtime_uav_speed_weight(1.0))
    except Exception:
        speed_weight = 1.0
    try:
        min_speed_mps = float(UAV_SPEED_MIN_MPS)
        max_speed_mps = float(UAV_SPEED_MAX_MPS)
    except Exception:
        min_speed_mps = 0.0
        max_speed_mps = float("inf")

    def _apply_speed(value: Any) -> float | None:
        try:
            speed = float(value)
        except Exception:
            return None
        if speed <= 0.0:
            return None
        if use_direct_runtime_apply:
            weighted = float(apply_runtime_uav_speed_weight_mps(speed))  # type: ignore[misc]
        else:
            weighted = min(max_speed_mps, max(min_speed_mps, speed * speed_weight))
        return round(weighted, 2)

    updates: list[tuple[str, int, str, float]] = []
    changed_keys: list[str] = []
    for key in ("waypointList", "uavWaypointList"):
        waypoints = data.get(key)
        if not isinstance(waypoints, list):
            continue
        key_changed = False
        for idx, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                continue
            new_speed = _apply_speed(waypoint.get("speed"))
            if new_speed is not None and abs(float(waypoint.get("speed", 0.0) or 0.0) - new_speed) > 1e-6:
                updates.append((key, idx, "speed", float(new_speed)))
                key_changed = True
            loiter = waypoint.get("loiterProperty")
            if isinstance(loiter, dict):
                new_loiter_speed = _apply_speed(loiter.get("speed"))
                if (
                    new_loiter_speed is not None
                    and abs(float(loiter.get("speed", 0.0) or 0.0) - new_loiter_speed) > 1e-6
                ):
                    updates.append((key, idx, "loiterProperty.speed", float(new_loiter_speed)))
                    key_changed = True
        if key_changed:
            changed_keys.append(key)

    if not updates:
        return data

    payload = deepcopy(data)
    for key, idx, field, value in updates:
        try:
            waypoint = payload[key][idx]
            if field == "speed":
                waypoint["speed"] = float(value)
            elif field == "loiterProperty.speed":
                loiter = waypoint.get("loiterProperty")
                if isinstance(loiter, dict):
                    loiter["speed"] = float(value)
        except Exception:
            continue

    try:
        from modules.common.eta import annotate_eta_flight_plan
    except Exception:
        annotate_eta_flight_plan = None  # type: ignore

    if annotate_eta_flight_plan is not None:
        for key in changed_keys:
            try:
                annotate_eta_flight_plan(
                    payload,
                    waypoint_list_keys=(key,),
                    line_search_timing="incoming",
                )
            except Exception:
                continue

    return payload


def prepare_json_payload(path: Path, data: Any) -> Any:
    prepared = _normalize_flight_path_payload(path, data)
    prepared = _apply_uav_speed_weight_to_flight_path_payload(path, prepared)
    return prepared


def dumps_json(
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
) -> bytes:
    if _orjson is not None:
        option = 0
        if pretty:
            option |= _orjson.OPT_INDENT_2
        if sort_keys:
            option |= _orjson.OPT_SORT_KEYS
        if ensure_ascii:
            option |= _orjson.OPT_ESCAPE_UNICODE
        return _orjson.dumps(data, option=option)

    if pretty:
        text = json.dumps(
            data,
            ensure_ascii=ensure_ascii,
            indent=2,
            sort_keys=sort_keys,
        )
    else:
        text = json.dumps(
            data,
            ensure_ascii=ensure_ascii,
            separators=(",", ":"),
            sort_keys=sort_keys,
        )
    return text.encode("utf-8")


def serialize_json_payload(
    path: Path,
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
) -> bytes:
    prepared = prepare_json_payload(Path(path), data)
    # FlightPath artifacts dominate replan JSON volume.  ujson keeps the same
    # JSON values while avoiding the stdlib encoder's GIL-heavy encoding path
    # (general replans use compact FlightPath JSON). Keep this deliberately
    # narrow: all other artifacts retain the
    # historical encoder, and any optional-library incompatibility falls back
    # to it without changing the write contract.
    if (
        _orjson is None
        and _ujson is not None
        and not ensure_ascii
        and not sort_keys
        and _is_flight_path_target(Path(path), prepared)
    ):
        try:
            kwargs = {
                "ensure_ascii": False,
                "escape_forward_slashes": False,
            }
            if pretty:
                kwargs["indent"] = 2
            return _ujson.dumps(prepared, **kwargs).encode("utf-8")
        except (TypeError, ValueError, OverflowError):
            pass
    return dumps_json(
        prepared,
        pretty=pretty,
        ensure_ascii=ensure_ascii,
        sort_keys=sort_keys,
    )


def write_json_bytes(
    path: Path,
    payload: bytes,
    *,
    skip_if_unchanged: bool = True,
) -> bool:
    perf_start = replan_perf.start_timer()
    readback_start = None
    readback_ms = 0.0
    path = Path(path)
    if skip_if_unchanged and path.exists():
        try:
            if path.stat().st_size == len(payload):
                readback_start = time.perf_counter() if replan_perf.is_enabled() else None
                if path.read_bytes() == payload:
                    if readback_start is not None:
                        readback_ms = (time.perf_counter() - readback_start) * 1000.0
                    replan_perf.add_elapsed(
                        "mission_planning.json.write_bytes",
                        perf_start,
                        payload_bytes=len(payload),
                        readback_ms=readback_ms,
                        unchanged=1,
                    )
                    return False
                if readback_start is not None:
                    readback_ms = (time.perf_counter() - readback_start) * 1000.0
        except Exception:
            pass

    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_bytes(payload)
    tmp_path.replace(path)
    replan_perf.add_elapsed(
        "mission_planning.json.write_bytes",
        perf_start,
        payload_bytes=len(payload),
        readback_ms=readback_ms,
        written=1,
    )
    return True


def write_json(
    path: Path,
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
    skip_if_unchanged: bool = True,
) -> bool:
    perf_start = replan_perf.start_timer()
    serialize_start = time.perf_counter() if replan_perf.is_enabled() else None
    payload = serialize_json_payload(
        path,
        data,
        pretty=pretty,
        ensure_ascii=ensure_ascii,
        sort_keys=sort_keys,
    )
    serialize_ms = 0.0
    if serialize_start is not None:
        serialize_ms = (time.perf_counter() - serialize_start) * 1000.0
    written = write_json_bytes(path, payload, skip_if_unchanged=skip_if_unchanged)
    replan_perf.add_elapsed(
        "mission_planning.json.write_json",
        perf_start,
        payload_bytes=len(payload),
        serialize_ms=serialize_ms,
        written=1 if written else 0,
        skipped=0 if written else 1,
    )
    return written


def _write_json_batch_workers(entry_count: int) -> int:
    raw = os.getenv("DSS_REPLAN_JSON_WRITE_WORKERS", "").strip()
    if not raw:
        return 1
    try:
        value = int(float(raw))
    except Exception:
        return 1
    if value <= 1 or entry_count <= 1:
        return 1
    return max(1, min(value, entry_count))


def _path_identity_key(path: Path) -> str:
    try:
        return os.path.normcase(os.path.abspath(os.fspath(path)))
    except Exception:
        return str(path)


def _has_duplicate_paths(paths: list[Path]) -> bool:
    seen: set[str] = set()
    for path in paths:
        key = _path_identity_key(path)
        if key in seen:
            return True
        seen.add(key)
    return False


def _write_json_batch_entry(
    path: Path,
    data: Any,
    *,
    pretty: bool,
    ensure_ascii: bool,
    sort_keys: bool,
    skip_if_unchanged: bool,
) -> dict[str, Any]:
    row: dict[str, Any] = {"path": str(path), "name": path.name}
    serialize_start = time.perf_counter() if replan_perf.is_enabled() else None
    payload = serialize_json_payload(
        path,
        data,
        pretty=pretty,
        ensure_ascii=ensure_ascii,
        sort_keys=sort_keys,
    )
    serialize_ms = 0.0
    if serialize_start is not None:
        serialize_ms = (time.perf_counter() - serialize_start) * 1000.0
    written = write_json_bytes(path, payload, skip_if_unchanged=skip_if_unchanged)
    row["payloadBytes"] = int(len(payload))
    row["serializeMs"] = round(float(serialize_ms), 3)
    row["written"] = bool(written)
    row["skipped"] = not bool(written)
    replan_perf.add(
        "mission_planning.json.write_json_batch.serialize",
        elapsed_ms=serialize_ms,
        payload_bytes=len(payload),
    )
    return row


def write_json_batch(
    entries: Iterable[tuple[Path, Any]],
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
    skip_if_unchanged: bool = True,
    log: Callable[[str], None] | None = None,
) -> list[dict[str, Any]]:
    perf_start = replan_perf.start_timer()
    normalized_entries = [(Path(raw_path), data) for raw_path, data in entries]
    worker_count = _write_json_batch_workers(len(normalized_entries))
    if worker_count > 1 and _has_duplicate_paths([path for path, _data in normalized_entries]):
        worker_count = 1

    if worker_count > 1:
        results: list[dict[str, Any] | None] = [None] * len(normalized_entries)
        errors = 0
        first_error: BaseException | None = None
        with ThreadPoolExecutor(max_workers=worker_count, thread_name_prefix="json_write") as executor:
            futures = {
                executor.submit(
                    _write_json_batch_entry,
                    path,
                    data,
                    pretty=pretty,
                    ensure_ascii=ensure_ascii,
                    sort_keys=sort_keys,
                    skip_if_unchanged=skip_if_unchanged,
                ): (index, path)
                for index, (path, data) in enumerate(normalized_entries)
            }
            for future in as_completed(futures):
                index, path = futures[future]
                try:
                    results[index] = future.result()
                except Exception as exc:
                    errors += 1
                    if first_error is None:
                        first_error = exc
                    results[index] = {
                        "path": str(path),
                        "name": path.name,
                        "written": False,
                        "skipped": False,
                        "error": str(exc),
                    }
        ordered_results = [row for row in results if isinstance(row, dict)]
        for row in ordered_results:
            if log:
                path = row.get("path")
                if row.get("error"):
                    log(f"[JSON][WRITE][ERR] {path}: {row.get('error')}")
                else:
                    status = "written" if row.get("written") else "unchanged"
                    log(f"[JSON][WRITE] {status}: {path}")
        replan_perf.add_elapsed(
            "mission_planning.json.write_json_batch",
            perf_start,
            entries=len(ordered_results),
            written=sum(1 for row in ordered_results if row.get("written")),
            skipped=sum(1 for row in ordered_results if row.get("skipped")),
            errors=errors,
            workers=worker_count,
            parallel=1,
        )
        if first_error is not None:
            raise first_error
        return ordered_results

    results: list[dict[str, Any]] = []
    errors = 0
    for path, data in normalized_entries:
        row: dict[str, Any] = {"path": str(path), "name": path.name}
        try:
            written = write_json(
                path,
                data,
                pretty=pretty,
                ensure_ascii=ensure_ascii,
                sort_keys=sort_keys,
                skip_if_unchanged=skip_if_unchanged,
            )
            row["written"] = bool(written)
            row["skipped"] = not bool(written)
            if log:
                status = "written" if written else "unchanged"
                log(f"[JSON][WRITE] {status}: {path}")
        except Exception as exc:
            errors += 1
            row["written"] = False
            row["skipped"] = False
            row["error"] = str(exc)
            if log:
                log(f"[JSON][WRITE][ERR] {path}: {exc}")
            results.append(row)
            raise
        results.append(row)
    replan_perf.add_elapsed(
        "mission_planning.json.write_json_batch",
        perf_start,
        entries=len(results),
        written=sum(1 for row in results if row.get("written")),
        skipped=sum(1 for row in results if row.get("skipped")),
        errors=errors,
        workers=1,
        parallel=0,
    )
    return results
