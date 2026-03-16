# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple, List, Set

from modules.common import db_paths

_TARGET_INFO_NAME = "targetInfo.json"

DEFAULT_TARGET_INFO: Dict[str, Any] = {
    "targetList": {},
}


def _write_json(path: Path, data: Dict[str, Any]) -> None:
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )


def _safe_get(obj: Any, *names: str) -> Any:
    for name in names:
        if isinstance(obj, dict) and name in obj:
            return obj[name]
        if hasattr(obj, name):
            return getattr(obj, name)
    return None


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool):
        return int(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return None


def _to_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        try:
            return float(str(value).strip())
        except (TypeError, ValueError):
            return None


def _to_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    try:
        lowered = str(value).strip().lower()
    except Exception:
        return None
    if lowered in {"1", "true", "t", "y", "yes", "on"}:
        return True
    if lowered in {"0", "false", "f", "n", "no", "off"}:
        return False
    return None


def _normalize_flag(value: Any) -> int:
    iv = _to_int(value)
    return 1 if iv is not None and iv != 0 else 0


def _iterable(value: Any) -> Iterable[Any]:
    if value is None:
        return []
    if isinstance(value, (list, tuple, set)):
        return list(value)
    return [value]


def _extract_coordinate(source: Any) -> Optional[Dict[str, Optional[float]]]:
    coord_candidate = _safe_get(source, "coordinate", "Coordinate")
    for candidate in (coord_candidate, source):
        if not candidate:
            continue
        latitude = _to_float(_safe_get(candidate, "latitude", "Latitude", "lat", "Lat"))
        longitude = _to_float(_safe_get(candidate, "longitude", "Longitude", "lon", "Lon", "lng", "Lng"))
        altitude_raw = _safe_get(candidate, "altitude", "Altitude", "alt", "Alt")
        altitude = _to_float(altitude_raw)
        if latitude is None and longitude is None and altitude is None:
            continue
        result: Dict[str, Optional[float]] = {}
        if latitude is not None:
            result["latitude"] = latitude
        if longitude is not None:
            result["longitude"] = longitude
        if altitude_raw is not None:
            result["altitude"] = altitude_raw if isinstance(altitude_raw, (int, float)) else altitude
        return result
    return None


def _extract_roi_entries(message: Any) -> list[Dict[str, Any]]:
    entries: list[Dict[str, Any]] = []
    roi_list = _safe_get(message, "roiInfoList", "ROIInfoList")
    if roi_list is None:
        roi_single = _safe_get(message, "roiInfo", "ROIInfo", "roiinfo")
        if roi_single is not None:
            roi_list = [roi_single]
    for item in _iterable(roi_list):
        coordinate = _extract_coordinate(item)
        if not coordinate:
            continue
        entries.append(
            {
                "aircraftID": _to_int(_safe_get(item, "aircraftID", "AircraftID")),
                "coordinate": coordinate,
                "fov": _to_float(_safe_get(item, "fov", "Fov")),
            }
        )
    return entries


def _extract_target_entries(message: Any) -> list[Dict[str, Any]]:
    entries: list[Dict[str, Any]] = []
    target_list = _safe_get(message, "targetList", "TargetList", "targets", "Targets")
    if target_list is None:
        target_list = _safe_get(
            message, "situationAwarenessInfoList", "SituationAwarenessInfoList"
        )
    for item in _iterable(target_list):
        target_id = _to_int(_safe_get(item, "targetID", "TargetID", "targetId", "TargetId"))
        coordinate = _extract_coordinate(item)
        watcher_obj = _safe_get(item, "watcher", "Watcher")
        watcher_id = _to_int(
            _safe_get(item, "watcherID", "WatcherID", "watcherId", "aircraftID", "AircraftID")
        )
        if watcher_id is None and watcher_obj is not None:
            watcher_id = _to_int(_safe_get(watcher_obj, "aircraftID", "AircraftID"))
        entry = {
            "targetID": target_id,
            "targetType": _to_int(_safe_get(item, "targetType", "TargetType")),
            "watcherID": watcher_id,
            "coordinate": coordinate,
            "threat": _to_float(_safe_get(item, "threat", "Threat")),
            "targetInFrame": _to_bool(_safe_get(item, "targetInFrame", "TargetInFrame")),
            "isDestroyed": _to_bool(_safe_get(item, "isDestroyed", "IsDestroyed")),
        }
        entries.append(entry)
    return [entry for entry in entries if entry.get("targetID") is not None]


def _make_target_key(entry: Dict[str, Any]) -> Optional[str]:
    target_id = entry.get("targetID")
    watcher_id = entry.get("watcherID")
    if target_id is None:
        return None
    if watcher_id is None:
        return str(target_id)
    return f"{target_id}-{watcher_id}"


def _serialize_target(
    entry: Dict[str, Any],
    timestamp: Optional[int],
    *,
    existing: Optional[Dict[str, Any]] = None,
) -> Optional[Dict[str, Any]]:
    target_id = entry.get("targetID")
    if target_id is None:
        return None

    existing = existing or {}
    watcher_id = entry.get("watcherID", existing.get("watcherID"))
    if watcher_id is None:
        watcher_obj = existing.get("watcher")
        if isinstance(watcher_obj, dict):
            watcher_id = watcher_obj.get("aircraftID")

    def _first_detected_timestamp() -> Optional[int]:
        candidates = [
            existing.get("firstDetected"),
            existing.get("initDetectedTime"),
            existing.get("lastUpdated"),
            timestamp,
        ]
        numeric = [val for val in (_to_int(v) for v in candidates) if val is not None]
        return min(numeric) if numeric else None

    first_detected = _first_detected_timestamp()

    serialized: Dict[str, Any] = {
        "targetID": target_id,
        "watcherID": watcher_id,
        "targetType": entry.get("targetType"),
        "coordinate": entry.get("coordinate"),
        "isDestroyed": entry.get("isDestroyed"),
        "targetInFrame": entry.get("targetInFrame"),
        "threat": entry.get("threat"),
        "isUsed": _normalize_flag(existing.get("isUsed", 0)),
        "isIgnored": _normalize_flag(existing.get("isIgnored", 0)),
    }
    if first_detected is not None:
        serialized["firstDetected"] = first_detected
    if timestamp is not None:
        serialized["lastUpdated"] = timestamp

    elapsed_ms: Optional[int] = None
    if first_detected is not None and timestamp is not None:
        elapsed_ms = max(0, timestamp - first_detected)
    if elapsed_ms is not None:
        is_used = _to_int(serialized.get("isUsed"))
        is_ignored = _to_int(serialized.get("isIgnored"))
        if is_used == 0 or is_ignored == 0:
            serialized["sinceFirstDetectedMs"] = elapsed_ms

    return serialized


def _convert_legacy_monitoring(data: Dict[str, Any]) -> Dict[str, Any]:
    target_map: Dict[str, Dict[str, Any]] = {}
    timestamp = _to_int(data.get("timestamp"))

    for raw in data.get("targetList") or []:
        entry = {
            "targetID": _to_int(_safe_get(raw, "targetID", "TargetID")),
            "targetType": _to_int(_safe_get(raw, "targetType", "TargetType")),
            "watcherID": _to_int(
                _safe_get(raw, "watcherID", "WatcherID", "watcherId")
            ),
            "coordinate": raw.get("coordinate"),
            "threat": _to_float(raw.get("threat")),
            "targetInFrame": _to_bool(raw.get("targetInFrame")),
            "isDestroyed": _to_bool(raw.get("isDestroyed")),
        }
        if entry["watcherID"] is None:
            watcher_obj = raw.get("watcher")
            if isinstance(watcher_obj, dict):
                entry["watcherID"] = _to_int(watcher_obj.get("aircraftID"))
        key = _make_target_key(entry)
        if key is None:
            continue
        serialized = _serialize_target(
            entry,
            _to_int(raw.get("lastUpdated")) or timestamp,
            existing=raw,
        )
        if serialized:
            target_map[key] = serialized

    roi_raw = data.get("roiInfo")
    if isinstance(roi_raw, dict):
        roi_entry = {
            "aircraftID": _to_int(roi_raw.get("aircraftID")),
            "coordinate": roi_raw.get("coordinate"),
            "fov": _to_float(roi_raw.get("fov")),
            "isUsed": roi_raw.get("isUsed", 0),
            "isIgnored": roi_raw.get("isIgnored", 0),
            "initDetectedTime": _to_int(roi_raw.get("initDetectedTime")),
        }
        _upsert_roi_entry(target_map, roi_entry, timestamp)

    return {"targetList": target_map}


def _upsert_roi_entry(
    target_map: Dict[str, Dict[str, Any]],
    roi_entry: Dict[str, Any],
    timestamp: Optional[int],
) -> None:
    watcher_id = roi_entry.get("aircraftID")
    coordinate = roi_entry.get("coordinate")
    key = _find_matching_unknown(target_map, watcher_id, coordinate)
    if key is None:
        key = _next_unknown_key(target_map)
    existing = target_map.get(key, {})
    serialized = {
        "initDetectedTime": existing.get("initDetectedTime") or roi_entry.get("initDetectedTime") or timestamp,
        "watcherID": watcher_id,
        "coordinate": coordinate,
        "fov": roi_entry.get("fov"),
        "isUsed": _normalize_flag(existing.get("isUsed", roi_entry.get("isUsed", 0))),
        "isIgnored": _normalize_flag(existing.get("isIgnored", roi_entry.get("isIgnored", 0))),
    }
    target_map[key] = serialized


def _find_matching_unknown(
    target_map: Dict[str, Dict[str, Any]],
    watcher_id: Optional[int],
    coordinate: Optional[Dict[str, Any]],
) -> Optional[str]:
    for key, value in target_map.items():
        if not isinstance(key, str) or not key.startswith("unknown-"):
            continue
        if value.get("watcherID") == watcher_id and value.get("coordinate") == coordinate:
            return key
    return None


def _next_unknown_key(target_map: Dict[str, Dict[str, Any]]) -> str:
    max_idx = 0
    for key in target_map.keys():
        if isinstance(key, str) and key.startswith("unknown-"):
            try:
                idx = int(key.split("-", 1)[-1])
                max_idx = max(max_idx, idx)
            except ValueError:
                continue
    return f"unknown-{max_idx + 1}"


def _prune_resolved_roi_entries(target_map: Dict[str, Dict[str, Any]]) -> None:
    concrete_pairs = {
        (
            entry.get("watcherID"),
            _coordinate_tuple(entry.get("coordinate")),
        )
        for key, entry in target_map.items()
        if isinstance(key, str) and not key.startswith("unknown-")
    }
    for key in list(target_map.keys()):
        if not isinstance(key, str) or not key.startswith("unknown-"):
            continue
        entry = target_map.get(key, {})
        pair = (entry.get("watcherID"), _coordinate_tuple(entry.get("coordinate")))
        if pair in concrete_pairs:
            target_map.pop(key, None)


def _coordinate_tuple(coord: Any) -> Optional[tuple]:
    if not isinstance(coord, dict):
        return None
    return (
        coord.get("latitude"),
        coord.get("longitude"),
        coord.get("altitude"),
    )


def ensure_target_info_file(initial_data: Optional[Dict[str, Any]] = None) -> Path:
    """Ensure DSS_Internal/targetInfo.json exists."""
    target = db_paths.get_db_subpath("DSS_Internal", _TARGET_INFO_NAME)
    legacy = db_paths.get_db_subpath("DSS_Internal", "monitoringStatus.json")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        return target

    if legacy.exists():
        try:
            with legacy.open("r", encoding="utf-8") as fh:
                legacy_data = json.load(fh)
            converted = _convert_legacy_monitoring(legacy_data)
            _write_json(target, converted)
            try:
                legacy.unlink()
            except Exception:
                pass
            return target
        except Exception:
            pass

    _write_json(target, dict(initial_data or DEFAULT_TARGET_INFO))
    return target


def load_target_info() -> Dict[str, Any]:
    """Load targetInfo.json."""
    path = ensure_target_info_file()
    try:
        raw = path.read_text(encoding="utf-8")
        data = json.loads(raw) if raw else {}
        if not isinstance(data, dict):
            data = {}
    except Exception:
        data = {}
    merged = dict(DEFAULT_TARGET_INFO)
    target_list = data.get("targetList")
    if isinstance(target_list, dict):
        merged["targetList"] = target_list
    merged.update({k: v for k, v in data.items() if k != "targetList"})
    if "targetList" not in merged or not isinstance(merged["targetList"], dict):
        merged["targetList"] = {}
    normalized: Dict[str, Dict[str, Any]] = {}
    for key, value in merged.get("targetList", {}).items():
        if isinstance(value, dict):
            normalized_entry = dict(value)
            normalized_entry["isUsed"] = _normalize_flag(normalized_entry.get("isUsed", 0))
            normalized_entry["isIgnored"] = _normalize_flag(normalized_entry.get("isIgnored", 0))
            normalized[str(key)] = normalized_entry
    merged["targetList"] = normalized
    return merged


def save_target_info(data: Dict[str, Any]) -> None:
    """Persist targetInfo.json."""
    _write_json(ensure_target_info_file(), data)


def update_target_info_from_0402(message: Any) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """
    Update targetInfo.json with the latest snapshot extracted from a 0402 message.

    Returns:
        (updated_snapshot, newly_detected_targets)
    """
    if isinstance(message, (bytes, bytearray, str)):
        try:
            message = json.loads(message)
        except Exception:
            message = {}

    if isinstance(message, (list, tuple, set)):
        info_result: Dict[str, Any] | None = None
        detections: List[Dict[str, Any]] = []
        for item in message:
            info_result, newly = update_target_info_from_0402(item)
            detections.extend(newly)
        if info_result is None:
            info_result = load_target_info()
        return info_result, detections

    info = load_target_info()
    tracking_map: Dict[str, Dict[str, Any]] = {
        str(k): dict(v) if isinstance(v, dict) else {}
        for k, v in info.get("targetList", {}).items()
    }
    target_map: Dict[str, Dict[str, Any]] = {
        key: dict(value) for key, value in tracking_map.items()
    }

    timestamp = _to_int(_safe_get(message, "timestamp", "Timestamp"))
    roi_entries = _extract_roi_entries(message)
    target_entries = _extract_target_entries(message)
    new_targets: List[Dict[str, Any]] = []

    if roi_entries:
        for roi_entry in roi_entries:
            _upsert_roi_entry(target_map, roi_entry, timestamp)

    for entry in target_entries:
        key = _make_target_key(entry)
        if key is None:
            continue
        existing_entry = target_map.get(key)
        serialized = _serialize_target(entry, timestamp, existing=existing_entry)
        if serialized:
            target_id = entry.get("targetID")
            serialized["targetID"] = target_id

            # Preserve usage/ignore flags across watcher/key changes for the same targetID.
            if target_id is not None:
                for prev_entry in tracking_map.values():
                    if not isinstance(prev_entry, dict):
                        continue
                    if _to_int(prev_entry.get("targetID")) != _to_int(target_id):
                        continue
                    if _normalize_flag(prev_entry.get("isUsed")) == 1:
                        serialized["isUsed"] = 1
                    if _normalize_flag(prev_entry.get("isIgnored")) == 1:
                        serialized["isIgnored"] = 1
                    if (
                        _to_int(serialized.get("isUsed")) == 1
                        and _to_int(serialized.get("isIgnored")) == 1
                    ):
                        break

            existed_before = key in tracking_map
            if not existed_before and target_id is not None:
                handled_elsewhere = False
                for prev_entry in tracking_map.values():
                    if not isinstance(prev_entry, dict):
                        continue
                    if _to_int(prev_entry.get("targetID")) != _to_int(target_id):
                        continue
                    prev_used = _normalize_flag(prev_entry.get("isUsed"))
                    prev_ignored = _normalize_flag(prev_entry.get("isIgnored"))
                    if prev_used == 1 or prev_ignored == 1:
                        handled_elsewhere = True
                        break

                if not handled_elsewhere:
                    new_targets.append(
                        {
                            "key": key,
                            "targetID": target_id,
                            "watcherID": serialized.get("watcherID"),
                            "targetType": serialized.get("targetType"),
                            "coordinate": serialized.get("coordinate"),
                            "firstDetected": serialized.get("firstDetected"),
                            "lastUpdated": serialized.get("lastUpdated"),
                            "elapsedMs": serialized.get("sinceFirstDetectedMs"),
                            "threat": serialized.get("threat"),
                            "timestamp": timestamp,
                        }
                    )

            target_map[key] = serialized
            tracking_map[key] = dict(serialized)

    _prune_resolved_roi_entries(target_map)

    canonical = {
        "targetList": dict(sorted(target_map.items())),
    }
    save_target_info(canonical)
    return canonical, new_targets


def mark_targets_as_used(target_entries: Iterable[Any]) -> Dict[str, Any]:
    """
    Update targetInfo.json so that provided targets are marked with isUsed=1.
    """
    info = load_target_info()
    target_map = info.get("targetList")
    if not isinstance(target_map, dict):
        target_map = {}
        info["targetList"] = target_map

    updated = False

    for entry in target_entries or []:
        if entry is None:
            continue

        candidate_keys: List[str] = []
        target_id: Optional[int] = None
        watcher_id: Optional[int] = None

        if isinstance(entry, dict):
            key_value = entry.get("key")
            if key_value is not None:
                candidate_keys.append(str(key_value))
            target_id = _to_int(entry.get("targetID"))
            watcher_id = _to_int(entry.get("watcherID"))
            derived_key = _make_target_key({"targetID": target_id, "watcherID": watcher_id})
            if derived_key:
                candidate_keys.append(derived_key)
            if target_id is not None:
                candidate_keys.append(str(target_id))
        else:
            candidate_keys.append(str(entry))

        seen: Set[str] = set()
        unique_keys: List[str] = []
        for key_str in candidate_keys:
            if key_str and key_str not in seen:
                seen.add(key_str)
                unique_keys.append(key_str)

        matched = False
        for key_str in unique_keys:
            target_entry = target_map.get(key_str)
            if isinstance(target_entry, dict):
                matched = True
                if _to_int(target_entry.get("isUsed")) != 1:
                    target_entry["isUsed"] = 1
                    updated = True

        if matched:
            continue

        if target_id is None:
            continue

        for existing_key, target_entry in target_map.items():
            if not isinstance(target_entry, dict):
                continue
            if _to_int(target_entry.get("targetID")) != target_id:
                continue
            matched = True
            if _to_int(target_entry.get("isUsed")) != 1:
                target_entry["isUsed"] = 1
                updated = True

    if updated:
        save_target_info(info)

    return info


def mark_targets_as_ignored(target_entries: Iterable[Any]) -> Dict[str, Any]:
    """
    Update targetInfo.json so that provided targets are marked with isIgnored=1.
    """
    info = load_target_info()
    target_map = info.get("targetList")
    if not isinstance(target_map, dict):
        target_map = {}
        info["targetList"] = target_map

    updated = False

    for entry in target_entries or []:
        if entry is None:
            continue

        candidate_keys: List[str] = []
        target_id: Optional[int] = None
        watcher_id: Optional[int] = None

        if isinstance(entry, dict):
            key_value = entry.get("key")
            if key_value is not None:
                candidate_keys.append(str(key_value))
            target_id = _to_int(entry.get("targetID"))
            watcher_id = _to_int(entry.get("watcherID"))
            derived_key = _make_target_key({"targetID": target_id, "watcherID": watcher_id})
            if derived_key:
                candidate_keys.append(derived_key)
            if target_id is not None:
                candidate_keys.append(str(target_id))
        else:
            candidate_keys.append(str(entry))

        seen: Set[str] = set()
        unique_keys: List[str] = []
        for key_str in candidate_keys:
            if key_str and key_str not in seen:
                seen.add(key_str)
                unique_keys.append(key_str)

        matched = False
        for key_str in unique_keys:
            target_entry = target_map.get(key_str)
            if isinstance(target_entry, dict):
                matched = True
                if _to_int(target_entry.get("isIgnored")) != 1:
                    target_entry["isIgnored"] = 1
                    updated = True

        if matched:
            continue

        if target_id is None:
            continue

        for target_entry in target_map.values():
            if not isinstance(target_entry, dict):
                continue
            if _to_int(target_entry.get("targetID")) != target_id:
                continue
            matched = True
            if _to_int(target_entry.get("isIgnored")) != 1:
                target_entry["isIgnored"] = 1
                updated = True

    if updated:
        save_target_info(info)

    return info
