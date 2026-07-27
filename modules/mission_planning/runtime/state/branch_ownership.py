from __future__ import annotations

"""Sticky per-UAV branch ownership store for Type 2 (ground maneuver support).

Type 2 "각자도생" (self-reliance) missions split a mission's multi-element
lineList/areaList so that each list element is a *branch* owned by a single UAV
for the whole branch (no width split). That ownership is established once at the
first 경계지역(regionType=7) branch mission (spec step 4) and must persist,
unchanged, across the following collaborative base missions (steps 5, 6) and any
replans in between - an enemy found in a UAV's own branch stays that UAV's
responsibility; another UAV never takes over.

The map is keyed by inputMissionPackageID (stable across the scenario) and stores
``branch_index -> [aircraftID, ...]``. A branch normally owns exactly one UAV; it
owns several only when there are more UAVs than branches and the branch was
sub-divided among them (spec: "무인기 수가 많아서 두대 이상이 할당된 경우는 나눠서").

This mirrors :mod:`modules.mission_planning.runtime.state.prior_tracking` for the
file-IO/atomic-write conventions so behaviour matches the rest of the runtime
state layer.
"""

import json
import os
import threading
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from modules.common import db_paths

_STATE_FILENAME = "branch_ownership_state.json"
_STATE_LOCK = threading.RLock()


def _state_path():
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    return directory / _STATE_FILENAME


def _load_state() -> dict:
    path = _state_path()
    if not path.exists():
        return {"packages": {}}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        try:
            text = path.read_text(encoding="utf-8")
            data, end = json.JSONDecoder().raw_decode(text)
            if str(text[end:]).strip():
                _save_state(data)
        except Exception:
            return {"packages": {}}
    if not isinstance(data, dict):
        return {"packages": {}}
    if not isinstance(data.get("packages"), dict):
        data["packages"] = {}
    return data


def _save_state(payload: dict) -> None:
    path = _state_path()
    tmp = path.with_name(f"{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
        tmp.replace(path)
    except Exception:
        try:
            tmp.unlink(missing_ok=True)
        except Exception:
            pass


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _package_key(package_id: Any) -> Optional[str]:
    pid = _to_int(package_id)
    if pid is None or pid <= 0:
        return None
    return str(pid)


def _normalize_ownership(ownership: Any) -> Dict[int, List[int]]:
    """Coerce a ``branch_index -> aircraft(s)`` mapping into ints/lists."""
    out: Dict[int, List[int]] = {}
    if not isinstance(ownership, dict):
        return out
    for raw_index, raw_owners in ownership.items():
        branch_index = _to_int(raw_index)
        if branch_index is None or branch_index < 0:
            continue
        owners: List[int] = []
        source = raw_owners if isinstance(raw_owners, (list, tuple, set)) else [raw_owners]
        for item in source:
            aid = _to_int(item)
            if aid is None or aid <= 0 or aid in owners:
                continue
            owners.append(int(aid))
        if owners:
            out[int(branch_index)] = owners
    return out


def register_branch_ownership(
    *,
    package_id: Any,
    branch_count: int,
    ownership: Dict[Any, Any],
    branch_mission_ids: Any = None,
    anchor_input_mission_id: Any = None,
    source: Optional[str] = None,
    immutable: bool = True,
) -> Dict[int, List[int]]:
    """Persist the first branch->UAV ownership for an input package.

    Once registered, the mapping is immutable for that package. Scenario DB
    roots isolate separate runs, so an initial-plan refresh inside one scenario
    reuses the existing mapping rather than moving it with the UAVs.
    """
    key = _package_key(package_id)
    normalized = _normalize_ownership(ownership)
    if key is None or not normalized:
        return normalized
    with _STATE_LOCK:
        data = _load_state()
        packages = data.setdefault("packages", {})
        existing_entry = packages.get(key)
        if (
            bool(immutable)
            and isinstance(existing_entry, dict)
            and bool(existing_entry.get("active", True))
        ):
            existing = _normalize_ownership(existing_entry.get("ownership"))
            if existing:
                return existing

        aircraft_to_branch: Dict[str, int] = {}
        for branch_index, owners in normalized.items():
            for aircraft_id in owners:
                aircraft_to_branch.setdefault(str(int(aircraft_id)), int(branch_index))
        packages[key] = {
            "package_id": int(key),
            "active": True,
            "branch_count": int(branch_count),
            "ownership_locked": bool(immutable),
            "ownership_version": 2,
            # JSON object keys must be strings; store as str, read back as int.
            "ownership": {str(idx): [int(a) for a in owners] for idx, owners in normalized.items()},
            "aircraft_to_branch": aircraft_to_branch,
            "branch_mission_ids": [
                int(x) for x in (branch_mission_ids or []) if _to_int(x) is not None
            ],
            "anchor_input_mission_id": _to_int(anchor_input_mission_id),
            "source": str(source) if source else None,
            "registered_at": _now_iso(),
            "updated_at": _now_iso(),
        }
        _save_state(data)
    return normalized


def update_branch_ownership(
    package_id: Any,
    ownership: Dict[Any, Any],
    *,
    source: Optional[str] = None,
) -> Dict[int, List[int]]:
    """Update a legacy mutable mapping; immutable Type-2 maps ignore it.

    Returns the authoritative mapping, or an empty mapping when no initial
    registration exists.
    """
    key = _package_key(package_id)
    normalized = _normalize_ownership(ownership)
    if key is None or not normalized:
        return normalized
    with _STATE_LOCK:
        data = _load_state()
        entry = data.get("packages", {}).get(key)
        if not isinstance(entry, dict) or not bool(entry.get("active", True)):
            return {}
        existing = _normalize_ownership(entry.get("ownership"))
        if bool(entry.get("ownership_locked", False)):
            return existing
        entry["ownership"] = {
            str(branch_index): [int(aircraft_id) for aircraft_id in owners]
            for branch_index, owners in normalized.items()
        }
        entry["aircraft_to_branch"] = {
            str(int(aircraft_id)): int(branch_index)
            for branch_index, owners in normalized.items()
            for aircraft_id in owners
        }
        entry["updated_at"] = _now_iso()
        if source:
            entry["reassigned_by"] = str(source)
        _save_state(data)
        return normalized


def get_branch_ownership(package_id: Any) -> Dict[int, List[int]]:
    """Return ``{branch_index: [aircraftID, ...]}`` for a package (empty if none)."""
    key = _package_key(package_id)
    if key is None:
        return {}
    data = _load_state()
    entry = data.get("packages", {}).get(key)
    if not isinstance(entry, dict) or not bool(entry.get("active", True)):
        return {}
    return _normalize_ownership(entry.get("ownership"))


def get_branch_owner(package_id: Any, branch_index: Any) -> List[int]:
    """Return the owner aircraft list for a single branch (empty if unknown)."""
    idx = _to_int(branch_index)
    if idx is None:
        return []
    return list(get_branch_ownership(package_id).get(int(idx), []))


def get_branch_meta(package_id: Any) -> Dict[str, Any]:
    """Return ``{branch_count, ownership, branch_mission_ids}`` for a package.

    Empty dict when there is no active ownership. Used by the split pipeline to
    recognize a branch mission during a single-mission next-collab replan, where
    the mission's own regionType is not enough to place it in the branch span.
    """
    key = _package_key(package_id)
    if key is None:
        return {}
    data = _load_state()
    entry = data.get("packages", {}).get(key)
    if not isinstance(entry, dict) or not bool(entry.get("active", True)):
        return {}
    ownership = _normalize_ownership(entry.get("ownership"))
    if not ownership:
        return {}
    return {
        "branch_count": int(entry.get("branch_count") or (max(ownership) + 1)),
        "ownership": ownership,
        "ownership_locked": bool(entry.get("ownership_locked", True)),
        "ownership_version": int(entry.get("ownership_version") or 1),
        "aircraft_to_branch": {
            int(aircraft_id): int(branch_index)
            for branch_index, owners in ownership.items()
            for aircraft_id in owners
        },
        "anchor_input_mission_id": _to_int(entry.get("anchor_input_mission_id")),
        "branch_mission_ids": [
            int(x) for x in (entry.get("branch_mission_ids") or []) if _to_int(x) is not None
        ],
    }


def get_locked_type2_branch_ownership(
    input_plan: Any,
    input_mission_id: Any = None,
) -> Dict[int, List[int]]:
    """Return immutable ownership only for a Type-2 branch-set mission."""
    if not isinstance(input_plan, dict):
        return {}
    package_type = _to_int(
        input_plan.get("inputMissionPackageType")
        or input_plan.get("InputMissionPackageType")
        or input_plan.get("packageType")
    )
    if package_type != 2:
        return {}
    package_id = _to_int(
        input_plan.get("inputMissionPackageID")
        or input_plan.get("InputMissionPackageID")
        or input_plan.get("inputMissionPackageId")
    )
    if package_id is None or package_id <= 0:
        return {}
    meta = get_branch_meta(int(package_id))
    ownership = meta.get("ownership") if isinstance(meta, dict) else None
    if not isinstance(ownership, dict) or not ownership:
        return {}
    mission_id = _to_int(input_mission_id)
    if mission_id is not None:
        branch_mission_ids = {
            int(value)
            for value in (meta.get("branch_mission_ids") or [])
            if _to_int(value) is not None
        }
        if int(mission_id) not in branch_mission_ids:
            return {}
    return {
        int(branch_index): [int(aircraft_id) for aircraft_id in owners]
        for branch_index, owners in ownership.items()
    }


def is_locked_type2_branch_mission(input_plan: Any, input_mission_id: Any) -> bool:
    if not isinstance(input_plan, dict):
        return False
    package_type = _to_int(
        input_plan.get("inputMissionPackageType")
        or input_plan.get("InputMissionPackageType")
        or input_plan.get("packageType")
    )
    if package_type != 2:
        return False
    mission_id = _to_int(input_mission_id)
    if mission_id is None:
        return False
    if get_locked_type2_branch_ownership(input_plan, int(mission_id)):
        return True
    # Fail closed for legacy/corrupt state: the input package itself can still
    # prove that this mission belongs to the Type-2 branch set. Callers then
    # preserve the current IMP instead of falling back to pooled redistribution.
    try:
        from modules.mission_planning.pipelines.ground_maneuver_mode import (
            detect_ground_maneuver_profile,
        )

        profile = detect_ground_maneuver_profile(input_plan, package_type=2)
    except Exception:
        profile = None
    if not isinstance(profile, dict):
        return False
    return int(mission_id) in {
        int(value)
        for value in (profile.get("branchInputMissionIDs") or [])
        if _to_int(value) is not None
    }


def clear_branch_ownership(package_id: Any, *, reason: Optional[str] = None) -> None:
    key = _package_key(package_id)
    if key is None:
        return
    data = _load_state()
    entry = data.get("packages", {}).get(key)
    if not isinstance(entry, dict):
        return
    entry["active"] = False
    entry["cleared_at"] = _now_iso()
    if reason:
        entry["clear_reason"] = str(reason)
    _save_state(data)
