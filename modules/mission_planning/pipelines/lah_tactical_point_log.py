"""Out-of-band record of tactical LAH waypoint roles, for SIM display only.

ICD 0304 ``LAHWaypoint`` carries ``hovering``, ``loiter`` and ``attack``, so a
hold point and an attack point are both recoverable from the message itself.
Concealment is not: nothing in the ICD says "this waypoint is the certified
hide point".  Rather than add a field to a fixed interface, the planner writes
the roles beside the flight path and SIM reads them for rendering.

Nothing operational may depend on this file: it is display metadata, it is
rewritten freely, and a missing or corrupt file must degrade to "no roles
known" rather than fail a mission.
"""

from __future__ import annotations

import json
from pathlib import Path
import threading
from typing import Any, Iterable

SIDECAR_NAME = "lah_tactical_points.json"
_LOCK = threading.RLock()
# Old scenarios accumulate; keep the file small enough to stay cheap to load.
_MAX_PATH_ENTRIES = 512


def sidecar_path(db_root: Path | str | None = None) -> Path | None:
    try:
        if db_root is None:
            from modules.common import db_paths

            db_root = db_paths.get_active_db_root()
        if db_root is None:
            return None
        return Path(db_root) / "DSS_Internal" / SIDECAR_NAME
    except Exception:
        return None


def load_tactical_points(db_root: Path | str | None = None) -> dict[str, Any]:
    path = sidecar_path(db_root)
    if path is None or not path.is_file():
        return {}
    try:
        with _LOCK:
            data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def record_tactical_points(
    path_id: int | None,
    *,
    conceal_waypoint_ids: Iterable[int] = (),
    hold_waypoint_ids: Iterable[int] = (),
    role: str | None = None,
    plan: dict[str, Any] | None = None,
    db_root: Path | str | None = None,
) -> bool:
    """Record which waypoints of one flight path are tactical points."""

    try:
        key = str(int(path_id))
    except (TypeError, ValueError):
        return False

    def _ids(values: Iterable[int]) -> list[int]:
        out: list[int] = []
        for value in values or ():
            try:
                parsed = int(value)
            except (TypeError, ValueError):
                continue
            if parsed > 0 and parsed not in out:
                out.append(parsed)
        return out

    entry: dict[str, Any] = {
        "concealWaypointIDs": _ids(conceal_waypoint_ids),
        "holdWaypointIDs": _ids(hold_waypoint_ids),
    }
    if role:
        entry["role"] = str(role)
    if isinstance(plan, dict):
        for source_key, target_key in (
            ("hideAchievedS", "hideAchievedS"),
            ("responseEtaS", "responseEtaS"),
            ("status", "status"),
            ("enemyVisibleCount", "enemyVisibleCount"),
            ("uavLinkCount", "uavLinkCount"),
            ("enemyAnalyzedCount", "enemyAnalyzedCount"),
        ):
            if source_key in plan:
                entry[target_key] = plan.get(source_key)
    if not entry["concealWaypointIDs"] and not entry["holdWaypointIDs"]:
        return False

    path = sidecar_path(db_root)
    if path is None:
        return False
    try:
        with _LOCK:
            existing = load_tactical_points(db_root)
            existing[key] = entry
            if len(existing) > _MAX_PATH_ENTRIES:
                # Path IDs increase monotonically, so the smallest are oldest.
                for stale in sorted(existing, key=lambda item: _sort_key(item))[
                    : len(existing) - _MAX_PATH_ENTRIES
                ]:
                    existing.pop(stale, None)
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(".json.tmp")
            temporary.write_text(
                json.dumps(existing, ensure_ascii=False, indent=1),
                encoding="utf-8",
            )
            temporary.replace(path)
    except Exception:
        return False
    return True


def _sort_key(value: str) -> tuple[int, int | str]:
    try:
        return (0, int(value))
    except (TypeError, ValueError):
        return (1, str(value))


__all__ = [
    "SIDECAR_NAME",
    "load_tactical_points",
    "record_tactical_points",
    "sidecar_path",
]
