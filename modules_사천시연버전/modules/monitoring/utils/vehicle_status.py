from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Set

from modules.common import db_paths

MANNED_AIRCRAFT_IDS = (1, 2, 3)
UNMANNED_AIRCRAFT_IDS = (4, 5, 6)


def _normalize_available(values: Iterable[int] | None) -> Set[int]:
    available: Set[int] = set()
    if values is None:
        return available
    for value in values:
        try:
            available.add(int(value))
        except (TypeError, ValueError):
            continue
    return available


def write_vehicle_status(available_ids: Iterable[int] | None = None) -> None:
    """
    Persist the latest aircraft availability snapshot under the active scenario.
    Creates Logs/Scenario.../VehicleStatus/status.json with 0/1 flags for each aircraft.
    """
    try:
        status_root = db_paths.get_active_db_root() / "VehicleStatus"
    except Exception:
        return

    available = _normalize_available(available_ids)
    payload = {
        "updated_at": datetime.now(timezone.utc).isoformat(),
        "available": sorted(available),
        "manned": {str(aid): int(aid in available) for aid in MANNED_AIRCRAFT_IDS},
        "unmanned": {str(aid): int(aid in available) for aid in UNMANNED_AIRCRAFT_IDS},
    }

    try:
        status_root.mkdir(parents=True, exist_ok=True)
        target = status_root / "status.json"
        temp = target.with_suffix(".tmp")
        temp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        temp.replace(target)
    except Exception:
        # Persistence failures should not break monitoring.
        return
