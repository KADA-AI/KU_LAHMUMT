from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Optional

try:
    from modules.common import db_paths
except ModuleNotFoundError:
    _root = Path(__file__).resolve().parents[2]
    _root_str = str(_root)
    if _root_str not in sys.path:
        sys.path.insert(0, _root_str)
    from modules.common import db_paths

_STATE_FILENAME = "attack_assignment_state.json"
_KEY_LAST_MANNED = "last_manned_aircraft_id"


def get_last_assigned_manned_id() -> Optional[int]:
    path = db_paths.get_db_subpath("DSS_Internal") / _STATE_FILENAME
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    value = data.get(_KEY_LAST_MANNED) if isinstance(data, dict) else None
    try:
        return int(value) if value is not None else None
    except Exception:
        return None


def set_last_assigned_manned_id(aircraft_id: Optional[int]) -> None:
    if aircraft_id is None:
        return
    try:
        aircraft_id_int = int(aircraft_id)
    except Exception:
        return
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / _STATE_FILENAME
    payload = {_KEY_LAST_MANNED: aircraft_id_int}
    try:
        path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    except Exception:
        pass
