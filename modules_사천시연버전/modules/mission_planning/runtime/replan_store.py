from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

from modules.common import db_paths


def _normalize_path(path: Path) -> Path:
    if os.name == "nt":
        return Path(str(path))
    text = str(path)
    if len(text) >= 2 and text[1] == ":":
        drive = text[0].lower()
        rest = text[2:].replace("\\", "/")
        rest = rest.lstrip("/")
        return Path(f"/mnt/{drive}/{rest}")
    return Path(text)


def _store_dir(store_name: str) -> Path:
    raw = db_paths.get_db_subpath("DSS_Internal", str(store_name))
    return _normalize_path(raw)


def _detail_path(store_name: str, detail_prefix: str, mission_plan_id: int) -> Path:
    return _store_dir(store_name) / f"{detail_prefix}_{int(mission_plan_id)}.json"


def save_detail(
    store_name: str,
    detail_prefix: str,
    mission_plan_id: int,
    payload: Dict[str, Any],
) -> Path:
    data = dict(payload or {})
    data.setdefault("missionPlanID", int(mission_plan_id))
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    path = _detail_path(store_name, detail_prefix, int(mission_plan_id))
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)
    return path


def load_detail(
    store_name: str,
    detail_prefix: str,
    mission_plan_id: int,
) -> Optional[Dict[str, Any]]:
    path = _detail_path(store_name, detail_prefix, int(mission_plan_id))
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def save_event(store_name: str, stage: str, payload: Dict[str, Any]) -> Path:
    stage_name = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in str(stage or "event"))
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    path = _store_dir(store_name) / f"{stage_name}_{timestamp}.json"
    data = dict(payload or {})
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    data.setdefault("stage", stage_name)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)
    return path
