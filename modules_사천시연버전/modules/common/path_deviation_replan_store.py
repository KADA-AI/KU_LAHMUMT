from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import os
from modules.common import db_paths


def _base_dir() -> Path:
    raw = db_paths.get_db_subpath("DSS_Internal", "path_deviation_replan")
    return _normalize_path(raw)


def _detail_path(mission_plan_id: int) -> Path:
    return _base_dir() / f"path_deviation_detail_{int(mission_plan_id)}.json"


def save_detail(mission_plan_id: int, payload: Dict[str, Any]) -> Path:
    data = dict(payload or {})
    data.setdefault("missionPlanID", int(mission_plan_id))
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    path = _detail_path(int(mission_plan_id))
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)
    return path


def load_detail(mission_plan_id: int) -> Optional[Dict[str, Any]]:
    path = _detail_path(int(mission_plan_id))
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def save_event(stage: str, payload: Dict[str, Any]) -> Path:
    stage_name = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in str(stage or "event"))
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    path = _base_dir() / f"{stage_name}_{timestamp}.json"
    data = dict(payload or {})
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    data.setdefault("stage", stage_name)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)
    return path


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
