from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import os
from modules.common import db_paths


def _detail_dir() -> Path:
    raw = db_paths.get_db_subpath("DSS_Internal", "prior_replan")
    return _normalize_path(raw)


def _detail_path(mission_plan_id: int) -> Path:
    return _detail_dir() / f"prior_detail_{int(mission_plan_id)}.json"


def save_detail(mission_plan_id: int, payload: Dict[str, Any]) -> None:
    data = dict(payload or {})
    data.setdefault("missionPlanID", mission_plan_id)
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    path = _detail_path(mission_plan_id)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(path)
    except Exception:
        return


def load_detail(mission_plan_id: int) -> Optional[Dict[str, Any]]:
    path = _detail_path(mission_plan_id)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _normalize_path(path: Path) -> Path:
    """
    Convert Windows-style paths (C:\foo) to WSL-style (/mnt/c/foo) when running on POSIX.
    On Windows, the original path is returned unchanged.
    """
    if os.name == "nt":
        return Path(str(path))
    text = str(path)
    if len(text) >= 2 and text[1] == ":":
        drive = text[0].lower()
        rest = text[2:].replace("\\", "/")
        rest = rest.lstrip("/")
        return Path(f"/mnt/{drive}/{rest}")
    return Path(text)
