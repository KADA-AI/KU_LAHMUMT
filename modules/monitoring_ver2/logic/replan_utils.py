from __future__ import annotations

from pathlib import Path
from typing import Optional

from modules.common import db_paths


def ensure_replan_level_details_file(content: str = "Hi") -> Optional[Path]:
    """
    Ensure the DSS_Internal/replanLevelDetails.json file exists for downstream consumers.

    Always writes the provided content so callers can rely on the file being up to date.
    """
    target = db_paths.get_db_subpath("DSS_Internal", "replanLevelDetails.json")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(content, encoding="utf-8")
    return target
