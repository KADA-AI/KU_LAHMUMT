from __future__ import annotations

import os
from pathlib import Path
from typing import Iterable

# Module root = .../FPL_Random
PROJECT_ROOT = Path(__file__).resolve().parents[1]
# Workspace root (sibling of FPL_Random) = .../LAHMUMT25
WORKSPACE_ROOT = PROJECT_ROOT.parent

_db_root_override: Path | None = None


def _pick_db_root() -> Path:
    """
    Decide where to store generated DB JSONs.
    Priority:
      1) Explicit override via set_db_root or FPL_DB_ROOT env var.
      2) Workspace-level /database (preferred shared location; created if missing).
      3) Local PROJECT_ROOT/MP/database (self contained copy).
    """
    if _db_root_override:
        return _db_root_override

    env_root = os.getenv("FPL_DB_ROOT")
    if env_root:
        return Path(env_root).expanduser().resolve()

    workspace_db = WORKSPACE_ROOT / "database"
    if workspace_db.exists():
        return workspace_db.resolve()

    # Prefer the shared workspace-level path even when it doesn't exist yet.
    sibling = WORKSPACE_ROOT / "MP" / "database"
    if sibling.exists():
        return workspace_db.resolve()

    return (PROJECT_ROOT / "MP" / "database").resolve()


def set_db_root(path: str | Path) -> Path:
    """Override DB root at runtime."""
    global _db_root_override
    _db_root_override = Path(path).expanduser().resolve()
    return _db_root_override


def db_root() -> Path:
    return _pick_db_root()


def ensure_dirs(paths: Iterable[Path]) -> None:
    for p in paths:
        p.mkdir(parents=True, exist_ok=True)


# Exported default root (evaluated lazily when imported)
DB_ROOT = db_root()
