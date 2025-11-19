from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, List, Optional

from modules.common import db_paths


class MonitoringOperationLogger:
    """Simple text logger that stores monitoring step summaries in DSS_Internal."""

    def __init__(self, *, component: str = "MON") -> None:
        self._component = component
        self._log_path: Optional[Path] = None

    def _resolve_log_path(self) -> Path:
        base_dir = db_paths.ensure_db_payload("DSS_Internal") / "log_monitoring"
        base_dir.mkdir(parents=True, exist_ok=True)
        current_stamp = datetime.now(timezone.utc).strftime("%Y%m%d")
        expected = base_dir / f"monitoring_{current_stamp}.log"
        if self._log_path is None or self._log_path != expected:
            self._log_path = expected
        return expected

    def _timestamp(self) -> str:
        return datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S")

    def log_section(self, title: str, lines: Iterable[str]) -> None:
        entries: List[str] = [f"[{self._component}][{self._timestamp()}] {title}"]
        for line in lines:
            entries.append(f"  - {line}")
        try:
            log_path = self._resolve_log_path()
            with log_path.open("a", encoding="utf-8") as fh:
                fh.write("\n".join(entries) + "\n")
        except Exception:
            # avoid raising inside monitoring loop; best effort only
            pass
