"""Single-source DSS_KU release metadata and per-scenario version logging."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from functools import lru_cache
import json
import os
from pathlib import Path
import re
import threading
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CHANGE_LOG_PATH = PROJECT_ROOT / "modules" / "change_log.md"
SCENARIO_VERSION_LOG_FILENAME = "DSS_KU_VERSION.json"

_RELEASE_LINE_RE = re.compile(
    r"^(\d{4}-\d{2}-\d{2})(?:\s+(\d{2}:\d{2}(?::\d{2})?))?"
    r"\s+(v?\d+(?:\.\d+)+)\s+-\s*(.*)$"
)
_FALLBACK_RELEASE_DATE = "2026-07-24"
_FALLBACK_VERSION = "1.4.0"


@dataclass(frozen=True)
class ReleaseInfo:
    """Release identity parsed from the first valid change-log entry."""

    version: str
    release_date: str
    change_summary: str
    release_time: str = ""

    @property
    def display_version(self) -> str:
        return f"v{self.version}"

    @property
    def code_label(self) -> str:
        return f"{self.modified_at} 최종 수정본"

    @property
    def modified_at(self) -> str:
        """Human-readable local release timestamp, with legacy date fallback."""

        parts = [str(self.release_date or "").strip()]
        release_time = str(self.release_time or "").strip()
        if release_time:
            parts.append(release_time)
        return " ".join(part for part in parts if part)


@lru_cache(maxsize=8)
def load_release_info(change_log_path: str | Path | None = None) -> ReleaseInfo:
    """Read the active release from the first valid line of ``change_log.md``."""

    path = Path(change_log_path) if change_log_path is not None else CHANGE_LOG_PATH
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            match = _RELEASE_LINE_RE.match(line.strip())
            if not match:
                continue
            version = match.group(3).lstrip("vV")
            return ReleaseInfo(
                version=version,
                release_date=match.group(1),
                change_summary=match.group(4).strip(),
                release_time=str(match.group(2) or "").strip(),
            )
    except OSError:
        pass
    return ReleaseInfo(
        version=_FALLBACK_VERSION,
        release_date=_FALLBACK_RELEASE_DATE,
        change_summary="release metadata fallback",
    )


def build_scenario_version_payload(
    *,
    timestamp_ms: int | None,
    scenario_iso: str | None,
    agency: str | None,
    release: ReleaseInfo | None = None,
    recorded_at: str | None = None,
) -> dict[str, Any]:
    """Build the small human-readable payload stored once per scenario run."""

    active_release = release or load_release_info()
    recorded = recorded_at or datetime.now().astimezone().isoformat(timespec="milliseconds")
    return {
        "schemaVersion": 1,
        "recordType": "dss_ku_run_version",
        "application": "DSS_KU",
        "version": active_release.version,
        "displayVersion": active_release.display_version,
        "currentCode": active_release.code_label,
        "summary": (
            f"현재 돌린 코드: {active_release.code_label} / "
            f"버전: {active_release.version}"
        ),
        "releaseDate": active_release.release_date,
        "releaseTime": active_release.release_time or None,
        "modifiedAt": active_release.modified_at,
        "recordedAt": recorded,
        "scenario": {
            "timestampMs": None if timestamp_ms is None else int(timestamp_ms),
            "iso": None if scenario_iso is None else str(scenario_iso),
            "agency": None if agency is None else str(agency),
        },
        "metadataSource": "modules/change_log.md",
    }


def write_scenario_version_log(
    scenario_dir: str | Path,
    *,
    timestamp_ms: int | None,
    scenario_iso: str | None,
    agency: str | None,
    release: ReleaseInfo | None = None,
    recorded_at: str | None = None,
) -> Path:
    """Atomically write the current run version at the scenario-folder root."""

    root = Path(scenario_dir)
    root.mkdir(parents=True, exist_ok=True)
    destination = root / SCENARIO_VERSION_LOG_FILENAME
    payload = build_scenario_version_payload(
        timestamp_ms=timestamp_ms,
        scenario_iso=scenario_iso,
        agency=agency,
        release=release,
        recorded_at=recorded_at,
    )
    temporary = destination.with_name(
        f".{destination.name}.{os.getpid()}.{threading.get_ident()}.tmp"
    )
    try:
        with temporary.open("w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temporary, destination)
    finally:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
    return destination


__all__ = [
    "CHANGE_LOG_PATH",
    "SCENARIO_VERSION_LOG_FILENAME",
    "ReleaseInfo",
    "load_release_info",
    "build_scenario_version_payload",
    "write_scenario_version_log",
]
