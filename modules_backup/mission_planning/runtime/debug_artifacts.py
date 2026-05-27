from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from modules.mission_planning.runtime.json_io import write_json

_MODE_ENV = "REPLAN_RUNTIME_ARTIFACT_MODE"
_FALLBACK_MODE_ENV = "REPLAN_DEBUG_ARTIFACT_MODE"
_OFF_MODES = {"0", "false", "no", "off", "skip", "disabled", "disable", "none"}
_COMPACT_MODES = {"compact", "performance", "perf", "fast", "min", "minimal"}


def debug_artifact_mode() -> str:
    raw = os.environ.get(_MODE_ENV)
    if raw is None:
        raw = os.environ.get(_FALLBACK_MODE_ENV, "")
    value = str(raw or "").strip().lower()
    if value in _OFF_MODES:
        return "off"
    if value in _COMPACT_MODES:
        return "compact"
    return "pretty"


def debug_artifacts_enabled() -> bool:
    return debug_artifact_mode() != "off"


def debug_artifact_pretty(default: bool = True) -> bool:
    mode = debug_artifact_mode()
    if mode == "compact":
        return False
    return bool(default)


def write_debug_json(
    path: Path,
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
    skip_if_unchanged: bool = False,
) -> bool:
    mode = debug_artifact_mode()
    if mode == "off":
        return False
    return write_json(
        path,
        data,
        pretty=False if mode == "compact" else bool(pretty),
        ensure_ascii=ensure_ascii,
        sort_keys=sort_keys,
        skip_if_unchanged=skip_if_unchanged,
    )
