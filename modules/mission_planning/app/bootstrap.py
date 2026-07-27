"""Bootstrap helpers for the mission-planning application shell."""

from __future__ import annotations

import os
from typing import MutableMapping


MISSION_ROLE = "mission"
PROCESS_LOG_NAME = "mission_planning"
DEFAULT_CONSOLE_TITLE = "KU Mission Planning Console"


def configure_mission_role(environ: MutableMapping[str, str] | None = None) -> str:
    env = os.environ if environ is None else environ
    env["KU_ROLE"] = MISSION_ROLE
    return MISSION_ROLE


def configure_mission_process_console(environ: MutableMapping[str, str] | None = None) -> None:
    env = os.environ if environ is None else environ
    from modules.common.process_console import ensure_console, install_process_file_logging

    ensure_console(env.get("KU_CONSOLE_TITLE", DEFAULT_CONSOLE_TITLE))
    install_process_file_logging(PROCESS_LOG_NAME)
