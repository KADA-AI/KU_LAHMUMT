"""Background worker loop that coordinates monitoring and replan logic."""

from __future__ import annotations

import threading
import time

from .monitoring_logic_part import MonitoringLogic
from .replan_logic_part import ReplanLogic

LOGIC_LOOP_FREQUENCY_HZ = 5
LOGIC_LOOP_SLEEP_SEC = 1.0 / LOGIC_LOOP_FREQUENCY_HZ


class MonitoringLogicHandler:
    """Runs monitoring/replan logic on a background thread."""

    def __init__(self, manager):
        self.manager = manager
        self.monitoring_logic = MonitoringLogic(manager)
        self.replan_logic = ReplanLogic(manager)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._logic_loop, daemon=True)
        self._thread.start()
        self.manager._log("LOGIC_HANDLER", "INFO", "logic thread started")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        self.manager._log("LOGIC_HANDLER", "INFO", "logic thread stopped")

    def _logic_loop(self) -> None:
        while not self._stop_event.is_set():
            system_mode = self.manager.logic_store.get_data("SystemMode")

            if system_mode == 4:
                try:
                    self.manager._log("LOGIC_LOOP", "INFO", "maintenance run start (mode=4)")
                    self.monitoring_logic.execute(mode_override=3)
                    self.replan_logic.execute(mode_override=3)
                    self.manager._log("LOGIC_LOOP", "INFO", "maintenance run end; switching to mode 1")
                except Exception as exc:
                    self.manager._log("LOGIC_LOOP", "ERROR", f"logic exception: {exc}")
                finally:
                    self.manager.set_system_mode(1)
            elif system_mode == 3:
                try:
                    self.monitoring_logic.execute()
                    self.replan_logic.execute()
                except Exception as exc:
                    self.manager._log("LOGIC_LOOP", "ERROR", f"logic exception: {exc}")

            time.sleep(LOGIC_LOOP_SLEEP_SEC)
