from __future__ import annotations

import os
import time
from collections import deque
from copy import deepcopy
from typing import Any

from modules.common.receive_center import register_listener, unregister_listener
from modules.common.settings_paths import fusion_runtime_working_dir
from modules.mission_status_monitoring.footprint_context import (
    build_0401_footprint_context,
)
from modules.sim.integration.integration_service import IntegrationService, _safe_json


class ReadOnly0401Integration(IntegrationService):
    """Read-only nFusion receiver for mission-status dashboard inputs."""

    _READ_MESSAGE_IDS = ("0401", "0402", "0602", "0701", "0702", "0903")

    def __init__(self) -> None:
        self._arrival_times: deque[float] = deque(maxlen=120)
        self._0402_events: deque[dict[str, Any]] = deque(maxlen=512)
        self._0602_events: deque[dict[str, Any]] = deque(maxlen=512)
        super().__init__(node_name="MISSION_STATUS_MONITOR")

    def _init_bus(self) -> None:
        if self.enabled and self._messenger is not None:
            return
        try:
            os.environ.setdefault("KU_ROLE", "mission_status_monitor")
            root = self._project_root()
            self._ensure_fusion_configs(root)
            self._load_msglib_and_deps(root)
            from modules.common.dll_files.nFusionImports import FusionNodeIoc, NodeMessenger

            __import__("modules.common.receive", fromlist=["*"])
            with fusion_runtime_working_dir(project_root=root):
                FusionNodeIoc.Configure()
                NodeMessenger.Initialize(self._node_name)
                NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
                NodeMessenger.InitAllSubscriberFromAssembly()
            self._messenger = NodeMessenger
            self.enabled = True
            self.error = None
        except Exception as exc:
            self.enabled = False
            self.error = str(exc)
            return

        for msg_id in self._READ_MESSAGE_IDS:
            register_listener(msg_id, self._on_receive)

    def _on_receive(self, msg_id: str, payload: object) -> None:
        msg_id = str(msg_id).zfill(4)
        if msg_id not in self._READ_MESSAGE_IDS:
            return
        observed_at = time.time()
        normalized_payload = _safe_json(payload)
        aircraft_context: dict[str, Any] = {}
        if msg_id in {"0402", "0602"}:
            with self._lock:
                latest_0401 = deepcopy(self.rx_payload.get("0401"))
            try:
                aircraft_context = build_0401_footprint_context(latest_0401)
            except Exception:
                aircraft_context = {}
        with self._lock:
            self.rx_payload[msg_id] = normalized_payload
            self._record_time(self.rx_times, msg_id, self.rx_counts)
            if msg_id == "0401":
                self._arrival_times.append(observed_at)
            elif msg_id == "0402":
                events = getattr(self, "_0402_events", None)
                if events is None:
                    events = deque(maxlen=512)
                    self._0402_events = events
                events.append(
                    {
                        "arrivalUnixMs": int(observed_at * 1000.0),
                        "payload": deepcopy(normalized_payload),
                        "footprintContext": aircraft_context,
                    }
                )
            elif msg_id == "0602":
                events = getattr(self, "_0602_events", None)
                if events is None:
                    events = deque(maxlen=512)
                    self._0602_events = events
                events.append(
                    {
                        "arrivalUnixMs": int(observed_at * 1000.0),
                        "payload": deepcopy(normalized_payload),
                        "aircraftContext": aircraft_context,
                    }
                )

    def latest_0401(self) -> dict[str, Any] | None:
        payload = self.latest_payload("0401")
        return payload if isinstance(payload, dict) else None

    def latest_payload(self, msg_id: str) -> Any:
        """Return a detached copy of the latest read-only integration payload."""
        key = str(msg_id).zfill(4)
        if key not in self._READ_MESSAGE_IDS:
            return None
        with self._lock:
            return deepcopy(self.rx_payload.get(key))

    def drain_0402_events(self) -> list[dict[str, Any]]:
        """Return every 0402 received since the last dashboard state poll."""

        with self._lock:
            events = getattr(self, "_0402_events", None)
            if not events:
                return []
            rows = [deepcopy(item) for item in events]
            events.clear()
        return rows

    def drain_0602_events(self) -> list[dict[str, Any]]:
        """Return every 0602 command received since the last state poll."""

        with self._lock:
            events = getattr(self, "_0602_events", None)
            if not events:
                return []
            rows = [deepcopy(item) for item in events]
            events.clear()
        return rows

    def receive_rate_hz(self) -> float | None:
        with self._lock:
            times = list(self._arrival_times)
        if len(times) < 2:
            return None
        cutoff = times[-1] - 10.0
        recent = [value for value in times if value >= cutoff]
        if len(recent) < 2 or recent[-1] <= recent[0]:
            return None
        return (len(recent) - 1) / (recent[-1] - recent[0])

    def latest_0401_arrival_age_ms(self) -> float | None:
        """Return wall-clock age of the most recently received 0401 packet."""

        with self._lock:
            latest = self._arrival_times[-1] if self._arrival_times else None
        if latest is None:
            return None
        return max(0.0, (time.time() - float(latest)) * 1000.0)

    def shutdown(self) -> None:
        for msg_id in self._READ_MESSAGE_IDS:
            try:
                unregister_listener(msg_id, self._on_receive)
            except Exception:
                pass
        super().shutdown()
