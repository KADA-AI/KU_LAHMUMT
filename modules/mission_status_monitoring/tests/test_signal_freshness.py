from __future__ import annotations

import threading
import time
import unittest
from collections import deque

from modules.mission_status_monitoring.receiver import ReadOnly0401Integration
from modules.mission_status_monitoring.service import _now_ms_2000, _signal_age_details


class SignalFreshnessTests(unittest.TestCase):
    def test_arrival_age_tracks_latest_0401_wall_clock(self) -> None:
        receiver = ReadOnly0401Integration.__new__(ReadOnly0401Integration)
        receiver._lock = threading.RLock()
        receiver._arrival_times = deque([time.time() - 0.05], maxlen=120)

        age_ms = receiver.latest_0401_arrival_age_ms()

        self.assertIsNotNone(age_ms)
        self.assertGreaterEqual(age_ms, 0.0)
        self.assertLess(age_ms, 500.0)

    def test_recent_arrival_wins_over_lagging_sim_timestamp(self) -> None:
        class FakeIntegration:
            @staticmethod
            def latest_0401_arrival_age_ms() -> float:
                return 180.4

        effective, payload, arrival = _signal_age_details(
            FakeIntegration(),
            _now_ms_2000() - 4_000,
        )

        self.assertEqual(180, effective)
        self.assertEqual(180, arrival)
        self.assertGreaterEqual(payload or 0, 3_900)

    def test_payload_age_is_fallback_for_legacy_receiver(self) -> None:
        effective, payload, arrival = _signal_age_details(
            object(),
            _now_ms_2000() - 750,
        )

        self.assertIsNone(arrival)
        self.assertGreaterEqual(payload or 0, 700)
        self.assertEqual(payload, effective)


if __name__ == "__main__":
    unittest.main()
