from __future__ import annotations

import json
import multiprocessing
import time
from pathlib import Path
from typing import Any


def _update_tracking_worker(
    state_path: str,
    state_loaded: Any,
    release_update: Any,
) -> None:
    from modules.mission_planning.runtime.state import attack_tracking

    attack_tracking._state_path = lambda: Path(state_path)
    original_load = attack_tracking._load_state

    def delayed_load() -> dict[str, object]:
        data = original_load()
        state_loaded.set()
        if not release_update.wait(10):
            raise TimeoutError("test did not release the tracking-state updater")
        return data

    attack_tracking._load_state = delayed_load
    attack_tracking.update_from_agent_states(
        [
            {
                "aircraftID": 6,
                "coordinate": {
                    "latitude": 36.1,
                    "longitude": 127.1,
                    "altitude": 100.0,
                },
                "unmannedInfo": {"currentWaypointID": {"waypointID": 222}},
            }
        ]
    )


def _clear_tracking_worker(
    state_path: str,
    clear_started: Any,
    clear_finished: Any,
) -> None:
    from modules.mission_planning.runtime.state import attack_tracking

    attack_tracking._state_path = lambda: Path(state_path)
    clear_started.set()
    attack_tracking.clear_tracking_assignment(6)
    clear_finished.set()


def test_clear_cannot_be_overwritten_by_stale_cross_process_update(tmp_path: Path) -> None:
    state_path = tmp_path / "attack_tracking_state.json"
    state_path.write_text(
        json.dumps(
            {
                "assignments": {
                    "6": {
                        "aircraft_id": 6,
                        "active": True,
                        "original_current_waypoint_id": 111,
                        "last_nonzero_waypoint_id": 111,
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    ctx = multiprocessing.get_context("spawn")
    state_loaded = ctx.Event()
    release_update = ctx.Event()
    clear_started = ctx.Event()
    clear_finished = ctx.Event()
    updater = ctx.Process(
        target=_update_tracking_worker,
        args=(str(state_path), state_loaded, release_update),
    )
    clearer = ctx.Process(
        target=_clear_tracking_worker,
        args=(str(state_path), clear_started, clear_finished),
    )

    updater.start()
    try:
        assert state_loaded.wait(10), "updater did not reach its state read"
        clearer.start()
        assert clear_started.wait(10), "clearer did not start"
        time.sleep(0.25)
        assert not clear_finished.is_set(), "clear was not serialized behind the active update"
        release_update.set()
        updater.join(10)
        clearer.join(10)
        assert updater.exitcode == 0
        assert clearer.exitcode == 0
    finally:
        release_update.set()
        for process in (updater, clearer):
            if process.pid is not None and process.is_alive():
                process.terminate()
                process.join(5)

    saved = json.loads(state_path.read_text(encoding="utf-8"))
    assignment = saved["assignments"]["6"]
    assert assignment["last_nonzero_waypoint_id"] == 222
    assert assignment["active"] is False
    assert assignment.get("cleared_at")

