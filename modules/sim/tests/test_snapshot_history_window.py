from __future__ import annotations

from typing import Any, Iterator

from modules.sim.runtime.sim_service import SimulationService


class _ReverseOnlyHistory:
    def __init__(self, frames: list[dict[str, Any]]) -> None:
        self._frames = frames
        self.reverse_visits = 0

    def __iter__(self) -> Iterator[dict[str, Any]]:
        raise AssertionError("snapshot must not copy or scan history forwards")

    def __reversed__(self) -> Iterator[dict[str, Any]]:
        for frame in reversed(self._frames):
            self.reverse_visits += 1
            yield frame


class _ForbiddenHistory:
    def __iter__(self) -> Iterator[dict[str, Any]]:
        raise AssertionError("history is not part of a non-incremental snapshot")

    def __reversed__(self) -> Iterator[dict[str, Any]]:
        raise AssertionError("history is not part of a non-incremental snapshot")


def _minimal_frame(**_kwargs) -> dict[str, Any]:
    return {
        "vehicles": {},
        "targets": [],
        "rois": [],
        "projectiles": [],
        "effects": [],
    }


def test_history_window_walks_backwards_and_stops_at_response_limit(monkeypatch) -> None:
    sim = SimulationService()
    history = _ReverseOnlyHistory([{"step": step} for step in range(1, 2_001)])
    sim._history = history
    sim._history_response_max = 3
    sim.geo = object()
    monkeypatch.setattr(sim, "_build_frame", _minimal_frame)
    monkeypatch.setattr(sim, "_capture_enemy_lah_los_job_locked", lambda **_kwargs: None)
    monkeypatch.setattr(sim, "_build_enemy_lah_los_links", lambda **_kwargs: [])
    monkeypatch.setattr(sim, "_build_lah_uav_communication_links", lambda **_kwargs: [])

    try:
        snapshot = sim.build_snapshot(since_step=1)
    finally:
        sim.shutdown()

    assert [frame["step"] for frame in snapshot["history"]] == [1998, 1999, 2000]
    assert history.reverse_visits == 3


def test_history_window_stops_at_since_boundary_in_chronological_order() -> None:
    sim = SimulationService.__new__(SimulationService)
    history = _ReverseOnlyHistory([{"step": step} for step in range(1, 11)])
    sim._history = history
    sim._history_response_max = 5

    frames = sim._history_frames_since_locked(8)

    assert [frame["step"] for frame in frames] == [9, 10]
    assert history.reverse_visits == 3


def test_non_incremental_snapshot_does_not_touch_history() -> None:
    sim = SimulationService()
    sim._history = _ForbiddenHistory()
    try:
        snapshot = sim.build_snapshot()
    finally:
        sim.shutdown()

    assert "history" not in snapshot

