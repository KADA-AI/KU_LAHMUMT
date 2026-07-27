from __future__ import annotations

import concurrent.futures
import threading

import pytest

from modules.mission_planning.MissionPlanner.planning_enhanced.models import (
    SplitPiece,
    SplitRunResult,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.pipeline import (
    _build_or_clone_shared_split,
    _shared_split_cache_key,
)


def _result() -> SplitRunResult:
    return SplitRunResult(
        uav_count=1,
        uav_ids=[4],
        pieces=[
            SplitPiece(
                parent_order=1,
                mission_id=11,
                mission_type=1,
                piece_index=1,
                data={"coordinateList": [{"latitude": 37.0, "longitude": 127.0}]},
                assigned_uav=4,
            )
        ],
    )


def test_shared_split_builds_once_and_returns_isolated_deepcopies() -> None:
    state = {"lock": threading.RLock(), "futures": {}}
    build_started = threading.Event()
    release_build = threading.Event()
    build_count = 0
    count_lock = threading.Lock()

    def build() -> SplitRunResult:
        nonlocal build_count
        with count_lock:
            build_count += 1
        build_started.set()
        assert release_build.wait(timeout=5.0)
        return _result()

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        first = executor.submit(_build_or_clone_shared_split, state, "same", build)
        assert build_started.wait(timeout=5.0)
        second = executor.submit(_build_or_clone_shared_split, state, "same", build)
        release_build.set()
        first_result, first_role = first.result(timeout=5.0)
        second_result, second_role = second.result(timeout=5.0)

    assert build_count == 1
    assert {first_role, second_role} == {"owner", "follower"}
    assert first_result == second_result
    assert first_result is not second_result
    assert first_result.pieces[0] is not second_result.pieces[0]
    assert first_result.pieces[0].data is not second_result.pieces[0].data

    first_result.pieces[0].data["coordinateList"][0]["latitude"] = 38.0
    assert second_result.pieces[0].data["coordinateList"][0]["latitude"] == 37.0


def test_shared_split_key_uses_filtered_payload_content() -> None:
    mrpk = {"missionReferencePackageID": 1}
    base = {
        "inputMissionPackageID": 10,
        "inputMissionList": [{"inputMissionID": 11, "inputMissionType": 1}],
    }
    reordered = {
        "inputMissionList": [{"inputMissionType": 1, "inputMissionID": 11}],
        "inputMissionPackageID": 10,
    }
    changed = {
        "inputMissionPackageID": 10,
        "inputMissionList": [{"inputMissionID": 12, "inputMissionType": 1}],
    }

    runtime = {"values": {"area_sweep_mode": "vertical"}}
    assert _shared_split_cache_key(base, mrpk, [4, 5], runtime) == _shared_split_cache_key(
        reordered,
        mrpk,
        [4, 5],
        runtime,
    )
    assert _shared_split_cache_key(base, mrpk, [4, 5], runtime) != _shared_split_cache_key(
        changed,
        mrpk,
        [4, 5],
        runtime,
    )
    assert _shared_split_cache_key(base, mrpk, [4, 5], runtime) != _shared_split_cache_key(
        base,
        mrpk,
        [4, 5],
        {"values": {"area_sweep_mode": "parallel"}},
    )


def test_shared_split_propagates_owner_failure_without_rebuilding() -> None:
    state = {"lock": threading.RLock(), "futures": {}}
    build_count = 0

    def fail() -> SplitRunResult:
        nonlocal build_count
        build_count += 1
        raise RuntimeError("split failed")

    with pytest.raises(RuntimeError, match="split failed"):
        _build_or_clone_shared_split(state, "same", fail)
    with pytest.raises(RuntimeError, match="split failed"):
        _build_or_clone_shared_split(state, "same", fail)

    assert build_count == 1
