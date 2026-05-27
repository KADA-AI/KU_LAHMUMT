"""
0202 전용 ID 할당기.
- 선행임무(0202)로 생성되는 개별임무/경로/웨이포인트 ID를 기존 파이프라인과 분리해 관리한다.
"""
from __future__ import annotations

import json
import threading
from pathlib import Path

from .id_allocator import next_path_id as _general_next_path_id, next_waypoint_id as _general_next_waypoint_id

_LOCK = threading.Lock()
_STORE = Path(__file__).resolve().parent / "id_tracker_0202.json"

_BASE_STATE = {
    "individualMissionID": 950_000_000,  # 0202 전용 individual 미션 시작 값
}


def _load_state() -> dict:
    if not _STORE.exists():
        return dict(_BASE_STATE)
    try:
        with _STORE.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
            if isinstance(data, dict):
                merged = dict(_BASE_STATE)
                merged.update(data)
                return merged
    except Exception:
        pass
    return dict(_BASE_STATE)


def _save_state(state: dict) -> None:
    tmp = _STORE.with_suffix(".tmp")
    with tmp.open("w", encoding="utf-8") as fh:
        json.dump(state, fh, ensure_ascii=False, separators=(",", ":"))
    tmp.replace(_STORE)


_STATE = _load_state()


def next_0202_individual_mission_id() -> int:
    with _LOCK:
        current = int(_STATE.get("individualMissionID", _BASE_STATE["individualMissionID"]))
        current += 1
        _STATE["individualMissionID"] = current
        try:
            _save_state(_STATE)
        except Exception:
            pass
        return current


def next_0202_path_id(aircraft_id: int) -> int:
    """0202 전용 pathID는 일반 배정 규칙을 그대로 따르되, 헬퍼명을 분리한다."""
    return _general_next_path_id(int(aircraft_id))


def next_0202_waypoint_id() -> int:
    """전용 웨이포인트 ID (공용 카운터 활용)."""
    return _general_next_waypoint_id()
