# generator/message0902_generator.py
# -*- coding: utf-8 -*-
"""
MessageID 0902 – ReplanRequest 랜덤 바디 생성기

스펙 요약:
- ulong timestamp (ms since 2000-01-01 UTC)
- string source
- ReplanRequestTime { ulong replanRequestTimestamp }
- uint replanLevel (0..4)
- List<InputMissionID> inputMissionIDList + 단일 inputMissionID
- List<IndividualMissionID> individualMissionIDList + 단일 individualMissionID
- List<CommonType.PriorMission> priorMissionList
- string replanReason
- List<CommonType.PendingOption> pendingOptionList
"""
from __future__ import annotations

import json, random, string
from datetime import datetime, timezone
from typing import Dict, List

# ── 공통 시간 유틸(2000 epoch ms) ─────────────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
def _now_ms() -> int:
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

# ── 랜덤 유틸 ─────────────────────────────────────────────
rand_u32 = lambda: random.randint(0, 2 ** 32 - 1)
def rand_str(n: int) -> str:
    return ''.join(random.choices(string.ascii_uppercase + string.digits, k=n))

def rand_float(lo: float, hi: float, nd: int = 6) -> float:
    return round(random.uniform(lo, hi), nd)

def rand_lat() -> float: return rand_float(-90, 90, 6)
def rand_lon() -> float: return rand_float(-180, 180, 6)
def rand_alt() -> int:   return random.randint(0, 50000)  # altitude = int

def _make_coordinate_orientation() -> dict:
    return {
        "latitude":  rand_lat(),
        "longitude": rand_lon(),
        "altitude":  rand_alt(),
    }

def _make_target_orientation() -> dict:
    return { "targetID": rand_u32() }

def _make_prior_mission() -> dict:
    # CommonType.PriorMission 스펙: priorMissionID, missionType, coordinateOrientation, targetOrientation
    mtype = random.randint(1, 2)  # 1: 좌표지향, 2: 표적추적
    d = {
        "priorMissionID": rand_u32(),
        "missionType":    mtype,
    }
    if mtype == 1:
        d["coordinateOrientation"] = _make_coordinate_orientation()
    else:
        d["targetOrientation"] = _make_target_orientation()
    return d

def _make_pending_option() -> dict:
    return {
        "optionID":      rand_u32(),
        "optionName":    rand_str(random.randint(5, 15)),
        "missionPlanID": rand_u32(),
    }

# ── 메시지 바디 생성 ───────────────────────────────────────
def make_msg0902_body() -> Dict:
    return {
        "timestamp": _now_ms(),
        "source":    f"CSC{random.randint(1,5)}",
        "replanRequestTime": { "replanRequestTimestamp": _now_ms() },
        "replanLevel":            random.randint(0, 4),

        "inputMissionIDList":     [{"inputMissionID": rand_u32()} for _ in range(random.randint(1, 3))],
        "inputMissionID":         rand_u32(),

        "individualMissionIDList":[{"individualMissionID": rand_u32()} for _ in range(random.randint(1, 3))],
        "individualMissionID":    rand_u32(),

        "priorMissionList":       [_make_prior_mission() for _ in range(random.randint(1, 3))],
        "replanReason":           rand_str(random.randint(5, 20)),
        "pendingOptionList":      [_make_pending_option() for _ in range(random.randint(1, 3))],
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0902_body(), ensure_ascii=False, indent=2))
