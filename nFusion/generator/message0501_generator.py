# ─────────────────────────────────────────────────────────
# message0501_generator.py  ★★ 0501 MissionStateInfo 메시지 생성기 ★★
# ─────────────────────────────────────────────────────────
"""
새 규격 (2025‑07‑04) — MissionID 0501 임무수행상태정보

필드 정의
-----------
Top‑level
    ulong timestamp                      : epoch(ms)
    uint  currentMissionPlanID           : 임무계획 ID
    uint  currentInputMissionID          : 현재 Input‑Mission ID
    List<IndividualMissionProgressStatus>: 각 항공기별 진행 현황

IndividualMissionProgressStatus
    uint  aircraftID                     : 항공기 번호 (0‑N)
    CurrentIndividualMission             : 진행 중인 Individual‑Mission
    uint  currentIndividualMissionProgress : 진행률 (0‑100)

CurrentIndividualMission
    uint individualMissionID             : Individual‑Mission ID
"""

import random
import json
import time

__all__ = [
    "make_msg0501_body",
]

# ──────────── 난수 헬퍼 ────────────
UINT32_MAX = (1 << 32) - 1  # 4_294_967_295
rand_uint32 = lambda: random.randint(0, UINT32_MAX)
rand_uint16 = lambda: random.randint(0, (1 << 16) - 1)
rand_int    = lambda lo=0, hi=100: random.randint(lo, hi)

# ──────────── 서브 오브젝트 빌더 ────────────

def _make_current_individual_mission() -> dict:
    """CurrentIndividualMission 객체 하나 반환"""
    return {
        "individualMissionID": rand_uint32()
    }


def _make_individual_status() -> dict:
    """IndividualMissionProgressStatus 객체 하나 생성"""
    return {
        "aircraftID": rand_int(0, 6),
        "currentIndividualMission": _make_current_individual_mission(),
        "currentIndividualMissionProgress": rand_int(0, 100)
    }

# ──────────── 메시지 바디 빌더 ────────────

def make_msg0501_body(num_individual_status: int | None = None) -> dict:
    """MissionStateInfo(0501) 메시지 바디 생성 (새 규격)"""
    if num_individual_status is None:
        num_individual_status = random.randint(1, 3)

    now_ms = int(time.time() * 1000)

    return {
        "timestamp": now_ms,
        "currentMissionPlanID": rand_uint32(),
        "currentInputMissionID": rand_uint32(),
        "individualMissionProgressStatusList": [
            _make_individual_status() for _ in range(num_individual_status)
        ]
    }


# ──────────── 스크립트 실행 시 샘플 출력 ────────────
if __name__ == "__main__":
    print(json.dumps(make_msg0501_body(), ensure_ascii=False, indent=2))
