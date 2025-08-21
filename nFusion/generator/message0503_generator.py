# ─────────────────────────────────────────────────────────
# message0503_generator.py  ★★ MissionID 0503 – MissionResults 생성기 ★★
# ─────────────────────────────────────────────────────────
"""
MissionID 0503 협업기저임무 완료 알림 (MissionResults)

필드 정의 (새 규격 · 2025‑07‑04)
---------------------------------------------------
Top‑level
    ulong timestamp                : ms since 2000‑01‑01 00:00:00 UTC
    uint  type                     : 결과 타입(0~3 등)
    List<IndividualMission> individualMissionList
    uint  inputMissionID
    uint  systemRecommend          : 시스템 권고 값(0/1)

IndividualMission
    uint aircraftID                : 0~6
    uint individualMissionID
"""

import random, json
from datetime import datetime, timezone

# ── Epoch (2000‑01‑01 UTC) ───────────────────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

UINT32_MAX = (1 << 32) - 1
rand_uint32 = lambda: random.randint(0, UINT32_MAX)
rand_int    = lambda lo=0, hi=100: random.randint(lo, hi)


def make_msg0503_body(num_individual: int | None = None) -> dict:
    """0503 MissionResults 메시지 바디 생성"""
    if num_individual is None:
        num_individual = random.randint(1, 3)

    im_list = [
        {
            "aircraftID":       rand_int(0, 6),
            "individualMissionID": rand_uint32(),
        }
        for _ in range(num_individual)
    ]

    return {
        "timestamp":            _now_ms(),
        "type":                 rand_int(0, 3),
        "individualMissionList": im_list,
        "inputMissionID":       rand_uint32(),
        "systemRecommend":      rand_int(0, 1),
    }


if __name__ == "__main__":
    print(json.dumps(make_msg0503_body(), ensure_ascii=False, indent=2))
