"""
generator/message0902_generator.py

ReplanRequest(0902) 메시지 자동 생성 스크립트.

* 생성 규칙
  • 시간값(timestamp, replanRequestTimestamp)은 2000‑01‑01 UTC 기준 epoch(ms)
  • ID 필드는 4‑byte 부호없는 정수(uint32) 난수
  • optionName / replanReason 등은 영문+숫자 랜덤 문자열
  • 리스트 길이는 1‑3개 범위(가변)

* 사용법
  $ python generator/message0902_generator.py            # 콘솔 출력
  $ python generator/message0902_generator.py --save     # plannedMission/ 에 JSON 저장 + 경로 안내

"""

from __future__ import annotations

import argparse
import json
import os
import random
import string
import sys
from datetime import datetime, timezone
from typing import Dict, List

# ──────────────────────────────────────────────
# 공통 유틸리티
# ──────────────────────────────────────────────

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

rand_uint: callable[[], int] = lambda: random.randint(0, 2 ** 32 - 1)
rand_str: callable[[int], str] = lambda n: "".join(random.choices(string.ascii_uppercase + string.digits, k=n))

# ──────────────────────────────────────────────
# 메시지 빌더
# ──────────────────────────────────────────────

def make_msg0902_body() -> Dict:
    """0902‑ReplanRequest 메시지 바디(dict) 생성"""
    return {
        "timestamp": _now_ms(),
        "replanRequestTime": {
            "replanRequestTimestamp": _now_ms()
        },
        "replanLevel": random.randint(0, 4),
        "inputMissionIDList": [{"inputMissionID": rand_uint()} for _ in range(random.randint(1, 3))],
        "individualMissionIDList": [{"individualMissionID": rand_uint()} for _ in range(random.randint(1, 3))],
        "priorMissionList": [{"priorMissionID": rand_uint()} for _ in range(random.randint(1, 3))],
        "replanReason": rand_str(random.randint(5, 20)),
        "optionList": [
            {
                "optionID": rand_uint(),
                "optionName": rand_str(random.randint(5, 15)),
                "missionPlanID": rand_uint(),
            }
            for _ in range(random.randint(1, 3))
        ],
    }

# ──────────────────────────────────────────────
# 저장/출력 헬퍼
# ──────────────────────────────────────────────

def save_msg(msg: Dict, out_dir: str = "plannedMission") -> str:
    """JSON 파일로 저장 후 경로 반환"""
    os.makedirs(out_dir, exist_ok=True)
    fname = f"ReplanRequest_{msg['timestamp']}.json"
    fpath = os.path.join(out_dir, fname)
    with open(fpath, "w", encoding="utf-8") as fp:
        json.dump(msg, fp, ensure_ascii=False, indent=2)
    return fpath

# ──────────────────────────────────────────────
# CLI 진입점
# ──────────────────────────────────────────────

def main(argv: List[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="0902‑ReplanRequest 메시지 생성기 (콘솔 출력 또는 저장)"
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="plannedMission/ 폴더에 JSON 저장"
    )
    args = parser.parse_args(argv)

    msg = make_msg0902_body()
    if args.save:
        path = save_msg(msg)
        print(json.dumps(msg, ensure_ascii=False, indent=2))
        print(f"✔ saved to {path}")
    else:
        print(json.dumps(msg, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main(sys.argv[1:])
