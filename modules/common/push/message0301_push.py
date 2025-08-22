# 파일: modules\common\push\message0301_push.py
# 모듈-레벨 헬퍼 추가
import os, glob, json
from pathlib import Path
from datetime import datetime, timezone
from System.Collections.Generic import List
from nFusion.Model.msg_0301 import *

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

# ★ 추가: ENV → MissionPlan 폴더 경로 도출
def _get_plan_dir() -> str:
    r"""
    KU_MISSION_DB_ROOT가 있으면 <root>\MissionPlan
    없으면 '프로젝트 루트\database\MissionPlan' 로 폴백
    (이 파일 경로: ...\modules\common\push\message0301_push.py)
      └ parents[1] = common
      └ parents[2] = modules
      └ parents[3] = 프로젝트 루트
    """
    env_root = os.getenv("KU_MISSION_DB_ROOT")
    # print(env_root)
    if env_root:
        return str(Path(env_root) / "MissionPlan")

    proj_root = Path(__file__).resolve().parents[3]   # ← 프로젝트 루트
    return str(proj_root / "database" / "MissionPlan")


def _dict_to_obj(body_dict: dict):
    """
    dict → MissionPlan(C# 객체)
    • 요구사항에 따라 timestamp / missionPlanID 두 필드만 설정
    """
    mp = MissionPlan()
    mp.timestamp     = body_dict["timestamp"]
    mp.missionPlanID = body_dict["missionPlanID"]
    return mp


def _list_plan_ids() -> list[int]:
    """ENV 기반 MissionPlan 디렉터리의 *.json 파일명을 숫자 missionPlanID 목록으로 반환"""
    ids: list[int] = []
    plan_dir = _get_plan_dir()
    for path in glob.glob(os.path.join(plan_dir, "*.json")):
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem.isdigit():  # ex) "700000"
            ids.append(int(stem))
    return sorted(ids)


def make_and_push(body_dict: dict, node_messenger) -> bytes | None:
    """
    dict → MissionPlan(C#) 변환 후 Push · GUI 로그용 bytes 반환
    • 0301 규격: timestamp / missionPlanID 두 필드만 전송
    """
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0301] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0301] PUSH 완료"
    )
    return log_line.encode()


def make_random_and_push(node_messenger) -> bytes | None:
    """
    • (ENV) KU_MISSION_DB_ROOT/MissionPlan 의 *.json → missionPlanID 추출
    • 2000-01-01 UTC 기준 ms 단위 timestamp 로 전송
    """
    logs: list[bytes] = []
    for mid in _list_plan_ids():
        body = {
            "timestamp": _now_ms(),
            "missionPlanID": mid,
        }
        log = make_and_push(body, node_messenger)
        if log:
            logs.append(log)
    return b"\n".join(logs) if logs else None
