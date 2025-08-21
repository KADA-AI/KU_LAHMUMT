# from System.Collections.Generic import List  # C# List
# from nFusion.Model.msg_0301 import *  # msg_0301에서 메시지 타입을 import
# from generator.message0301_generator import make_msg0301_body  # generator에서 메시지 바디 가져오기


# def _dict_to_obj(body_dict: dict):
#     mission_plan = MissionPlan()
#     mission_plan.timestamp                 = body_dict["timestamp"]
#     mission_plan.missionPlanID             = body_dict["missionPlanID"]
#     mission_plan.missionPlanTimestamp      = body_dict["missionPlanTimestamp"]
#     mission_plan.planningTime              = body_dict["planningTime"]
#     mission_plan.plannerID                 = body_dict["plannerID"]
#     mission_plan.inputMissionPackageID     = body_dict["inputMissionPackageID"]
#     mission_plan.missionReferencePackageID = body_dict["missionReferencePackageID"]

#     aircraft_list = List[Aircraft]()  # List<Aircraft>
#     for ac in body_dict["aircraftList"]:
#         ac_entry = Aircraft()
#         ac_entry.aircraftID                     = ac["aircraftID"]
#         ac_entry.individualMissionPackageID = ac["individualMissionPackageID"]
#         aircraft_list.Add(ac_entry)

#     mission_plan.aircraftList = aircraft_list
#     return mission_plan

# import json 
# def make_and_push(body_dict: dict, node_messenger) -> None:
#     msg = _dict_to_obj(body_dict)
#     #print(f"Message pushed: {msg}")
#     node_messenger.Push(msg)
#     # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
#     log_line = (
#         f"[0301] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
#         f"[0301] PUSH 완료"
#     )
#     ##print(log_line)
#     return log_line.encode()

# def make_random_and_push(node_messenger) -> None:
#     return make_and_push(make_msg0301_body(), node_messenger)


# ─────────────────────────────────────────────────────────────
# push/message0301_push.py – 0301 MissionPlan 발신 스텁
#   • PLAN_DIR 안 *.json → 파일명 숫자 = missionPlanID
#   • {timestamp, missionPlanID} 두 필드만 채워서 여러 번 Push
# ─────────────────────────────────────────────────────────────
import os, glob, time, json
from System.Collections.Generic import List    # (기존 import 유지)
from nFusion.Model.msg_0301 import *           # (기존 import 유지)
from datetime import datetime, timezone   # ← 추가

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms     = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000
)

# 1) ★ MissionPlan JSON 저장 위치 (절대경로) -------------------
PLAN_DIR = r"C:\Users\LAHMUMT_2\Desktop\nFusion\missionPlanner\plannedMission\MissionPlan"

# ─────────────────────────────────────────────────────────────
def _dict_to_obj(body_dict: dict):
    """
    dict → MissionPlan(C# 객체)
    • 요구사항에 따라 timestamp / missionPlanID 두 필드만 설정
    """
    mp = MissionPlan()
    mp.timestamp     = body_dict["timestamp"]
    mp.missionPlanID = body_dict["missionPlanID"]
    # 나머지 필드는 기본값(0, null) 그대로 둡니다.
    return mp


def _list_plan_ids() -> list[int]:
    """PLAN_DIR의 *.json 파일명을 숫자 missionPlanID 목록으로 반환"""
    ids: list[int] = []
    for path in glob.glob(os.path.join(PLAN_DIR, "*.json")):
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem.isdigit():          # ex) "700000"
            ids.append(int(stem))
    return sorted(ids)

# ───────────────────────── 추가 / 복구 ─────────────────────────
def make_and_push(body_dict: dict, node_messenger) -> bytes | None:
    """
    dict → MissionPlan(C#) 변환 후 Push · GUI 로그용 bytes 반환
    • 0301 규격: timestamp / missionPlanID 두 필드만 전송
    """
    msg = _dict_to_obj(body_dict)      # ← 위에서 정의한 변환 함수 사용
    node_messenger.Push(msg)           # 실제 전송

    # ── GUI 로그 문자열 만들기 ------------------------------
    log_line = (
        f"[0301] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0301] PUSH 완료"
    )
    return log_line.encode()

def make_random_and_push(node_messenger) -> bytes | None:
    """
    • PLAN_DIR의 *.json → missionPlanID 추출
    • 2000-01-01 UTC 기준 ms 단위 timestamp 로 전송
    """
    logs: list[bytes] = []
    for mid in _list_plan_ids():
        body = {
            "timestamp": _now_ms(),     # ← 여기!
            "missionPlanID": mid,
        }
        log = make_and_push(body, node_messenger)
        if log:
            logs.append(log)

    return b"\n".join(logs) if logs else None
