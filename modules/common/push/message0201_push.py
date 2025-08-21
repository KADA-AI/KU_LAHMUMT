
# from nFusion.Model.msg_0201 import *
# from generator.message0201_generator import make_msg0201_body  # generator에서 메시지 바디 가져오기
# from System.Collections.Generic import List  # C# List


# def _dict_to_obj(body_dict: dict):
#     """
#     dict(JSON, 소문자 카멜) → InputMissionPlan(C# 객체)
#     """
#     plan = InputMissionPlan()
#     plan.timestamp               = body_dict["timestamp"]
#     plan.inputMissionPackageID   = body_dict["inputMissionPackageID"]
#     plan.inputMissionPackageType = body_dict["inputMissionPackageType"]
#     plan.mainSensor              = body_dict["mainSensor"]

#     # ───────── AvailableAircraftList ─────────
#     avail_list = List[AvailableAircraft]()
#     for a in body_dict["availableAircraftList"]:
#         av = AvailableAircraft()
#         av.aircraftID = a["aircraftID"]
#         avail_list.Add(av)
#     plan.availableAircraftList = avail_list

#     # ───────── InputMissionList ─────────
#     im_list = List[InputMission]()
#     for im in body_dict["inputMissionList"]:
#         im_obj = InputMission()
#         im_obj.inputMissionID   = im["inputMissionID"]
#         im_obj.inputMissionType = im["inputMissionType"]
#         im_obj.isDone           = im["isDone"]

#         # ───── MissionDetail ─────
#         md = MissionDetail()
#         # CoordinateList
#         coord_list = List[Coordinate]()
#         for c in im["missionDetail"]["coordinateList"]:
#             coord = Coordinate()
#             coord.latitude  = c["latitude"]
#             coord.longitude = c["longitude"]
#             coord.altitude  = c["altitude"]
#             coord_list.Add(coord)
#         md.coordinateList = coord_list

#         # LineList
#         line_list = List[Line]()
#         for l in im["missionDetail"]["lineList"]:
#             line_obj = Line()
#             line_obj.width = l["width"]
#             line_coords = List[Coordinate]()
#             for lc in l["coordinateList"]:
#                 coord = Coordinate()
#                 coord.latitude  = lc["latitude"]
#                 coord.longitude = lc["longitude"]
#                 coord.altitude  = lc["altitude"]
#                 line_coords.Add(coord)
#             line_obj.coordinateList = line_coords
#             line_list.Add(line_obj)
#         md.lineList = line_list

#         # AreaList
#         area_list = List[Area]()
#         for a in im["missionDetail"]["areaList"]:
#             area_obj = Area()
#             area_obj.isHole = a["isHole"]
#             area_coords = List[Coordinate]()
#             for ac in a["coordinateList"]:
#                 coord = Coordinate()
#                 coord.latitude  = ac["latitude"]
#                 coord.longitude = ac["longitude"]
#                 coord.Altitude  = ac["altitude"]
#                 area_coords.Add(coord)
#             area_obj.coordinateList = area_coords
#             area_list.Add(area_obj)
#         md.areaList = area_list

#         im_obj.missionDetail = md
#         im_list.Add(im_obj)

#     plan.inputMissionList = im_list
#     return plan


# import json 
# def make_and_push(body_dict: dict, node_messenger) -> None:
#     msg = _dict_to_obj(body_dict)
#     #print(f"Message pushed: {msg}")
#     node_messenger.Push(msg)
#     # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
#     log_line = (
#         f"[0201] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
#         f"[0201] PUSH 완료"

#     )
#     ##print(log_line)
#     return log_line.encode()

# def make_random_and_push(node_messenger) -> None:
#     return make_and_push(make_msg0201_body(), node_messenger)


# push/message0201_push.py
# ─────────────────────────────────────────────────────────────
# 0201 InputMissionPlan 발신 스텁
#   • InputMissionPlan 폴더의 *.json → 파일명 숫자 = inputMissionPackageID
#   • {timestamp, inputMissionPackageID} 두 필드만 세팅하여 Push
#   • timestamp: 2000-01-01 UTC 기준 ms
# ─────────────────────────────────────────────────────────────
import os, glob, json
from datetime import datetime, timezone
from nFusion.Model.msg_0201 import *   # InputMissionPlan

# 1) ★ InputMissionPlan JSON 위치 (절대경로) -------------------
PLAN_DIR = r"C:\Users\LAHMUMT_2\Desktop\nFusion\missionPlanner\plannedMission\InputMissionPlan"

# 2) 2000-01-01 UTC 기준 ms 계산 ------------------------------
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms     = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000
)

# ─────────────────────────────────────────────────────────────
def _dict_to_obj(body_dict: dict) -> InputMissionPlan:
    """dict → InputMissionPlan(C#) – timestamp / inputMissionPackageID 만 설정"""
    plan = InputMissionPlan()
    plan.timestamp             = body_dict["timestamp"]
    plan.inputMissionPackageID = body_dict["inputMissionPackageID"]
    # 다른 필드는 기본값(0, null) 유지
    return plan


def _list_package_ids() -> list[int]:
    """
    PLAN_DIR 의 *.json 파일명 중 숫자만 → inputMissionPackageID 목록 반환
    """
    ids: list[int] = []
    for path in glob.glob(os.path.join(PLAN_DIR, "*.json")):
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem.isdigit():          # ex) "1", "7001"
            ids.append(int(stem))
    return sorted(ids)


def make_and_push(body_dict: dict, node_messenger) -> bytes | None:
    """dict → C# 객체 변환·Push, GUI 로그 bytes 반환"""
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0201] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0201] PUSH 완료"
    )
    return log_line.encode()


def make_random_and_push(node_messenger) -> bytes | None:
    """
    • PLAN_DIR 의 JSON 파일명 → inputMissionPackageID 로 사용
    • {timestamp, inputMissionPackageID} 메시지를 순차 Push
    """
    logs: list[bytes] = []
    for pid in _list_package_ids():
        body = {
            "timestamp":             _now_ms(),
            "inputMissionPackageID": pid,
        }
        log = make_and_push(body, node_messenger)
        if log:
            logs.append(log)

    return b"\n".join(logs) if logs else None
# ─────────────────────────────────────────────────────────────
