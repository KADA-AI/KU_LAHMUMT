# from System.Collections.Generic import List  # C# List
# from nFusion.Model.msg_0302 import *  # msg_0302에서 메시지 타입을 import
# from nFusion.Model.msg_0302 import IndividualMissionPlan 
# from generator.message0302_generator import make_msg0302_body  # generator에서 메시지 바디 가져오기
# import json
# import time


# def _dict_to_obj(body_dict: dict):
#     """
#     JSON(dict) → nFusion.Model.msg_0302.IndividualMissionPlan 객체 변환
#     (lineList / areaList / coordinateList 존재 여부에 따라 안전 처리)
#     """
#     plan = IndividualMissionPlan()
#     plan.timestamp                  = body_dict["timestamp"]
#     plan.individualMissionPackageID = body_dict["individualMissionPackageID"]
#     plan.aircraftID                 = body_dict["aircraftID"]

#     im_list = List[IndividualMission]()             # List<IndividualMission>

#     for im in body_dict["individualMissionList"]:
#         im_obj = IndividualMission()
#         im_obj.individualMissionID = im["individualMissionID"]
#         im_obj.isDone             = im["isDone"]

#         # ── RelatedMission ──────────────────────────────────────
#         rel = RelatedMission()
#         rel.relatedMissionType = im["relatedMission"]["relatedMissionType"]
#         rel.inputMissionID     = im["relatedMission"]["inputMissionID"]
#         rel.priorMissionID     = im["relatedMission"]["priorMissionID"]
#         im_obj.relatedMission  = rel

#         # ── IndividualMissionInfo ───────────────────────────────
#         info_dict = im["individualMissionInfo"]     # 원본 딕셔너리
#         info_obj  = IndividualMissionInfo()         # C# 객체

#         info_obj.individualMissionType = info_dict["individualMissionType"]
#         info_obj.patternType           = info_dict["patternType"]
#         info_obj.autoZoomIn            = info_dict["autoZoomIn"]

#         # ─ coordinateList (포인트형) ──────────────────────────
#         coord_list = List[Coordinate]()
#         for c in info_dict.get("coordinateList", []):
#             coord = Coordinate()
#             coord.latitude  = c["latitude"]
#             coord.longitude = c["longitude"]
#             coord.altitude  = c["altitude"]
#             coord_list.Add(coord)
#         if coord_list.Count > 0:
#             info_obj.coordinateList = coord_list

#         # ─ lineList (corridor형) ──────────────────────────────
#         line_list = List[Line]()
#         for l in info_dict.get("lineList", []):
#             line_obj = Line()
#             line_obj.width = l["width"]

#             line_coord_list = List[Coordinate]()
#             for lc in l["coordinateList"]:
#                 coord = Coordinate()
#                 coord.latitude  = lc["latitude"]
#                 coord.longitude = lc["longitude"]
#                 coord.altitude  = lc["altitude"]
#                 line_coord_list.Add(coord)
#             line_obj.coordinateList = line_coord_list
#             line_list.Add(line_obj)
#         if line_list.Count > 0:
#             info_obj.lineList = line_list

#         # ─ areaList (polygon형) ───────────────────────────────
#         area_list = List[Area]()
#         for a in info_dict.get("areaList", []):
#             area_obj = Area()
#             area_obj.isHole = bool(a.get("isHole", 0))

#             area_coord_list = List[Coordinate]()
#             for ac in a["coordinateList"]:
#                 coord = Coordinate()
#                 coord.latitude  = ac["latitude"]
#                 coord.longitude = ac["longitude"]
#                 coord.altitude  = ac["altitude"]
#                 area_coord_list.Add(coord)
#             area_obj.coordinateList = area_coord_list
#             area_list.Add(area_obj)
#         if area_list.Count > 0:
#             info_obj.areaList = area_list

#         # ─ targetID ───────────────────────────────────────────
#         tid = info_dict.get("targetID")       # None 또는 값
#         if tid is not None:                   # 값이 있을 때만
#             info_obj.targetID = int(tid)      # int 캐스팅 후 대입

#         # (필요 시 relatedIndividualMissionIDList 처리 추가)

#         im_obj.individualMissionInfo = info_obj
#         im_obj.pathID                = im["pathID"]

#         im_list.Add(im_obj)

#     plan.individualMissionList = im_list
#     time.sleep(1)
#     return plan

# def make_and_push(body_dict: dict, node_messenger) -> None:
#     msg = _dict_to_obj(body_dict)
#     #print(f"Message pushed: {msg}")
#     node_messenger.Push(msg)

#     # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
#     log_line = (
#         f"[0302] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
#         f"[0302] PUSH 완료"
#     )
#     return log_line.encode()

# def make_random_and_push(node_messenger):
#     """
#     ① make_msg0302_body() → list 또는 dict 반환
#     ② list면 각 원소를 push, dict면 그대로 push
#     """
#     body = make_msg0302_body()          # 파일·랜덤 어떤 형식이든 OK

#     # ── 여러 기체(list)인 경우 ─────────────────────────
#     if isinstance(body, list):
#         logs = []
#         for item in body:
#             logs.append(make_and_push(item, node_messenger))
#         # 필요 없다면 logs 합친 값을 버려도 무방
#         return b"\n".join(logs)

#     # ── 단일 기체(dict)인 경우 ─────────────────────────
#     return make_and_push(body, node_messenger)



# ─────────────────────────────────────────────────────────────
# push/message0302_push.py – 0302 IndividualMissionPlan 발신 스텁
#   • PLAN_DIR 안 *.json → 파일명 숫자 = individualMissionPackageID
#   • {timestamp, individualMissionPackageID} 두 필드만 채워 Push
# ─────────────────────────────────────────────────────────────
# 파일: modules\common\push\message0302_push.py
# 목적: 0302(IndividualMissionPlan) PUSH - DB 루트 경로를 ENV(KU_MISSION_DB_ROOT)로부터 수신
# 동작:
#   • (ENV) KU_MISSION_DB_ROOT/IndividualMissionPlan 아래의 *.json 파일 스캔
#   • 파일명 숫자 → individualMissionPackageID 로 간주
#   • {timestamp(ms since 2000-01-01 UTC), individualMissionPackageID} 최소 필드만 채워 Push
# 반환:
#   • make_and_push(...) → bytes (GUI 로그용)
#   • make_random_and_push(...) → bytes (여러 건이면 \n로 합침), 없으면 None

from __future__ import annotations
import os
import glob
import json
from pathlib import Path
from datetime import datetime, timezone
from typing import List as PyList, Optional

# C# 메시지 타입 (nFusion 바인딩)
from nFusion.Model.msg_0302 import IndividualMissionPlan  # noqa: F401
from nFusion.Model.msg_0302 import *  # IndividualMission, RelatedMission 등은 사용 안 하지만 호환 위해 유지

# ─────────────────────────────────────────────────────────────
# 타임스탬프: 2000-01-01 00:00:00 UTC 기준 ms
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def _get_imp_dir() -> str:
    r"""
    KU_MISSION_DB_ROOT가 있으면 <root>\IndividualMissionPlan
    없으면 '프로젝트 루트\database\IndividualMissionPlan' 로 폴백
    (이 파일 경로: ...\modules\common\push\message0302_push.py)
      └ parents[1] = common
      └ parents[2] = modules
      └ parents[3] = 프로젝트 루트
    """
    env_root = os.getenv("KU_MISSION_DB_ROOT")
    if env_root:
        return str(Path(env_root) / "IndividualMissionPlan")

    proj_root = Path(__file__).resolve().parents[3]  # ← 프로젝트 루트
    return str(proj_root / "database" / "IndividualMissionPlan")


def _list_package_ids() -> PyList[int]:
    """
    IndividualMissionPlan 디렉터리의 *.json 파일명 → 숫자 individualMissionPackageID 목록
    ex) '700123.json' -> 700123
    """
    imp_dir = _get_imp_dir()
    ids: PyList[int] = []
    for path in glob.glob(os.path.join(imp_dir, "*.json")):
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem.isdigit():
            ids.append(int(stem))
    return sorted(ids)


def _dict_to_obj(body_dict: dict) -> IndividualMissionPlan:
    """
    dict → IndividualMissionPlan(C# 객체)
    • 요구사항: timestamp / individualMissionPackageID 두 필드만 세팅
    """
    imp = IndividualMissionPlan()
    imp.timestamp = int(body_dict["timestamp"])
    imp.individualMissionPackageID = int(body_dict["individualMissionPackageID"])
    # aircraftID / individualMissionList 등 나머지는 기본값 유지
    return imp


def make_and_push(body_dict: dict, node_messenger) -> Optional[bytes]:
    """
    단건 생성·전송 + GUI 로그 생성
    """
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0302] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0302] PUSH 완료"
    )
    return log_line.encode()


def make_random_and_push(node_messenger) -> Optional[bytes]:
    """
    • (ENV) KU_MISSION_DB_ROOT/IndividualMissionPlan 의 *.json → individualMissionPackageID 추출
    • 2000-01-01 UTC 기준 ms 단위 timestamp 로 전송
    """
    logs: PyList[bytes] = []
    for pid in _list_package_ids():
        body = {
            "timestamp": _now_ms(),
            "individualMissionPackageID": pid,
        }
        log = make_and_push(body, node_messenger)
        if log:
            logs.append(log)

    return b"\n".join(logs) if logs else None
