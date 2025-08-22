# from System.Collections.Generic import List  # C# List
# from nFusion.Model.msg_0304 import *  # msg_0304에서 메시지 타입을 import
# from generator.message0304_generator import make_msg0304_body  # generator에서 메시지 바디 가져오기


# def _dict_to_obj(body_dict: dict):
#     plan = LAHFlightPlan()
#     plan.timestamp  = body_dict["timestamp"]
#     plan.pathID     = body_dict["pathID"]
#     plan.aircraftID = body_dict["aircraftID"]

#     wp_list = List[Waypoint]()
#     for wp in body_dict.get("waypointList", []):
#         wp_obj = Waypoint()
#         wp_obj.waypointID = wp["waypointID"]

#         # ── Coordinate ─────────────────────────────────────
#         cdict = wp["coordinate"]
#         coord = Coordinate()
#         coord.latitude  = cdict["latitude"]
#         coord.longitude = cdict["longitude"]
#         coord.altitude  = cdict["altitude"]
#         wp_obj.coordinate = coord

#         # ── 기본 필드 ──────────────────────────────────────
#         wp_obj.speed          = wp.get("speed", 0.0)
#         wp_obj.eta            = wp.get("eta", 0)
#         wp_obj.ecf            = wp.get("ecf", 0.0)
#         wp_obj.nextWaypointID = wp.get("nextWaypointID", 0)

#         # ── Hovering (optional) ────────────────────────────
#         hov_dict = wp.get("hovering") or {}
#         if hov_dict:
#             hovering = Hovering()
#             hovering.time = hov_dict.get("time", 0)
#             wp_obj.hovering = hovering

#         # ── Loiter (optional) ──────────────────────────────
#         loi_dict = wp.get("loiter") or {}
#         if loi_dict:
#             loiter = Loiter()
#             loiter.radius    = loi_dict.get("radius", 0)
#             loiter.direction = loi_dict.get("direction", 0)
#             loiter.time      = loi_dict.get("time", 0)
#             loiter.speed     = loi_dict.get("speed", 0.0)
#             wp_obj.loiter = loiter

#         # ── Attack (optional) ──────────────────────────────
#         atk_dict = wp.get("attack") or {}
#         if atk_dict:
#             attack = Attack()
#             attack.targetID   = atk_dict.get("targetID", 0)
#             attack.weaponType = atk_dict.get("weaponType", 0)
#             wp_obj.attack = attack

#         wp_list.Add(wp_obj)

#     plan.waypointList = wp_list
#     return plan


# import json 
# # ------------------------------------------------------------------
# def make_and_push(body, node_messenger) -> bytes:
#     """
#     body : dict 하나 또는 dict의 list
#     반환  : GUI 로그용 bytes
#     """
#     logs: list[str] = []

#     # ── 여러 기체(list)인 경우 ──────────────────────────────
#     if isinstance(body, list):
#         for item in body:
#             msg = _dict_to_obj(item)
#             node_messenger.Push(msg)
#             logs.append(
#                 f"[0304] BODY  : {json.dumps(item, ensure_ascii=False)}\n"
#                 f"[0304] PUSH 완료"
#             )
#     # ── 단일 기체(dict)인 경우 ──────────────────────────────
#     else:
#         msg = _dict_to_obj(body)
#         node_messenger.Push(msg)
#         logs.append(
#             f"[0304] BODY  : {json.dumps(body, ensure_ascii=False)}\n"
#             f"[0304] PUSH 완료"
#         )

#     return "\n".join(logs).encode()


# def make_random_and_push(node_messenger) -> bytes:
#     """
#     generator에서 바디(list or dict) 받아와 make_and_push로 전달
#     """
#     body = make_msg0304_body()
#     return make_and_push(body, node_messenger)
# # ------------------------------------------------------------------


# ─────────────────────────────────────────────────────────────
# push/message0304_push.py – 0304 LAHFlightPlan 발신 스텁
#   • FlightPath 폴더의 *.json 중 파일명 첫글자 1·2·3 → 유인기
#   • {timestamp, pathID} 두 필드만 세팅하여 Push
#   • timestamp: 2000-01-01 UTC 기준 ms
# ─────────────────────────────────────────────────────────────
# 파일: modules\common\push\message0304_push.py
# 목적: 0304 (LAHFlightPlan) PUSH
# 동작:
#   • (ENV) KU_MISSION_DB_ROOT/FlightPath 또는 상대경로 FlightPath 의 *.json 스캔
#   • 파일명 숫자이며 첫 글자가 1/2/3인 경우 → 유인기로 간주, 그 숫자를 pathID로 사용
#   • {timestamp(ms since 2000-01-01 UTC), pathID} 최소 필드만 채워 Push
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

from nFusion.Model.msg_0304 import LAHFlightPlan  # 메시지 타입

# ─────────────────────────────────────────────────────────────
# 타임스탬프: 2000-01-01 00:00:00 UTC 기준 ms
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def _get_fp_dir() -> str:
    r"""
    KU_MISSION_DB_ROOT가 있으면 <root>\MissionPlan
    없으면 '프로젝트 루트\database\MissionPlan' 로 폴백
    (이 파일 경로: ...\modules\common\push\message0301_push.py)
      └ parents[1] = common
      └ parents[2] = modules
      └ parents[3] = 프로젝트 루트
    """
    env_root = os.getenv("KU_MISSION_DB_ROOT")
    print(env_root)
    if env_root:
        return str(Path(env_root) / "FlightPath")

    proj_root = Path(__file__).resolve().parents[3]   # ← 프로젝트 루트
    return str(proj_root / "database" / "FlightPath")


def _list_path_ids() -> PyList[int]:
    """
    FlightPath 디렉터리의 *.json 중
    - 파일명이 전부 숫자이고
    - 첫 글자가 1/2/3 (유인기)
    인 파일들의 숫자 부분을 pathID로 반환
    """
    fp_dir = _get_fp_dir()
    ids: PyList[int] = []
    for path in glob.glob(os.path.join(fp_dir, "*.json")):
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem.isdigit() and stem[0] in "123":
            ids.append(int(stem))
    return sorted(ids)


def _dict_to_obj(body_dict: dict) -> LAHFlightPlan:
    """
    dict → LAHFlightPlan(C# 객체)
    • 요구사항: timestamp / pathID 두 필드만 세팅
    """
    fp = LAHFlightPlan()
    fp.timestamp = int(body_dict["timestamp"])
    fp.pathID    = int(body_dict["pathID"])
    # aircraftID / waypointList 등 나머지는 기본값 유지
    return fp


def make_and_push(body_dict: dict, node_messenger) -> Optional[bytes]:
    """
    단건 생성·전송 + GUI 로그 생성
    """
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0304] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0304] PUSH 완료"
    )
    return log_line.encode()


def make_random_and_push(node_messenger) -> Optional[bytes]:
    """
    • (ENV/상대) FlightPath의 유인기(1~3***) JSON 이름을 pathID로 사용
    • {timestamp, pathID} 메시지를 차례로 Push
    """
    logs: PyList[bytes] = []
    for pid in _list_path_ids():
        body = {
            "timestamp": _now_ms(),
            "pathID":    pid,
        }
        log = make_and_push(body, node_messenger)
        if log:
            logs.append(log)

    return b"\n".join(logs) if logs else None
