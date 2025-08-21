# from System.Collections.Generic import List
# from nFusion.Model.msg_0303 import *
# from generator.message0303_generator import make_msg0303_body


# def _dict_to_obj(body_dict: dict):
#     plan = UAVFlightPlan()
#     plan.timestamp         = body_dict["timestamp"]
#     plan.pathID            = body_dict["pathID"]
#     plan.aircraftID        = body_dict["aircraftID"]
#     plan.isFormationFlight = body_dict["isFormationFlight"]

#     # ───────── Formation ─────────
#     form_dict  = body_dict.get("formation", {})
#     formation  = Formation()

#     # 실제 JSON은 readerAircraftID 로 저장돼 있으므로 둘 다 체크
#     leader_block = form_dict.get("leaderAircraftID") or form_dict.get("readerAircraftID") or {}
#     leader = LeaderAircraftID()
#     leader.aircraftID = leader_block.get("aircraftID", 0)
#     formation.leaderAircraftID = leader

#     fd_list = List[FormationDistance]()
#     for fd in form_dict.get("formationDistanceList", []):
#         fd_obj = FormationDistance()
#         fd_obj.dX = fd.get("dX", 0.0)
#         fd_obj.dY = fd.get("dY", 0.0)
#         fd_obj.dZ = fd.get("dZ", 0.0)
#         fd_list.Add(fd_obj)
#     formation.formationDistanceList = fd_list
#     plan.formation = formation

#     # ───────── Waypoints ─────────
#     wp_list = List[Waypoint]()
#     for wp in body_dict.get("waypointList", []):
#         wp_obj = Waypoint()
#         wp_obj.waypointID        = wp["waypointID"]

#         coord = Coordinate()
#         coord.latitude  = wp["coordinate"]["latitude"]
#         coord.longitude = wp["coordinate"]["longitude"]
#         coord.altitude  = wp["coordinate"]["altitude"]
#         wp_obj.coordinate = coord

#         wp_obj.speed            = wp.get("speed", 0.0)
#         wp_obj.eta              = wp.get("eta", 0)
#         wp_obj.ecf              = wp.get("ecf", 0.0)
#         wp_obj.nextWaypointID   = wp.get("nextWaypointID", 0)
#         wp_obj.waypointPassType = wp.get("waypointPassType", 0)

#         # ── LoiterProperty (optional) ─────────────────────
#         lp_dict = wp.get("loiterProperty")
#         if lp_dict:
#             loiter  = LoiterProperty()
#             loiter.radius    = lp_dict.get("radius", 0)
#             loiter.direction = lp_dict.get("direction", 0)
#             loiter.time      = lp_dict.get("time", 0)
#             loiter.speed     = lp_dict.get("speed", 0.0)
#             wp_obj.loiterProperty = loiter

#         # ── FilmingProperty (optional) ────────────────────
#         fp_dict = wp.get("filmingProperty")
#         if fp_dict:
#             film    = FilmingProperty()
#             film.fieldOfView  = fp_dict.get("fieldOfView", 0.0)
#             film.sensorType   = fp_dict.get("sensorType", 0)
#             film.operationMode = fp_dict.get("operationMode", 0)

#             # CoordinateOrientation
#             if "coordinateOrientation" in fp_dict:
#                 co = CoordinateOrientation()
#                 co_coord = Coordinate()
#                 cdict = fp_dict["coordinateOrientation"]["coordinate"]
#                 co_coord.latitude  = cdict.get("latitude", 0.0)
#                 co_coord.longitude = cdict.get("longitude", 0.0)
#                 co_coord.altitude  = cdict.get("altitude", 0.0)
#                 co.coordinate = co_coord
#                 film.coordinateOrientation = co

#             # LineSearch
#             if "lineSearch" in fp_dict:
#                 ls = LineSearch()
#                 ls.searchSpeed = fp_dict["lineSearch"].get("searchSpeed", 0.0)
#                 ls_coords = List[Coordinate]()
#                 for lc in fp_dict["lineSearch"].get("coordinateList", []):
#                     c = Coordinate()
#                     c.latitude  = lc.get("latitude", 0.0)
#                     c.longitude = lc.get("longitude", 0.0)
#                     c.altitude  = lc.get("altitude", 0.0)
#                     ls_coords.Add(c)
#                 ls.coordinateList = ls_coords
#                 film.lineSearch = ls

#             # AutoTracking
#             if "autoTracking" in fp_dict:
#                 at = AutoTracking()
#                 at.targetID = fp_dict["autoTracking"].get("targetID", 0)
#                 film.autoTracking = at

#             # AircraftFixed
#             if "aircraftFixed" in fp_dict:
#                 af = AircraftFixed()
#                 af.gimbalPitch = fp_dict["aircraftFixed"].get("gimbalPitch", 0.0)
#                 af.gimbalYaw   = fp_dict["aircraftFixed"].get("gimbalYaw", 0.0)
#                 film.aircraftFixed = af

#             # AutoScan
#             if "autoScan" in fp_dict:
#                 asc = AutoScan()
#                 asc.gimbalPitch            = fp_dict["autoScan"].get("gimbalPitch", 0.0)
#                 asc.gimbalYawAngularSpeed  = fp_dict["autoScan"].get("gimbalYawAngularSpeed", 0.0)
#                 gyl = GimbalYawLimits()
#                 gyl.leftLimit  = fp_dict["autoScan"]["gimbalYawLimits"].get("leftLimit", 0.0)
#                 gyl.rightLimit = fp_dict["autoScan"]["gimbalYawLimits"].get("rightLimit", 0.0)
#                 asc.gimbalYawLimits = gyl
#                 film.autoScan = asc

#             wp_obj.filmingProperty = film

#         wp_list.Add(wp_obj)

#     plan.waypointList = wp_list
#     return plan
# import json 
# def make_and_push(body, node_messenger) -> bytes:
#     """
#     body: dict 1개 -or- dict 의 list
#     반환: GUI 로그용 bytes
#     """
#     logs = []

#     # ── 여러 항공기(list)인 경우 ─────────────────────────
#     if isinstance(body, list):
#         for item in body:
#             msg = _dict_to_obj(item)
#             node_messenger.Push(msg)
#             logs.append(
#                 f"[0303] BODY  : {json.dumps(item, ensure_ascii=False)}\n"
#                 f"[0303] PUSH 완료"
#             )
#     # ── 단일 항공기(dict)인 경우 ─────────────────────────
#     else:
#         msg = _dict_to_obj(body)
#         node_messenger.Push(msg)
#         logs.append(
#             f"[0303] BODY  : {json.dumps(body, ensure_ascii=False)}\n"
#             f"[0303] PUSH 완료"
#         )

#     return "\n".join(logs).encode()

# def make_random_and_push(node_messenger) -> None:
#     return make_and_push(make_msg0303_body(), node_messenger)



# ─────────────────────────────────────────────────────────────
# push/message0303_push.py – 0303 UAVFlightPlan 발신 스텁
#   • FlightPath 폴더의 *.json 중 파일명 첫글자 4‧5‧6 → 무인기
#   • {timestamp, pathID} 두 필드만 채워 Push
#   • timestamp: 2000-01-01 UTC 기준 ms
# ─────────────────────────────────────────────────────────────
import os, glob, json
from datetime import datetime, timezone
from nFusion.Model.msg_0303 import *   # UAVFlightPlan, …

# 1) ★ FlightPath JSON 위치 (절대경로) --------------------------
PLAN_DIR = r"C:\Users\LAHMUMT_2\Desktop\nFusion\missionPlanner\plannedMission\FlightPath"

# 2) 2000-01-01 UTC 기준 ms 계산용 ------------------------------
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms     = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000
)

# ─────────────────────────────────────────────────────────────
def _dict_to_obj(body_dict: dict):
    """dict → UAVFlightPlan(C# 객체) – timestamp / pathID 만 세팅"""
    fp = UAVFlightPlan()
    fp.timestamp = body_dict["timestamp"]
    fp.pathID    = body_dict["pathID"]
    # aircraftID·waypointList 등은 기본값(0, null) 그대로
    return fp


def _list_path_ids() -> list[int]:
    """
    PLAN_DIR 의 *.json 중
    • 파일명이 전부 숫자이고
    • 첫 글자가 4·5·6 (무인기) → pathID 목록 반환
    """
    ids: list[int] = []
    for path in glob.glob(os.path.join(PLAN_DIR, "*.json")):
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem.isdigit() and stem[0] in "456":
            ids.append(int(stem))
    return sorted(ids)


def make_and_push(body_dict: dict, node_messenger) -> bytes | None:
    """dict → C# 객체 변환·Push, GUI 로그 bytes 반환"""
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0303] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0303] PUSH 완료"
    )
    return log_line.encode()


def make_random_and_push(node_messenger) -> bytes | None:
    """
    • PLAN_DIR 의 무인기(4~6**) JSON 이름을 pathID 로 사용
    • {timestamp, pathID} 메시지를 차례로 Push
    """
    logs: list[bytes] = []
    for pid in _list_path_ids():
        body = {
            "timestamp": _now_ms(),
            "pathID":    pid,
        }
        log = make_and_push(body, node_messenger)
        if log:
            logs.append(log)

    return b"\n".join(logs) if logs else None
# ─────────────────────────────────────────────────────────────
