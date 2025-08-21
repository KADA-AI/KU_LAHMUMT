# # receive/message0304_receiver.py
# # ──────────────────────────────────────────────────────────────
# from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
# from nFusion.Model.msg_0304 import *              # LAHFlightPlan, Coordinate, Waypoint, etc.
# from .database import received_db
# from receive_center import notify
# import json, traceback, sys

# # ────────── 대/소문자 안전 접근 헬퍼 ──────────
# _get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# # ────────── CLR → dict 변환 ──────────
# def _lah_flight_plan_to_dict(plan: LAHFlightPlan) -> dict:
#     def coord2d(ct: Coordinate) -> dict:
#         return {
#             "latitude":  _get(ct, "latitude",  "Latitude"),
#             "longitude": _get(ct, "longitude", "Longitude"),
#             "altitude":  _get(ct, "altitude",  "Altitude")
#         }

#     body = {
#         "timestamp":      _get(plan, "timestamp",   "Timestamp"),
#         "pathID":         _get(plan, "pathID",      "PathID"),
#         "aircraftID":     _get(plan, "aircraftID",  "AircraftID"),
#         "waypointList":   []
#     }

#     for wp in _get(plan, "waypointList", "WaypointList") or []:
#         hovering = _get(wp, "hovering", "Hovering")
#         loiter   = _get(wp, "loiter",   "Loiter")
#         attack   = _get(wp, "attack",   "Attack")

#         wp_dict = {
#             "waypointID":      _get(wp, "waypointID",      "WaypointID"),
#             "coordinate":      coord2d(_get(wp, "coordinate", "Coordinate")),
#             "speed":           _get(wp, "speed",           "Speed"),
#             "eta":             _get(wp, "eta",             "Eta"),
#             "ecf":             _get(wp, "ecf",             "Ecf"),
#             "nextWaypointID":  _get(wp, "nextWaypointID",  "NextWaypointID"),
#             "hovering": {
#                 "time": _get(hovering, "time", "Time")
#             },
#             "loiter": {
#                 "radius":    _get(loiter,   "radius",    "Radius"),
#                 "direction": _get(loiter,   "direction", "Direction"),
#                 "time":      _get(loiter,   "time",      "Time"),
#                 "speed":     _get(loiter,   "speed",     "Speed")
#             },
#             "attack": {
#                 "targetID":   _get(attack,   "targetID",   "TargetID"),
#                 "weaponType": _get(attack,   "weaponType", "WeaponType")
#             }
#         }
#         body["waypointList"].append(wp_dict)

#     return body

# # ────────── Receiver 클래스 ──────────
# class LAHFlightPlanReceiver_0304(
#     IFusionReceive[LAHFlightPlan], IsLocal, IsSingletone
# ):
#     """0304 LAHFlightPlan 메시지 수신 리시버"""
#     __namespace__ = "LAHFlightPlanReceiver_0304"

#     def Receive(self, data: LAHFlightPlan, src):
#         try:
#             # 1) DB 저장
#             received_db.set_received_0304(data)

#             # 2) GUI에 JSON 바디 형태로 전달
#             notify(
#                 "0304",
#                 json.dumps(_lah_flight_plan_to_dict(data), ensure_ascii=False).encode()
#             )

#         except Exception:
#             print("[ERROR][Receive-0304] traceback ↓↓↓")
#             traceback.print_exc(file=sys.stderr)

# receive/message0304_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0304 import *              # LAHFlightPlan
from .database import received_db                 # DB 저장 모듈
from receive_center import notify                 # GUI 알림 함수

import json
import traceback
import sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── LAHFlightPlan → dict (필수 필드만) ──────────
def _lah_flight_plan_to_dict(plan: LAHFlightPlan) -> dict:
    return {
        "timestamp": _get(plan, "timestamp", "Timestamp"),
        "pathID":    _get(plan, "pathID",    "PathID"),
    }

# ────────── Receiver 클래스 ──────────
class LAHFlightPlanReceiver_0304(
    IFusionReceive[LAHFlightPlan], IsLocal, IsSingletone
):
    """0304 LAHFlightPlan 메시지 수신 리시버 (timestamp + pathID 전용)"""
    __namespace__ = "LAHFlightPlanReceiver_0304"

    def Receive(self, data: LAHFlightPlan, src):
        try:
            # 1) DB 저장
            received_db.set_received_0304(data)

            # 2) GUI 알림
            notify(
                "0304",
                json.dumps(_lah_flight_plan_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0304] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
