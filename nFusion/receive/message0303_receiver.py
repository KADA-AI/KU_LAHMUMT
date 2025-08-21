# receive/message0303_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0303 import *              # UAVFlightPlan
from .database import received_db                 # DB 저장 모듈
from receive_center import notify                 # GUI 알림 함수

import json
import traceback
import sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── UAVFlightPlan → dict (필수 필드만) ──────────
def _uav_flight_plan_to_dict(plan: UAVFlightPlan) -> dict:
    return {
        "timestamp": _get(plan, "timestamp", "Timestamp"),
        "pathID":    _get(plan, "pathID",    "PathID"),
    }

# ────────── Receiver 클래스 ──────────
class UAVFlightPlanReceiver_0303(
    IFusionReceive[UAVFlightPlan], IsLocal, IsSingletone
):
    """0303 UAVFlightPlan 메시지 수신 리시버 (timestamp + pathID 전용)"""
    __namespace__ = "UAVFlightPlanReceiver_0303"

    def Receive(self, data: UAVFlightPlan, src):
        try:
            # 1) DB 저장
            received_db.set_received_0303(data)

            # 2) GUI 알림
            notify(
                "0303",
                json.dumps(_uav_flight_plan_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0303] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)


# # receive/message0303_receiver.py
# # ──────────────────────────────────────────────────────────────
# from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
# from nFusion.Model.msg_0303 import *              # UAVFlightPlan, Coordinate, etc.
# from .database import received_db
# from receive_center import notify
# import json, traceback, sys

# # ────────── 대/소문자 안전 접근 헬퍼 ──────────
# _get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# # ────────── CLR → dict 변환 ──────────
# def _uav_flight_plan_to_dict(plan: UAVFlightPlan) -> dict:
#     def coord2d(ct: Coordinate) -> dict:
#         return {
#             "latitude":  _get(ct, "latitude",  "Latitude"),
#             "longitude": _get(ct, "longitude", "Longitude"),
#             "altitude":  _get(ct, "altitude",  "Altitude")
#         }

#     body = {
#         "timestamp":         _get(plan, "timestamp",         "Timestamp"),
#         "pathID":            _get(plan, "pathID",            "PathID"),
#         "aircraftID":        _get(plan, "aircraftID",        "AircraftID"),
#         "isFormationFlight": _get(plan, "isFormationFlight", "IsFormationFlight"),
#         "formation": {
#             "leaderAircraftID": {
#                 "aircraftID": _get(_get(plan, "formation", "Formation").leaderAircraftID, 
#                                     "aircraftID", "AircraftID")
#             },
#             "formationDistanceList": []
#         },
#         "waypointList": []
#     }

#     # formationDistanceList
#     for fd in _get(_get(plan, "formation", "Formation"), "formationDistanceList", "FormationDistanceList") or []:
#         body["formation"]["formationDistanceList"].append({
#             "dX": _get(fd, "dX", "DX"),
#             "dY": _get(fd, "dY", "DY"),
#             "dZ": _get(fd, "dZ", "DZ")
#         })

#     # waypointList
#     for wp in _get(plan, "waypointList", "WaypointList") or []:
#         wp_dict = {
#             "waypointID":       _get(wp, "waypointID",       "WaypointID"),
#             "coordinate":       coord2d(_get(wp, "coordinate", "Coordinate")),
#             "speed":            _get(wp, "speed",            "Speed"),
#             "eta":              _get(wp, "eta",              "Eta"),
#             "ecf":              _get(wp, "ecf",              "Ecf"),
#             "nextWaypointID":   _get(wp, "nextWaypointID",   "NextWaypointID"),
#             "waypointPassType": _get(wp, "waypointPassType", "WaypointPassType"),
#             "loiterProperty":   {},
#             "filmingProperty":  {}
#         }

#         # loiterProperty
#         lp = _get(wp, "loiterProperty", "LoiterProperty")
#         if lp:
#             wp_dict["loiterProperty"] = {
#                 "radius":    _get(lp, "radius",    "Radius"),
#                 "direction": _get(lp, "direction", "Direction"),
#                 "time":      _get(lp, "time",      "Time"),
#                 "speed":     _get(lp, "speed",     "Speed")
#             }

#         # filmingProperty
#         fp = _get(wp, "filmingProperty", "FilmingProperty")
#         if fp:
#             fp_dict = {
#                 "fieldOfView":           _get(fp, "fieldOfView",           "FieldOfView"),
#                 "sensorType":            _get(fp, "sensorType",            "SensorType"),
#                 "operationMode":         _get(fp, "operationMode",         "OperationMode"),
#                 "coordinateOrientation": {
#                     "coordinate": coord2d(_get(_get(fp, "coordinateOrientation", "CoordinateOrientation"), "coordinate", "Coordinate"))
#                 },
#                 "lineSearch": {
#                     "searchSpeed":    _get(_get(fp, "lineSearch", "LineSearch"), "searchSpeed", "SearchSpeed"),
#                     "coordinateList": []
#                 },
#                 "autoTracking": {
#                     "targetID": _get(_get(fp, "autoTracking", "AutoTracking"), "targetID", "TargetID")
#                 },
#                 "aircraftFixed": {
#                     "gimbalPitch": _get(_get(fp, "aircraftFixed", "AircraftFixed"), "gimbalPitch", "GimbalPitch"),
#                     "gimbalYaw":   _get(_get(fp, "aircraftFixed", "AircraftFixed"), "gimbalYaw",   "GimbalYaw")
#                 },
#                 "autoScan": {
#                     "gimbalPitch":           _get(_get(fp, "autoScan", "AutoScan"), "gimbalPitch",           "GimbalPitch"),
#                     "gimbalYawAngularSpeed": _get(_get(fp, "autoScan", "AutoScan"), "gimbalYawAngularSpeed", "GimbalYawAngularSpeed"),
#                     "gimbalYawLimits": {
#                         "leftLimit":  _get(_get(_get(fp, "autoScan", "AutoScan"), "gimbalYawLimits", "GimbalYawLimits"), "leftLimit",  "LeftLimit"),
#                         "rightLimit": _get(_get(_get(fp, "autoScan", "AutoScan"), "gimbalYawLimits", "GimbalYawLimits"), "rightLimit", "RightLimit")
#                     }
#                 }
#             }

#             # lineSearch.coordinateList
#             for lc in _get(_get(fp, "lineSearch", "LineSearch"), "coordinateList", "CoordinateList") or []:
#                 fp_dict["lineSearch"]["coordinateList"].append(coord2d(lc))

#             wp_dict["filmingProperty"] = fp_dict

#         body["waypointList"].append(wp_dict)

#     return body

# # ────────── Receiver 클래스 ──────────
# class UAVFlightPlanReceiver_0303(
#     IFusionReceive[UAVFlightPlan], IsLocal, IsSingletone
# ):
#     """0303 UAVFlightPlan 메시지 수신 리시버"""
#     __namespace__ = "UAVFlightPlanReceiver_0303"

#     def Receive(self, data: UAVFlightPlan, src):
#         try:
#             # 1) DB 저장
#             received_db.set_received_0303(data)

#             # 2) GUI 알림
#             notify(
#                 "0303",
#                 json.dumps(_uav_flight_plan_to_dict(data), ensure_ascii=False).encode()
#             )

#         except Exception:
#             print("[ERROR][Receive-0303] traceback ↓↓↓")
#             traceback.print_exc(file=sys.stderr)
