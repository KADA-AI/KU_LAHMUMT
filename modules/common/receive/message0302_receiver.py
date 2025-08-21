# # receive/message0302_receiver.py
# # ──────────────────────────────────────────────────────────────
# from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
# from nFusion.Model.msg_0302 import *              # IndividualMissionPlan, IndividualMission, etc.
# from .database import received_db
# from receive_center import notify
# import json, traceback, sys

# # ────────── 대/소문자 안전 접근 헬퍼 ──────────
# _get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# # ────────── CLR → dict 변환 ──────────
# def _im_plan_to_dict(plan: IndividualMissionPlan) -> dict:
#     def c2d(ct):
#         return {
#             "latitude":  _get(ct, "latitude",  "Latitude"),
#             "longitude": _get(ct, "longitude", "Longitude"),
#             "altitude":  _get(ct, "altitude",  "Altitude")
#         }

#     body = {
#         "timestamp":                _get(plan, "timestamp",                "Timestamp"),
#         "individualMissionPackageID": _get(plan, "individualMissionPackageID", "IndividualMissionPackageID"),
#         "aircraftID":               _get(plan, "aircraftID",               "AircraftID"),
#         "individualMissionList":    []
#     }

#     for im in _get(plan, "individualMissionList", "IndividualMissionList") or []:
#         related = _get(im, "relatedMission", "RelatedMission")
#         info    = _get(im, "individualMissionInfo", "IndividualMissionInfo")

#         im_dict = {
#             "individualMissionID": _get(im, "individualMissionID", "IndividualMissionID"),
#             "isDone":             _get(im, "isDone",             "IsDone"),
#             "relatedMission": {
#                 "relatedMissionType": _get(related, "relatedMissionType", "RelatedMissionType"),
#                 "inputMissionID":     _get(related, "inputMissionID",     "InputMissionID"),
#                 "priorMissionID":     _get(related, "priorMissionID",     "PriorMissionID")
#             },
#             "individualMissionInfo": {
#                 "individualMissionType": _get(info, "individualMissionType", "IndividualMissionType"),
#                 "patternType":           _get(info, "patternType",           "PatternType"),
#                 "autoZoomIn":            _get(info, "autoZoomIn",            "AutoZoomIn"),
#                 "coordinateList":        [
#                     c2d(c) for c in _get(info, "coordinateList", "CoordinateList") or []
#                 ],
#                 "lineList": [
#                     {
#                         "width":          _get(ln, "width", "Width"),
#                         "coordinateList": [c2d(c) for c in _get(ln, "coordinateList", "CoordinateList") or []]
#                     }
#                     for ln in _get(info, "lineList", "LineList") or []
#                 ],
#                 "areaList": [
#                     {
#                         "isHole":         _get(ar, "isHole", "IsHole"),
#                         "coordinateList": [c2d(c) for c in _get(ar, "coordinateList", "CoordinateList") or []]
#                     }
#                     for ar in _get(info, "areaList", "AreaList") or []
#                 ],
#                 "targetID": _get(info, "targetID", "TargetID"),
#             },
#             "pathID": _get(im, "pathID", "PathID")
#         }
#         body["individualMissionList"].append(im_dict)

#     return body

# # ────────── Receiver 클래스 ──────────
# class IndividualMissionPlanReceiver_0302(
#     IFusionReceive[IndividualMissionPlan], IsLocal, IsSingletone
# ):
#     """0302 IndividualMissionPlan 메시지 수신 리시버"""
#     __namespace__ = "IndividualMissionPlanReceiver_0302"

#     def Receive(self, data: IndividualMissionPlan, src):
#         try:
#             # DB 저장
#             received_db.set_received_0302(data)

#             # GUI 알림
#             notify(
#                 "0302",
#                 json.dumps(_im_plan_to_dict(data), ensure_ascii=False).encode()
#             )

#         except Exception:
#             print("[ERROR][Receive-0302] traceback ↓↓↓")
#             traceback.print_exc(file=sys.stderr)


# receive/message0302_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0302 import *              # IndividualMissionPlan
from .database import received_db                 # DB 저장 모듈
from receive_center import notify                 # GUI 알림 함수

import json
import traceback
import sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── IndividualMissionPlan → dict (필수 필드만) ──────────
def _im_plan_to_dict(plan: IndividualMissionPlan) -> dict:
    return {
        "timestamp":                 _get(plan, "timestamp",                 "Timestamp"),
        "individualMissionPackageID": _get(plan, "individualMissionPackageID", "IndividualMissionPackageID"),
    }

# ────────── Receiver 클래스 ──────────
class IndividualMissionPlanReceiver_0302(
    IFusionReceive[IndividualMissionPlan], IsLocal, IsSingletone
):
    """0302 IndividualMissionPlan 메시지 수신 리시버 (timestamp + individualMissionPackageID 전용)"""
    __namespace__ = "IndividualMissionPlanReceiver_0302"

    def Receive(self, data: IndividualMissionPlan, src):
        try:
            # 1) DB 저장
            received_db.set_received_0302(data)

            # 2) GUI 알림
            notify(
                "0302",
                json.dumps(_im_plan_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0302] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
