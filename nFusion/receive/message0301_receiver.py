# # receive/message0301_receiver.py
# # ──────────────────────────────────────────────────────────────
# from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
# from nFusion.Model.msg_0301 import *              # MissionPlan, Aircraft
# from .database import received_db
# from receive_center import notify
# import json, traceback, sys


# # ────────── 대/소문자 안전 접근 헬퍼 ──────────
# _get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# # ────────── CLR → dict 변환 ──────────
# def _mission_plan_to_dict(mp: MissionPlan) -> dict:
#     body = {
#         "timestamp":                _get(mp, "timestamp",                "Timestamp"),
#         "missionPlanID":            _get(mp, "missionPlanID",            "MissionPlanID"),
#         "missionPlanTimestamp":     _get(mp, "missionPlanTimestamp",     "MissionPlanTimestamp"),
#         "planningTime":             _get(mp, "planningTime",             "PlanningTime"),
#         "plannerID":                _get(mp, "plannerID",                "PlannerID"),
#         "inputMissionPackageID":    _get(mp, "inputMissionPackageID",    "InputMissionPackageID"),
#         "missionReferencePackageID":_get(mp, "missionReferencePackageID","MissionReferencePackageID"),
#         "aircraftList":             []
#     }

#     for ac in (_get(mp, "aircraftList", "AircraftList") or []):
#         ac_dict = {
#             "aircraftID":                     _get(ac, "aircraftID",                     "AircraftID"),
#             "individualMissionPackageID": _get(ac, "individualMissionPackageID", "IndividualMissionPackageID")
#         }
#         body["aircraftList"].append(ac_dict)

#     return body

# # ────────── Receiver 클래스 ──────────
# class MissionPlanReceiver_0301(
#     IFusionReceive[MissionPlan], IsLocal, IsSingletone
# ):
#     """0301 MissionPlan 메시지 수신 리시버"""
#     __namespace__ = "MissionPlanReceiver_0301"

#     def Receive(self, data: MissionPlan, src):
#         try:
#             # 1) DB 저장
#             received_db.set_received_0301(data)

#             # 2) GUI 알림
#             notify(
#                 "0301",
#                 json.dumps(_mission_plan_to_dict(data), ensure_ascii=False).encode()
#             )

#         except Exception:
#             print("[ERROR][Receive-0301] traceback ↓↓↓")
#             traceback.print_exc(file=sys.stderr)


# receive/message0301_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0301 import *              # MissionPlan
from .database import received_db                 # DB 저장 모듈
from receive_center import notify                 # GUI 알림 함수

import json
import traceback
import sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── MissionPlan → dict (필수 필드만) ──────────
def _mission_plan_to_dict(mp: MissionPlan) -> dict:
    return {
        "timestamp":     _get(mp, "timestamp",     "Timestamp"),
        "missionPlanID": _get(mp, "missionPlanID", "MissionPlanID"),
    }

# ────────── Receiver 클래스 ──────────
class MissionPlanReceiver_0301(
    IFusionReceive[MissionPlan], IsLocal, IsSingletone
):
    """0301 MissionPlan 메시지 수신 리시버 (timestamp + missionPlanID 전용)"""
    __namespace__ = "MissionPlanReceiver_0301"

    def Receive(self, data: MissionPlan, src):
        try:
            # 1) DB 저장
            received_db.set_received_0301(data)

            # 2) GUI 알림
            notify(
                "0301",
                json.dumps(_mission_plan_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0301] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
