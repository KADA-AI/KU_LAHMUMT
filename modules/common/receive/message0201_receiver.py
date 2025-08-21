# # receive/message0201_receiver.py
# # ──────────────────────────────────────────────────────────────
# from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
# from nFusion.Model.msg_0201 import *              # InputMissionPlan, AvailableAircraft, MissionDetail, etc.
# from .database import received_db
# from receive_center import notify
# import json, traceback, sys

# # ────────── 대/소문자 안전 접근 헬퍼 ──────────
# _get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# # ────────── CLR → dict 변환 ──────────
# def _input_mission_plan_to_dict(plan: InputMissionPlan) -> dict:
#     def c2d(ct):
#         return {
#             "latitude":  _get(ct, "latitude",  "Latitude"),
#             "longitude": _get(ct, "longitude", "Longitude"),
#             "altitude":  _get(ct, "altitude",  "Altitude")
#         }

#     body = {
#         "timestamp":               _get(plan, "timestamp",              "Timestamp"),
#         "inputMissionPackageID":   _get(plan, "inputMissionPackageID",  "InputMissionPackageID"),
#         "inputMissionPackageType": _get(plan, "inputMissionPackageType", "InputMissionPackageType"),
#         "mainSensor":              _get(plan, "mainSensor",             "MainSensor"),
#         "availableAircraftList":   [],
#         "inputMissionList":        []
#     }

#     # AvailableAircraftList
#     for aa in _get(plan, "availableAircraftList", "AvailableAircraftList") or []:
#         body["availableAircraftList"].append({
#             "aircraftID": _get(aa, "aircraftID", "AircraftID")
#         })

#     # InputMissionList
#     for im in _get(plan, "inputMissionList", "InputMissionList") or []:
#         im_dict = {
#             "inputMissionID":   _get(im, "inputMissionID",   "InputMissionID"),
#             "inputMissionType": _get(im, "inputMissionType", "InputMissionType"),
#             "isDone":           _get(im, "isDone",           "IsDone"),
#             "missionDetail": {
#                 "coordinateList": [],
#                 "lineList":       [],
#                 "areaList":       []
#             }
#         }
#         md = _get(im, "missionDetail", "MissionDetail")

#         # coordinateList
#         for c in _get(md, "coordinateList", "CoordinateList") or []:
#             im_dict["missionDetail"]["coordinateList"].append(c2d(c))

#         # lineList
#         for ln in _get(md, "lineList", "LineList") or []:
#             ln_dict = {
#                 "width":          _get(ln, "width", "Width"),
#                 "coordinateList": []
#             }
#             for lc in _get(ln, "coordinateList", "CoordinateList") or []:
#                 ln_dict["coordinateList"].append(c2d(lc))
#             im_dict["missionDetail"]["lineList"].append(ln_dict)

#         # areaList
#         for ar in _get(md, "areaList", "AreaList") or []:
#             ar_dict = {
#                 "isHole":         _get(ar, "isHole", "IsHole"),
#                 "coordinateList": []
#             }
#             for ac in _get(ar, "coordinateList", "CoordinateList") or []:
#                 ar_dict["coordinateList"].append(c2d(ac))
#             im_dict["missionDetail"]["areaList"].append(ar_dict)

#         body["inputMissionList"].append(im_dict)

#     return body

# # ────────── Receiver 클래스 ──────────
# class InputMissionPlanReceiver_0201(
#     IFusionReceive[InputMissionPlan], IsLocal, IsSingletone
# ):
#     """0201 InputMissionPlan 메시지 수신 리시버"""
#     __namespace__ = "InputMissionPlanReceiver_0201"

#     def Receive(self, data: InputMissionPlan, src):
#         try:
#             # 1) DB 저장
#             received_db.set_received_0201(data)

#             # 2) GUI에 JSON 바디 형태로 전달
#             notify(
#                 "0201",
#                 json.dumps(_input_mission_plan_to_dict(data), ensure_ascii=False).encode()
#             )

#         except Exception:
#             print("[ERROR][Receive-0201] traceback ↓↓↓")
#             traceback.print_exc(file=sys.stderr)


# receive/message0201_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0201 import *              # InputMissionPlan
from .database import received_db                 # DB 저장 모듈
from receive_center import notify                 # GUI 알림 함수

import json
import traceback
import sys
import os

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── InputMissionPlan → dict (필수 필드만) ──────────
def _input_mission_plan_to_dict(plan: InputMissionPlan) -> dict:
    return {
        "timestamp":              _get(plan, "timestamp",            "Timestamp"),
        "inputMissionPackageID":  _get(plan, "inputMissionPackageID","InputMissionPackageID"),
    }

# ★ DB JSON 저장 경로 (패키지 ID → {id}.json) ------------------
PLAN_DIR = r"C:\Users\LAHMUMT_2\Desktop\nFusion\missionPlanner\plannedMission\InputMissionPlan"

# ────────── Receiver 클래스 ──────────
class InputMissionPlanReceiver_0201(
    IFusionReceive[InputMissionPlan], IsLocal, IsSingletone
):
    """0201 InputMissionPlan 메시지 수신 리시버 (timestamp + inputMissionPackageID 전용)"""
    __namespace__ = "InputMissionPlanReceiver_0201"

    def Receive(self, data: InputMissionPlan, src):
        try:
            # 1) DB 저장
            received_db.set_received_0201(data)

            # 2) 파일 경로 결정 & 로드
            body_min = _input_mission_plan_to_dict(data)
            pkg_id   = body_min["inputMissionPackageID"]
            json_path = os.path.join(PLAN_DIR, f"{pkg_id}.json")

            # ── 로그: DB 참조! ─────────────────────────────
            print(f"[0201] DB 참조! ({json_path})")

            if os.path.exists(json_path):
                with open(json_path, "r", encoding="utf-8") as f:
                    file_data = json.load(f)

                # 3) GUI 알림: 파일 내용 그대로 출력
                notify(
                    "0201",
                    json.dumps(file_data, ensure_ascii=False).encode()
                )
            else:
                # 파일이 없으면 최소 바디만 알림
                notify(
                    "0201",
                    json.dumps({"error": "DB 파일 없음", **body_min}, ensure_ascii=False).encode()
                )

        except Exception:
            print("[ERROR][Receive-0201] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
