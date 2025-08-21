# # receive/message0203_receiver.py
# # ──────────────────────────────────────────────────────────────
# from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
# from nFusion.Model.msg_0203 import *              # FlightReferenceInfo, TakeOverInfo, HandOverInfo, etc.
# from .database import received_db
# from receive_center import notify
# import json, traceback, sys

# # ────────── 대/소문자 안전 접근 헬퍼 ──────────
# _get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# # ────────── CLR → dict 변환 ──────────
# def _flight_reference_info_to_dict(info: FlightReferenceInfo) -> dict:
#     def coord2d(ct: Coordinate) -> dict:
#         return {
#             "latitude":  _get(ct, "latitude",  "Latitude"),
#             "longitude": _get(ct, "longitude", "Longitude"),
#             "altitude":  _get(ct, "altitude",  "Altitude")
#         }

#     body = {
#         "timestamp":                 _get(info, "timestamp", "Timestamp"),
#         "missionReferencePackageID": _get(info, "missionReferencePackageID", "MissionReferencePackageID"),
#         "inputTimestamp":            _get(info, "inputTimestamp", "InputTimestamp"),
#         "takeOverInfoList":          [],
#         "handOverInfoList":          [],
#         "rtbCoordinateList":         [],
#         "flightAreaList":            [],
#         "prohibitedAreaList":        []
#     }

#     # TakeOverInfoList
#     for t in (_get(info, "takeOverInfoList", "TakeOverInfoList") or []):
#         body["takeOverInfoList"].append({
#             "aircraftID": _get(t, "aircraftID", "AircraftID"),
#             "coordinate": coord2d(_get(t, "coordinate", "Coordinate"))
#         })

#     # HandOverInfoList
#     for h in (_get(info, "handOverInfoList", "HandOverInfoList") or []):
#         body["handOverInfoList"].append({
#             "aircraftID": _get(h, "aircraftID", "AircraftID"),
#             "coordinate": coord2d(_get(h, "coordinate", "Coordinate"))
#         })

#     # RTBCoordinateList
#     for r in (_get(info, "rtbCoordinateList", "RtbCoordinateList") or []):
#         body["rtbCoordinateList"].append(coord2d(r))

#     # FlightAreaList
#     for fa in (_get(info, "flightAreaList", "FlightAreaList") or []):
#         alt_limits = _get(fa, "altitudeLimits", "AltitudeLimits")
#         fa_dict = {
#             "flightAreaID": _get(fa, "flightAreaID", "FlightAreaID"),
#             "areaLatLonList": [],
#             "altitudeLimits": {
#                 "lowerLimit": _get(alt_limits, "lowerLimit", "LowerLimit"),
#                 "upperLimit": _get(alt_limits, "upperLimit", "UpperLimit")
#             }
#         }
#         for al in (_get(fa, "areaLatLonList", "AreaLatLonList") or []):
#             fa_dict["areaLatLonList"].append({
#                 "latitude":  _get(al, "latitude", "Latitude"),
#                 "longitude": _get(al, "longitude", "Longitude")
#             })
#         body["flightAreaList"].append(fa_dict)

#     # ProhibitedAreaList
#     for pa in (_get(info, "prohibitedAreaList", "ProhibitedAreaList") or []):
#         alt_limits = _get(pa, "altitudeLimits", "AltitudeLimits")
#         pa_dict = {
#             "prohibitedAreaID": _get(pa, "prohibitedAreaID", "ProhibitedAreaID"),
#             "areaLatLonList":   [],
#             "altitudeLimits": {
#                 "lowerLimit": _get(alt_limits, "lowerLimit", "LowerLimit"),
#                 "upperLimit": _get(alt_limits, "upperLimit", "UpperLimit")
#             }
#         }
#         for pal in (_get(pa, "areaLatLonList", "AreaLatLonList") or []):
#             pa_dict["areaLatLonList"].append({
#                 "latitude":  _get(pal, "latitude", "Latitude"),
#                 "longitude": _get(pal, "longitude", "Longitude")
#             })
#         body["prohibitedAreaList"].append(pa_dict)

#     return body

# # ────────── Receiver 클래스 ──────────
# class FlightReferenceInfoReceiver_0203(
#     IFusionReceive[FlightReferenceInfo], IsLocal, IsSingletone
# ):
#     """0203 FlightReferenceInfo 메시지 수신 리시버"""
#     __namespace__ = "FlightReferenceInfoReceiver_0203"

#     def Receive(self, data: FlightReferenceInfo, src):
#         try:
#             # 1) DB 저장
#             received_db.set_received_0203(data)

#             # 2) GUI에 JSON 바디 형태로 전달
#             notify(
#                 "0203",
#                 json.dumps(
#                     _flight_reference_info_to_dict(data),
#                     ensure_ascii=False
#                 ).encode()
#             )

#         except Exception:
#             print("[ERROR][Receive-0203] traceback ↓↓↓")
#             traceback.print_exc(file=sys.stderr)


# receive/message0203_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0203 import *              # FlightReferenceInfo
from .database import received_db                 # DB 저장 모듈
from receive_center import notify                 # GUI 알림 함수

import json
import traceback
import sys
import os

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── FlightReferenceInfo → dict (필수 필드만) ──────────
def _flight_reference_info_to_dict(info: FlightReferenceInfo) -> dict:
    return {
        "timestamp":                 _get(info, "timestamp", "Timestamp"),
        "missionReferencePackageID": _get(info, "missionReferencePackageID", "MissionReferencePackageID"),
    }

# ★ MissionReferenceInfo JSON 저장 경로 ------------------------
PLAN_DIR = r"C:\Users\LAHMUMT_2\Desktop\nFusion\missionPlanner\plannedMission\MissionReferenceInfo"

# ────────── Receiver 클래스 ──────────
class FlightReferenceInfoReceiver_0203(
    IFusionReceive[FlightReferenceInfo], IsLocal, IsSingletone
):
    """0203 FlightReferenceInfo 메시지 수신 리시버 (timestamp + missionReferencePackageID 전용)"""
    __namespace__ = "FlightReferenceInfoReceiver_0203"

    def Receive(self, data: FlightReferenceInfo, src):
        try:
            # 1) DB 저장
            received_db.set_received_0203(data)

            # 2) DB 파일 로드
            body_min = _flight_reference_info_to_dict(data)
            pkg_id   = body_min["missionReferencePackageID"]
            json_path = os.path.join(PLAN_DIR, f"{pkg_id}.json")

            # ── 로그: DB 참조! ─────────────────────────────
            print(f"[0203] DB 참조! ({json_path})")

            if os.path.exists(json_path):
                with open(json_path, "r", encoding="utf-8") as f:
                    file_data = json.load(f)

                # 3) GUI 알림: 파일 내용 전체 전달
                notify(
                    "0203",
                    json.dumps(file_data, ensure_ascii=False).encode()
                )
            else:
                # 파일이 없으면 최소 바디 + 오류 메시지 전달
                notify(
                    "0203",
                    json.dumps({"error": "DB 파일 없음", **body_min}, ensure_ascii=False).encode()
                )

        except Exception:
            print("[ERROR][Receive-0203] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
