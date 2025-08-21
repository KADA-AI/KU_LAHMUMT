# from nFusion.Model.msg_0203 import *
# from generator.message0203_generator import make_msg0203_body  # generator에서 메시지 바디 가져오기
# from System.Collections.Generic import List




# def _dict_to_obj(body_dict: dict):
#     """
#     dict(JSON, 소문자 카멜) → FlightReferenceInfo(C# 객체)
#     """
#     info = FlightReferenceInfo()
#     info.timestamp                = body_dict["timestamp"]
#     info.missionReferencePackageID = body_dict["missionReferencePackageID"]
#     info.inputTimestamp           = body_dict["inputTimestamp"]

#     # ───────── TakeOverInfoList ─────────
#     to_list = List[TakeOverInfo]()
#     for t in body_dict["takeOverInfoList"]:
#         to_obj = TakeOverInfo()
#         to_obj.aircraftID = t["aircraftID"]

#         coord_t = Coordinate()
#         coord_t.latitude  = t["coordinate"]["latitude"]
#         coord_t.longitude = t["coordinate"]["longitude"]
#         coord_t.altitude  = t["coordinate"]["altitude"]
#         to_obj.coordinate = coord_t

#         to_list.Add(to_obj)
#     info.takeOverInfoList = to_list

#     # ───────── HandOverInfoList ─────────
#     ho_list = List[HandOverInfo]()
#     for h in body_dict["handOverInfoList"]:
#         ho_obj = HandOverInfo()
#         ho_obj.aircraftID = h["aircraftID"]

#         coord_h = Coordinate()
#         coord_h.latitude  = h["coordinate"]["latitude"]
#         coord_h.longitude = h["coordinate"]["longitude"]
#         coord_h.altitude  = h["coordinate"]["altitude"]
#         ho_obj.coordinate = coord_h

#         ho_list.Add(ho_obj)
#     info.handOverInfoList = ho_list

#     # ───────── RTBCoordinateList ─────────
#     rtb_list = List[RTBCoordinate]()
#     for r in body_dict["rtbCoordinateList"]:
#         r_obj = RTBCoordinate()
#         r_obj.latitude  = r["latitude"]
#         r_obj.longitude = r["longitude"]
#         r_obj.altitude  = r["altitude"]
#         rtb_list.Add(r_obj)
#     info.rtbCoordinateList = rtb_list

#     # ───────── FlightAreaList ─────────
#     fa_list = List[FlightArea]()
#     for f in body_dict["flightAreaList"]:
#         fa_obj = FlightArea()
#         fa_obj.flightAreaID = f["flightAreaID"]

#         # AreaLatLonList
#         al_list = List[AreaLatLon]()
#         for al in f["areaLatLonList"]:
#             al_obj = AreaLatLon()
#             al_obj.latitude  = al["latitude"]
#             al_obj.longitude = al["longitude"]
#             al_list.Add(al_obj)
#         fa_obj.areaLatLonList = al_list

#         # AltitudeLimits
#         alims = AltitudeLimits()
#         alims.lowerLimit = f["altitudeLimits"]["lowerLimit"]
#         alims.upperLimit = f["altitudeLimits"]["upperLimit"]
#         fa_obj.altitudeLimits = alims

#         fa_list.Add(fa_obj)
#     info.flightAreaList = fa_list

#     # ───────── ProhibitedAreaList ─────────
#     pa_list = List[ProhibitedArea]()
#     for p in body_dict["prohibitedAreaList"]:
#         pa_obj = ProhibitedArea()
#         pa_obj.prohibitedAreaID = p["prohibitedAreaID"]

#         # AreaLatLonList
#         pal_list = List[AreaLatLon]()
#         for pal in p["areaLatLonList"]:
#             pal_obj = AreaLatLon()
#             pal_obj.latitude  = pal["latitude"]
#             pal_obj.longitude = pal["longitude"]
#             pal_list.Add(pal_obj)
#         pa_obj.areaLatLonList = pal_list

#         # AltitudeLimits
#         palims = AltitudeLimits()
#         palims.lowerLimit = p["altitudeLimits"]["lowerLimit"]
#         palims.upperLimit = p["altitudeLimits"]["upperLimit"]
#         pa_obj.altitudeLimits = palims

#         pa_list.Add(pa_obj)
#     info.prohibitedAreaList = pa_list

#     return info


# import json 
# def make_and_push(body_dict: dict, node_messenger) -> None:
#     msg = _dict_to_obj(body_dict)
#     #print(f"Message pushed: {msg}")
#     node_messenger.Push(msg)
#     # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
#     log_line = (
#         f"[0203] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
#         f"[0203] PUSH 완료"

#     )
#     ##print(log_line)
#     return log_line.encode()

# def make_random_and_push(node_messenger) -> None:

#     return make_and_push(make_msg0203_body(), node_messenger)


# push/message0203_push.py
# ─────────────────────────────────────────────────────────────
# 0203 FlightReferenceInfo 발신 스텁
#   • 지정 폴더의 *.json → 파일명 숫자 = missionReferencePackageID
#   • {timestamp, missionReferencePackageID} 두 필드만 세팅하여 Push
#   • timestamp: 2000-01-01 UTC 기준 ms
# ─────────────────────────────────────────────────────────────
import os, glob, json
from datetime import datetime, timezone
from nFusion.Model.msg_0203 import *   # FlightReferenceInfo

# 1) ★ FlightReferenceInfo JSON 위치 (절대경로) -----------------
#    필요에 따라 경로만 수정하면 됩니다.
PLAN_DIR = r"C:\Users\LAHMUMT_2\Desktop\nFusion\missionPlanner\plannedMission\MissionReferenceInfo"

# 2) 2000-01-01 UTC 기준 ms 계산 -------------------------------
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms     = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000
)

# ─────────────────────────────────────────────────────────────
def _dict_to_obj(body_dict: dict) -> FlightReferenceInfo:
    """dict → FlightReferenceInfo(C#) – timestamp / missionReferencePackageID 만 설정"""
    info = FlightReferenceInfo()
    info.timestamp                = body_dict["timestamp"]
    info.missionReferencePackageID = body_dict["missionReferencePackageID"]
    # inputTimestamp·takeOverInfoList … 등은 기본값 유지
    return info


def _list_package_ids() -> list[int]:
    """
    PLAN_DIR 의 *.json 파일명 중 숫자만 → missionReferencePackageID 목록
    """
    ids: list[int] = []
    for path in glob.glob(os.path.join(PLAN_DIR, "*.json")):
        stem = os.path.splitext(os.path.basename(path))[0]
        if stem.isdigit():          # ex) "1003", "78001"
            ids.append(int(stem))
    return sorted(ids)


def make_and_push(body_dict: dict, node_messenger) -> bytes | None:
    """dict → C# 객체 변환·Push, GUI 로그 bytes 반환"""
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0203] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0203] PUSH 완료"
    )
    return log_line.encode()


def make_random_and_push(node_messenger) -> bytes | None:
    """
    • PLAN_DIR 의 JSON 파일명 → missionReferencePackageID 로 사용
    • {timestamp, missionReferencePackageID} 메시지를 순차 Push
    """
    logs: list[bytes] = []
    for pid in _list_package_ids():
        body = {
            "timestamp":                 _now_ms(),
            "missionReferencePackageID": pid,
        }
        log = make_and_push(body, node_messenger)
        if log:
            logs.append(log)

    return b"\n".join(logs) if logs else None
# ─────────────────────────────────────────────────────────────
