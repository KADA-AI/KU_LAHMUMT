# receive/message0202_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0202 import *
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
def _get(obj, *names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

# ────────── CLR → dict 변환 ──────────
def _prior_mission_info_to_dict(info: PriorMissionInfo) -> dict:
    def coord2d(ct: CoordinateOrientation) -> dict:
        return {
            "latitude":  _get(ct, "latitude",  "Latitude"),
            "longitude": _get(ct, "longitude", "Longitude"),
            "altitude":  _get(ct, "altitude",  "Altitude"),  # int로 표준화
        }

    def tgt2d(to: TargetOrientation) -> dict:
        return { "targetID": _get(to, "targetID", "TargetID") }

    body = {
        "timestamp":        _get(info, "timestamp", "Timestamp"),
        "priorMissionList": []
    }

    seq = _get(info, "priorMissionList", "PriorMissionList") or []
    for pm in seq:
        mtype = _get(pm, "missionType", "MissionType")
        item = {
            "priorMissionID": _get(pm, "priorMissionID", "PriorMissionID"),
            "missionType":    mtype,
        }
        if mtype == 1:
            co = _get(pm, "coordinateOrientation", "CoordinateOrientation") or CoordinateOrientation()
            item["coordinateOrientation"] = coord2d(co)
        else:
            to = _get(pm, "targetOrientation", "TargetOrientation") or TargetOrientation()
            item["targetOrientation"] = tgt2d(to)

        body["priorMissionList"].append(item)
    return body

# ────────── Receiver 클래스 ──────────
class PriorMissionInfoReceiver_0202(
    IFusionReceive[PriorMissionInfo], IsLocal, IsSingletone
):
    """0202 PriorMissionInfo 메시지 수신 리시버"""
    __namespace__ = "PriorMissionInfoReceiver_0202"

    def Receive(self, data: PriorMissionInfo, src):
        try:
            # 1) DB 저장
            received_db.set_received_0202(data)

            # 2) GUI 알림(JSON)
            notify(
                "0202",
                json.dumps(_prior_mission_info_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore")
            )

        except Exception:
            print("[ERROR][Receive-0202] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
