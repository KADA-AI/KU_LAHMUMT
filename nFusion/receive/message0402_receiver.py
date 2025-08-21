# receive/message0402_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0402 import *              # SituationAwarenessInfo, ROIInfo, Target, etc.
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _situation_awareness_info_to_dict(info: SituationAwarenessInfo) -> dict:
    def coord2d(ct: Coordinate) -> dict:
        return {
            "latitude":  _get(ct, "latitude",  "Latitude"),
            "longitude": _get(ct, "longitude", "Longitude"),
            "altitude":  _get(ct, "altitude",  "Altitude")
        }

    roi = _get(info, "roiInfo", "RoiInfo")
    body = {
        "timestamp": _get(info, "timestamp", "Timestamp"),
        "roiInfo": {
            "aircraftID": _get(roi, "aircraftID", "AircraftID"),
            "coordinate": coord2d(_get(roi, "coordinate", "Coordinate")),
            "fov":        _get(roi, "fov", "Fov")
        },
        "targetList": []
    }

    for t in _get(info, "targetList", "TargetList") or []:
        watcher = _get(t, "watcher", "Watcher")
        target_dict = {
            "targetID":    _get(t, "targetID",    "TargetID"),
            "targetType":  _get(t, "targetType",  "TargetType"),
            "coordinate":  coord2d(_get(t, "coordinate", "Coordinate")),
            "watcher": {
                "aircraftID": _get(watcher, "aircraftID", "AircraftID")
            },
            "targetInFrame": _get(t, "targetInFrame", "TargetInFrame"),
            "isDestroyed":   _get(t, "isDestroyed",   "IsDestroyed"),
            "threat":        _get(t, "threat",        "Threat")
        }
        body["targetList"].append(target_dict)

    return body

# ────────── Receiver 클래스 ──────────
class SituationAwarenessInfoReceiver_0402(
    IFusionReceive[SituationAwarenessInfo], IsLocal, IsSingletone
):
    """0402 SituationAwarenessInfo 메시지 수신 리시버"""
    __namespace__ = "SituationAwarenessInfoReceiver_0402"

    def Receive(self, data: SituationAwarenessInfo, src):
        try:
            # 1) DB 저장
            received_db.set_received_0402(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0402",
                json.dumps(
                    _situation_awareness_info_to_dict(data),
                    ensure_ascii=False
                ).encode()
            )

        except Exception:
            print("[ERROR][Receive-0402] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
