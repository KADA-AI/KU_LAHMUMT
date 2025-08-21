# receive/message0501_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *               # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0501 import *                 # MissionStateInfo, CurrentMissionPlanID, etc.
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _mission_state_to_dict(state: MissionProgress) -> dict:
    body = {
        "timestamp": _get(state, "timestamp", "Timestamp"),
        "currentMissionPlanID": {
            "missionPlanID": _get(_get(state, "currentMissionPlanID", "CurrentMissionPlanID"),
                                  "missionPlanID", "MissionPlanID")
        },
        "inputMissionProgressStatus": {},
        "individualMissionProgressStatusList": [],
        "uncompletedPriorMissionIDList": [],
        "completedPriorMissionIDList": []
    }

    # ── inputMissionProgressStatus ──
    inp = _get(state, "inputMissionProgressStatus", "InputMissionProgressStatus")
    if inp:
        body["inputMissionProgressStatus"] = {
            "inputMissionPackageID":       _get(inp, "inputMissionPackageID", "InputMissionPackageID"),
            "currentInputMissionID":       _get(inp, "currentInputMissionID", "CurrentInputMissionID"),
            "currentInputMissionProgress": _get(inp, "currentInputMissionProgress", "CurrentInputMissionProgress"),
            "uncompletedInputMissionList": [
                { "inputMissionID": _get(u, "inputMissionID", "InputMissionID") }
                for u in (_get(inp, "uncompletedInputMissionList", "UncompletedInputMissionList") or [])
            ],
            "completedInputMissionList": [
                { "inputMissionID": _get(c, "inputMissionID", "InputMissionID") }
                for c in (_get(inp, "completedInputMissionList", "CompletedInputMissionList") or [])
            ]
        }

    # ── individualMissionProgressStatusList ──
    for imps in (_get(state, "individualMissionProgressStatusList", "IndividualMissionProgressStatusList") or []):
        imps_dict = {
            "aircraftID": _get(imps, "aircraftID", "AircraftID"),
            "uncompletedIndividualMissionList": [
                { "individualMissionID": _get(u, "individualMissionID", "IndividualMissionID") }
                for u in (_get(imps, "uncompletedIndividualMissionList", "UncompletedIndividualMissionList") or [])
            ],
            "completedIndividualMissionList": [
                { "individualMissionID": _get(c, "individualMissionID", "IndividualMissionID") }
                for c in (_get(imps, "completedIndividualMissionList", "CompletedIndividualMissionList") or [])
            ],
            "currentIndividualMission": {
                "individualMissionID": _get(_get(imps, "currentIndividualMission", "CurrentIndividualMission"),
                                            "individualMissionID", "IndividualMissionID")
            },
            "currentPathID": {
                "pathID": _get(_get(imps, "currentPathID", "CurrentPathID"), "pathID", "PathID")
            },
            "lastWaypointID": {
                "waypointID": _get(_get(imps, "lastWaypointID", "LastWaypointID"), "waypointID", "WaypointID")
            },
            "currentIndividualMissionProgress": _get(imps, "currentIndividualMissionProgress", "CurrentIndividualMissionProgress"),
            "currentBasicAction": {
                "flightMode":    _get(_get(imps, "currentBasicAction", "CurrentBasicAction"), "flightMode", "FlightMode"),
                "operationMode": _get(_get(imps, "currentBasicAction", "CurrentBasicAction"), "operationMode", "OperationMode")
            },
            "mandatoryCommandType": _get(imps, "mandatoryCommandType", "mandatoryCommandType"),
            "priorMissionID": _get(imps, "priorMissionID", "PriorMissionID")
        }
        body["individualMissionProgressStatusList"].append(imps_dict)

    # ── prior mission lists ──
    body["uncompletedPriorMissionIDList"] = [
        { "priorMissionID": _get(u, "priorMissionID", "PriorMissionID") }
        for u in (_get(state, "uncompletedPriorMissionIDList", "UncompletedPriorMissionIDList") or [])
    ]
    body["completedPriorMissionIDList"] = [
        { "priorMissionID": _get(c, "priorMissionID", "PriorMissionID") }
        for c in (_get(state, "completedPriorMissionIDList", "CompletedPriorMissionIDList") or [])
    ]
    return body

# ────────── Receiver ──────────
class MissionStateInfoReceiver_0501(
    IFusionReceive[MissionProgress], IsLocal, IsSingletone
):
    """0501 MissionStateInfo 메시지 수신 리시버"""
    __namespace__ = "MissionStateInfoReceiver_0501"

    def Receive(self, data: MissionProgress, src):
        try:
            received_db.set_received_0501(data)   # DB 저장
            notify(
                "0501",
                json.dumps(_mission_state_to_dict(data), ensure_ascii=False).encode()
            )
        except Exception:
            print("[ERROR][Receive-0501] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
