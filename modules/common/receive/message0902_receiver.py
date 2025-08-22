# receive/message0902_receiver.py
# -*- coding: utf-8 -*-
from dll_files.nFusionImports import *               # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0902 import (
    ReplanRequest, ReplanRequestTime,
    InputMissionID, IndividualMissionID,
)
# ✅ 공통타입은 CommonType
from nFusion.Model.CommonType import (
    PriorMission, PendingOption,
    CoordinateOrientation, TargetOrientation
)

from .database import received_db
from receive_center import notify
import json, traceback, sys

def _get(obj, *names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

def _replan_request_to_dict(req: ReplanRequest) -> dict:
    source_val = _get(req, "source", "Source")
    rrt = _get(req, "replanRequestTime", "ReplanRequestTime")
    rrt_ts = _get(rrt, "replanRequestTimestamp", "ReplanRequestTimestamp") if rrt else None

    im_seq   = _get(req, "inputMissionIDList", "InputMissionIDList") or []
    ind_seq  = _get(req, "individualMissionIDList", "IndividualMissionIDList") or []
    prior_seq= _get(req, "priorMissionList", "PriorMissionList") or []
    pend_seq = _get(req, "pendingOptionList", "PendingOptionList") or []

    body = {
        "timestamp":          _get(req, "timestamp", "Timestamp"),
        "source":             source_val,
        "replanRequestTime":  {"replanRequestTimestamp": rrt_ts},
        "replanLevel":        _get(req, "replanLevel", "ReplanLevel"),

        "inputMissionIDList": [{"inputMissionID": _get(im, "inputMissionID", "InputMissionID")} for im in im_seq],
        "inputMissionID":     _get(req, "inputMissionID", "InputMissionID"),

        "individualMissionIDList": [{"individualMissionID": _get(ind, "individualMissionID", "IndividualMissionID")} for ind in ind_seq],
        "individualMissionID":    _get(req, "individualMissionID", "IndividualMissionID"),

        "priorMissionList": [
            {
                "priorMissionID": _get(pm, "priorMissionID", "PriorMissionID"),
                "missionType":    _get(pm, "missionType", "MissionType"),
                "coordinateOrientation": (
                    lambda co: {
                        "latitude":  _get(co, "latitude",  "Latitude"),
                        "longitude": _get(co, "longitude", "Longitude"),
                        "altitude":  _get(co, "altitude",  "Altitude"),
                    } if co else None
                )(_get(pm, "coordinateOrientation", "CoordinateOrientation")),
                "targetOrientation": (
                    lambda to: {
                        "targetID": _get(to, "targetID", "TargetID")
                    } if to else None
                )(_get(pm, "targetOrientation", "TargetOrientation")),
            } for pm in prior_seq
        ],

        "replanReason":       _get(req, "replanReason", "ReplanReason"),

        "pendingOptionList":  [
            {
                "optionID":      _get(po, "optionID",      "OptionID"),
                "optionName":    _get(po, "optionName",    "OptionName"),
                "missionPlanID": _get(po, "missionPlanID", "MissionPlanID"),
            } for po in pend_seq
        ],
    }
    return body

class ReplanRequestReceiver_0902(
    IFusionReceive[ReplanRequest], IsLocal, IsSingletone
):
    """0902 ReplanRequest 메시지 수신 리시버"""
    __namespace__ = "ReplanRequestReceiver_0902"

    def Receive(self, data: ReplanRequest, src):
        try:
            received_db.set_received_0902(data)
            notify("0902", json.dumps(_replan_request_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore"))
        except Exception:
            print("[ERROR][Receive-0902] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
