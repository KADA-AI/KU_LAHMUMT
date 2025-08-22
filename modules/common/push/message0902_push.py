# push/message0902_push.py
# -*- coding: utf-8 -*-
from System.Collections.Generic import List

# msg_0902 쪽 타입
from nFusion.Model.msg_0902 import (
    ReplanRequest, ReplanRequestTime,
    InputMissionID, IndividualMissionID,
)

# ✅ 공통타입은 CommonType 네임스페이스!
from nFusion.Model.CommonType import (
    PriorMission, PendingOption,
    CoordinateOrientation, TargetOrientation
)

from generator.message0902_generator import make_msg0902_body
import json

def _dict_to_obj(body: dict) -> ReplanRequest:
    req = ReplanRequest()

    # ------- Top-level -------
    req.timestamp = int(body["timestamp"])
    # source(string) 소/대문자 둘 다 대비
    if hasattr(req, "source"):
        req.source = str(body.get("source", ""))
    elif hasattr(req, "Source"):
        req.Source = str(body.get("source", ""))

    req.replanLevel  = int(body["replanLevel"])
    req.replanReason = str(body["replanReason"])

    # ------- ReplanRequestTime -------
    rrt = ReplanRequestTime()
    rrt.replanRequestTimestamp = int(body["replanRequestTime"]["replanRequestTimestamp"])
    req.replanRequestTime = rrt

    # ------- inputMissionIDList (List<InputMissionID>) -------
    im_list = List[InputMissionID]()
    for itm in body.get("inputMissionIDList", []):
        im = InputMissionID()
        im.inputMissionID = int(itm["inputMissionID"])
        im_list.Add(im)
    if hasattr(req, "inputMissionIDList"): req.inputMissionIDList = im_list
    elif hasattr(req, "InputMissionIDList"): req.InputMissionIDList = im_list

    # 단일 inputMissionID
    if "inputMissionID" in body:
        if hasattr(req, "inputMissionID"): req.inputMissionID = int(body["inputMissionID"])
        elif hasattr(req, "InputMissionID"): req.InputMissionID = int(body["inputMissionID"])

    # ------- individualMissionIDList (List<IndividualMissionID>) -------
    ind_list = List[IndividualMissionID]()
    for itm in body.get("individualMissionIDList", []):
        ind = IndividualMissionID()
        ind.individualMissionID = int(itm["individualMissionID"])
        ind_list.Add(ind)
    if hasattr(req, "individualMissionIDList"): req.individualMissionIDList = ind_list
    elif hasattr(req, "IndividualMissionIDList"): req.IndividualMissionIDList = ind_list

    # 단일 individualMissionID
    if "individualMissionID" in body:
        if hasattr(req, "individualMissionID"): req.individualMissionID = int(body["individualMissionID"])
        elif hasattr(req, "IndividualMissionID"): req.IndividualMissionID = int(body["individualMissionID"])

    # ------- priorMissionList (List<CommonType.PriorMission>) -------
    pm_list = List[PriorMission]()
    for itm in body.get("priorMissionList", []):
        pm = PriorMission()
        pm.priorMissionID = int(itm.get("priorMissionID", 0))
        # optional: missionType, coordinate/target
        if "missionType" in itm:
            pm.missionType = int(itm["missionType"])
        if "coordinateOrientation" in itm:
            co = itm["coordinateOrientation"]
            coord = CoordinateOrientation()
            coord.latitude  = float(co.get("latitude", 0.0))
            coord.longitude = float(co.get("longitude", 0.0))
            coord.altitude  = int(co.get("altitude", 0))
            pm.coordinateOrientation = coord
        if "targetOrientation" in itm:
            to = itm["targetOrientation"]
            tgt = TargetOrientation()
            tgt.targetID = int(to.get("targetID", 0))
            pm.targetOrientation = tgt
        pm_list.Add(pm)
    if hasattr(req, "priorMissionList"): req.priorMissionList = pm_list
    elif hasattr(req, "PriorMissionList"): req.PriorMissionList = pm_list

    # ------- pendingOptionList (List<CommonType.PendingOption>) -------
    po_list = List[PendingOption]()
    for itm in body.get("pendingOptionList", []):
        po = PendingOption()
        if hasattr(po, "optionID"): po.optionID = int(itm["optionID"])
        elif hasattr(po, "OptionID"): po.OptionID = int(itm["optionID"])
        if hasattr(po, "optionName"): po.optionName = str(itm["optionName"])
        elif hasattr(po, "OptionName"): po.OptionName = str(itm["optionName"])
        if hasattr(po, "missionPlanID"): po.missionPlanID = int(itm["missionPlanID"])
        elif hasattr(po, "MissionPlanID"): po.MissionPlanID = int(itm["missionPlanID"])
        po_list.Add(po)
    if hasattr(req, "pendingOptionList"): req.pendingOptionList = po_list
    elif hasattr(req, "PendingOptionList"): req.PendingOptionList = po_list

    return req

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0902] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0902] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0902_body(), node_messenger)
