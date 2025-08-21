from System.Collections.Generic import List                   # C# List
from nFusion.Model.msg_0902 import *            # msg_0902 타입 import
from generator.message0902_generator import make_msg0902_body  # 메시지 바디 생성기
import json


def _dict_to_obj(body: dict):
    req = ReplanRequest()

    # ── 최상위 필드 ───────────────────────────────
    req.timestamp    = body["timestamp"]
    req.replanLevel  = body["replanLevel"]
    req.replanReason = body["replanReason"]

    # ── ReplanRequestTime ────────────────────────
    rrt = ReplanRequestTime()
    rrt.replanRequestTimestamp = body["replanRequestTime"]["replanRequestTimestamp"]
    req.replanRequestTime = rrt

    # ── InputMissionIDList ───────────────────────
    in_list = List[InputMissionID]()
    for itm in body["inputMissionIDList"]:
        im = InputMissionID()
        im.inputMissionID = itm["inputMissionID"]
        in_list.Add(im)
    req.InputMissionIDList = in_list

    # ── IndividualMissionIDList ──────────────────
    ind_list = List[IndividualMissionID]()
    for itm in body["individualMissionIDList"]:
        ind = IndividualMissionID()
        ind.individualMissionID = itm["individualMissionID"]
        ind_list.Add(ind)
    req.individualMissionIDList = ind_list

    # ── PriorMissionList ─────────────────────────
    prior_list = List[PriorMission]()
    for itm in body["priorMissionList"]:
        pm = PriorMission()
        pm.priorMissionID = itm["priorMissionID"]
        prior_list.Add(pm)
    req.priorMissionList = prior_list

    # ── OptionList ───────────────────────────────
    opt_list = List[Option]()
    for opt in body["optionList"]:
        op = Option()
        op.optionID      = opt["optionID"]
        op.optionName    = opt["optionName"]
        op.missionPlanID = opt["missionPlanID"]
        opt_list.Add(op)
    req.OptionList = opt_list

    return req

def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)

    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0902] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0902] PUSH 완료"
    )
    return log_line.encode()                 # ← push_center → _mark_sent 로 전달

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0902_body() ,node_messenger)
