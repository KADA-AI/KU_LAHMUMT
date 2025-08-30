# modules/monitoring/push/message0501_push.py
import json
from System.Collections.Generic import List
from nFusion.Model.msg_0501 import *
from nFusion.Model.CommonType import *
from generator.message0501_generator import make_msg0501_body
from .push_utils import (
    cs_new,
    try_set,
    select_tx_fields,
    make_and_push_based_on_rules,
    TX_FIELD_WHITELIST,
)

MSG_ID = "0501"

# --- Dict to C# Object Converters ---

def _dict_to_current_individual_mission(data: dict) -> CurrentIndividualMission:
    obj = cs_new('CurrentIndividualMission', MSG_ID)
    if "individualMissionID" in data:
        try_set(obj, "individualMissionID", int(data["individualMissionID"]))
    return obj

def _dict_to_individual_mission_progress_status(data: dict) -> IndividualMissionProgressStatus:
    obj = cs_new('IndividualMissionProgressStatus', MSG_ID)
    if "aircraftID" in data:
        try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "currentIndividualMission" in data and isinstance(data["currentIndividualMission"], dict):
        try_set(obj, "currentIndividualMission", _dict_to_current_individual_mission(data["currentIndividualMission"]))
    if "currentIndividualMissionProgress" in data:
        try_set(obj, "currentIndividualMissionProgress", int(data["currentIndividualMissionProgress"]))
    return obj

def _dict_to_mission_progress(data: dict) -> MissionProgress:
    obj = cs_new('MissionProgress', MSG_ID)
    if "timestamp" in data:
        try_set(obj, "timestamp", int(data["timestamp"]))
    
    val_src = data.get("source", data.get("sourceModuleName", data.get("requestModuleName", "")))
    if val_src:
        if not try_set(obj, "source", str(val_src)):
            try_set(obj, "sourceModuleName", str(val_src))

    if "currentMissionPlanID" in data:
        try_set(obj, "currentMissionPlanID", int(data["currentMissionPlanID"]))
    if "currentInputMissionID" in data:
        try_set(obj, "currentInputMissionID", int(data["currentInputMissionID"]))
    if "individualMissionProgressStatusList" in data and isinstance(data["individualMissionProgressStatusList"], list):
        T = cs_new('IndividualMissionProgressStatus', MSG_ID)
        lst = List[T]()
        for item in data["individualMissionProgressStatusList"]:
            lst.Add(_dict_to_individual_mission_progress_status(item if isinstance(item, dict) else {}))
        try_set(obj, "individualMissionProgressStatusList", lst)
    return obj

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """Creates and pushes a message, returning a log string."""
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = select_tx_fields(body_dict, wl)
        
    msg = _dict_to_mission_progress(body_dict)
    node_messenger.Push(msg)
    
    log_line = (
        f"[{MSG_ID}] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[{MSG_ID}] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    """Creates a message with random/default data and pushes it."""
    return make_and_push_based_on_rules(
        MSG_ID,
        __file__,
        make_msg0501_body,
        make_and_push,
        node_messenger,
    )