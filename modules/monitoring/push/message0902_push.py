# modules/monitoring/push/message0902_push.py
from System.Collections.Generic import List
from nFusion.Model.msg_0902 import *
from nFusion.Model.CommonType import *
from generator.message0902_generator import make_msg0902_body
from .push_utils import (
    cs_new,
    try_set,
    select_tx_fields,
    make_and_push_based_on_rules,
    TX_FIELD_WHITELIST,
)

MSG_ID = "0902"

# --- Dict to C# Object Converters ---


def _dict_to_replan_request_time(data: dict) -> ReplanRequestTime:
    obj = cs_new("ReplanRequestTime", MSG_ID)
    if "replanRequestTimestamp" in data:
        try_set(obj, "replanRequestTimestamp", int(data["replanRequestTimestamp"]))
    return obj


def _dict_to_input_mission_id(data: dict) -> InputMissionID:
    obj = cs_new("InputMissionID", MSG_ID)
    if "inputMissionID" in data:
        try_set(obj, "inputMissionID", int(data["inputMissionID"]))
    return obj


def _dict_to_individual_mission_id(data: dict) -> IndividualMissionID:
    obj = cs_new("IndividualMissionID", MSG_ID)
    if "individualMissionID" in data:
        try_set(obj, "individualMissionID", int(data["individualMissionID"]))
    return obj


def _dict_to_coordinate(data: dict) -> Coordinate:
    obj = cs_new("Coordinate", MSG_ID)
    if "latitude" in data:
        try_set(obj, "latitude", float(data["latitude"]))
    if "longitude" in data:
        try_set(obj, "longitude", float(data["longitude"]))
    if "altitude" in data:
        try_set(obj, "altitude", int(data["altitude"]))
    return obj


def _dict_to_coordinate_orientation(data: dict) -> CoordinateOrientation:
    obj = cs_new("CoordinateOrientation", MSG_ID)
    if "coordinate" in data and isinstance(data["coordinate"], dict):
        try_set(obj, "coordinate", _dict_to_coordinate(data["coordinate"]))
    return obj


def _dict_to_target_orientation(data: dict) -> TargetOrientation:
    obj = cs_new("TargetOrientation", MSG_ID)
    if "targetID" in data:
        try_set(obj, "targetID", int(data["targetID"]))
    return obj


def _dict_to_prior_mission(data: dict) -> PriorMission:
    obj = cs_new("PriorMission", MSG_ID)
    if "priorMissionID" in data:
        try_set(obj, "priorMissionID", int(data["priorMissionID"]))
    if "missionType" in data:
        try_set(obj, "missionType", int(data["missionType"]))
    if "coordinateOrientation" in data and isinstance(
        data["coordinateOrientation"], dict
    ):
        try_set(
            obj,
            "coordinateOrientation",
            _dict_to_coordinate_orientation(data["coordinateOrientation"]),
        )
    if "targetOrientation" in data and isinstance(data["targetOrientation"], dict):
        try_set(
            obj,
            "targetOrientation",
            _dict_to_target_orientation(data["targetOrientation"]),
        )
    return obj


def _dict_to_pending_option(data: dict) -> PendingOption:
    obj = cs_new("PendingOption", MSG_ID)
    if "optionID" in data:
        try_set(obj, "optionID", int(data["optionID"]))
    if "optionName" in data:
        try_set(obj, "optionName", str(data["optionName"]))
    if "missionPlanID" in data:
        try_set(obj, "missionPlanID", int(data["missionPlanID"]))
    return obj


def _dict_to_replan_request(data: dict) -> ReplanRequest:
    obj = cs_new("ReplanRequest", MSG_ID)
    if "timestamp" in data:
        try_set(obj, "timestamp", int(data["timestamp"]))

    val_src = data.get(
        "source", data.get("sourceModuleName", data.get("requestModuleName", ""))
    )
    if val_src:
        if not try_set(obj, "source", str(val_src)):
            try_set(obj, "sourceModuleName", str(val_src))

    if "replanRequestTime" in data and isinstance(data["replanRequestTime"], dict):
        try_set(
            obj,
            "replanRequestTime",
            _dict_to_replan_request_time(data["replanRequestTime"]),
        )
    if "replanLevel" in data:
        try_set(obj, "replanLevel", int(data["replanLevel"]))
    if "inputMissionIDList" in data and isinstance(data["inputMissionIDList"], list):
        T = cs_new("InputMissionID", MSG_ID)
        lst = List[T]()
        for item in data["inputMissionIDList"]:
            lst.Add(_dict_to_input_mission_id(item if isinstance(item, dict) else {}))
        try_set(obj, "inputMissionIDList", lst)
    if "individualMissionIDList" in data and isinstance(
        data["individualMissionIDList"], list
    ):
        T = cs_new("IndividualMissionID", MSG_ID)
        lst = List[T]()
        for item in data["individualMissionIDList"]:
            lst.Add(
                _dict_to_individual_mission_id(item if isinstance(item, dict) else {})
            )
        try_set(obj, "individualMissionIDList", lst)
    if "priorMissionList" in data and isinstance(data["priorMissionList"], list):
        T = cs_new("PriorMission", MSG_ID)
        lst = List[T]()
        for item in data["priorMissionList"]:
            lst.Add(_dict_to_prior_mission(item if isinstance(item, dict) else {}))
        try_set(obj, "priorMissionList", lst)
    if "replanReason" in data:
        try_set(obj, "replanReason", str(data["replanReason"]))
    if "pendingOptionList" in data and isinstance(data["pendingOptionList"], list):
        T = cs_new("PendingOption", MSG_ID)
        lst = List[T]()
        for item in data["pendingOptionList"]:
            lst.Add(_dict_to_pending_option(item if isinstance(item, dict) else {}))
        try_set(obj, "pendingOptionList", lst)
    return obj


def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """Creates and pushes a message, returning a log string."""
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = select_tx_fields(body_dict, wl)

    msg = _dict_to_replan_request(body_dict)
    node_messenger.Push(msg)

    log_line = f"[{MSG_ID}] PUSH 완료"
    return log_line.encode("utf-8", "ignore")


def make_random_and_push(node_messenger) -> bytes:
    """Creates a message with random/default data and pushes it."""
    return make_and_push_based_on_rules(
        MSG_ID,
        __file__,
        make_msg0902_body,
        make_and_push,
        node_messenger,
    )
