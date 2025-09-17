# modules/monitoring/push/message0502_push.py
from nFusion.Model.msg_0502 import EndMissionRequest
from generator.message0502_generator import make_msg0502_body
from .push_utils import (
    cs_new,
    try_set,
    select_tx_fields,
    make_and_push_based_on_rules,
    TX_FIELD_WHITELIST,
)

MSG_ID = "0502"


def _dict_to_end_mission_request(data: dict) -> EndMissionRequest:
    """Converts a dictionary to a C# EndMissionRequest object."""
    obj = cs_new("EndMissionRequest", MSG_ID)
    if "timestamp" in data:
        try_set(obj, "timestamp", int(data["timestamp"]))

    val_src = data.get(
        "source", data.get("sourceModuleName", data.get("requestModuleName", ""))
    )
    if val_src:
        if not try_set(obj, "source", str(val_src)):
            try_set(obj, "sourceModuleName", str(val_src))

    return obj


def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """Creates and pushes a message, returning a log string."""
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = select_tx_fields(body_dict, wl)

    msg = _dict_to_end_mission_request(body_dict)
    node_messenger.Push(msg)

    log_line = f"[{MSG_ID}] PUSH 완료"
    return log_line.encode("utf-8", "ignore")


def make_random_and_push(node_messenger) -> bytes:
    """Creates a message with random/default data and pushes it."""
    return make_and_push_based_on_rules(
        MSG_ID,
        __file__,
        make_msg0502_body,
        make_and_push,
        node_messenger,
    )
