# modules/monitoring/push/message0102_push.py
from nFusion.Model.msg_0102 import ModuleStatus
from generator.message0102_generator import make_msg0102_body
from .push_utils import (
    cs_new,
    try_set,
    select_tx_fields,
    make_and_push_based_on_rules,
    TX_FIELD_WHITELIST,
)

MSG_ID = "0102"


def _dict_to_module_status(data: dict) -> ModuleStatus:
    """Converts a dictionary to a C# ModuleStatus object."""
    obj = cs_new("ModuleStatus", MSG_ID)
    if "timestamp" in data:
        try_set(obj, "timestamp", int(data["timestamp"]))

    val_src = data.get(
        "source", data.get("sourceModuleName", data.get("requestModuleName", ""))
    )
    if val_src:
        if not try_set(obj, "source", str(val_src)):
            try_set(obj, "sourceModuleName", str(val_src))

    if "status" in data:
        try_set(obj, "status", int(data["status"]))

    return obj


def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """Creates and pushes a message, returning a log string."""
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = select_tx_fields(body_dict, wl)

    msg = _dict_to_module_status(body_dict)
    node_messenger.Push(msg)

    log_line = f"[{MSG_ID}] PUSH 완료"
    return log_line.encode("utf-8", "ignore")


def make_random_and_push(node_messenger) -> bytes:
    """Creates a message with random/default data and pushes it."""
    return make_and_push_based_on_rules(
        MSG_ID,
        __file__,
        make_msg0102_body,
        make_and_push,
        node_messenger,
    )
