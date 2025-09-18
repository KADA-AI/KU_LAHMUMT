# modules/monitoring/push/message0501_push.py
from nFusion.Model.msg_0501 import MissionProgress
from data.message_models import MissionProgressBodyModel
from .push_utils import cs_new, try_set

MSG_ID = "0501"


def _model_to_cs_object(body: MissionProgressBodyModel) -> MissionProgress:
    """Converts a MissionProgressBodyModel object to a C# MissionProgress object."""
    obj = cs_new("MissionProgress", MSG_ID)

    try_set(obj, "timestamp", body.timestamp)

    return obj


def make_and_push(body: MissionProgressBodyModel, node_messenger) -> bytes:
    """Creates and pushes a message from a MissionProgressBodyModel object or a compatible dict."""
    if isinstance(body, dict):
        body = MissionProgressBodyModel(**body)
    elif not isinstance(body, MissionProgressBodyModel):
        raise TypeError(
            f"body must be a MissionProgressBodyModel object or a dict, not {type(body).__name__}"
        )

    msg = _model_to_cs_object(body)
    node_messenger.Push(msg)

    log_line = f"[{MSG_ID}] PUSH 완료"
    return log_line.encode("utf-8", "ignore")
