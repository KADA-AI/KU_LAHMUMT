# modules/monitoring/push/message0502_push.py
from nFusion.Model.msg_0502 import EndMissionRequest
from data.message_models import MissionEndRequestBodyModel
from .push_utils import cs_new, try_set

MSG_ID = "0502"


def _model_to_cs_object(body: MissionEndRequestBodyModel) -> EndMissionRequest:
    """Converts a MissionEndRequestBodyModel object to a C# EndMissionRequest object."""
    obj = cs_new("EndMissionRequest", MSG_ID)

    try_set(obj, "timestamp", body.timestamp)
    try_set(obj, "sourceModuleName", body.sourceModuleName)
    try_set(obj, "reason", body.reason)

    return obj


def make_and_push(body: MissionEndRequestBodyModel, node_messenger) -> bytes:
    """Creates and pushes a message from a MissionEndRequestBodyModel object or a compatible dict."""
    if isinstance(body, dict):
        body = MissionEndRequestBodyModel(**body)
    elif not isinstance(body, MissionEndRequestBodyModel):
        raise TypeError(
            f"body must be a MissionEndRequestBodyModel object or a dict, not {type(body).__name__}"
        )

    msg = _model_to_cs_object(body)
    node_messenger.Push(msg)

    log_line = f"[{MSG_ID}] PUSH 완료"
    return log_line.encode("utf-8", "ignore")
