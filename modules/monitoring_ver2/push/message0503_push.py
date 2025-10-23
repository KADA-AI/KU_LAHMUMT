# modules/monitoring/push/message0503_push.py
from nFusion.Model.msg_0503 import MissionResult
from data.message_models import CollaborativeMissionCompleteModel
from .push_utils import cs_new, try_set

MSG_ID = "0503"


def _model_to_cs_object(body: CollaborativeMissionCompleteModel) -> MissionResult:
    """Converts a CollaborativeMissionCompleteModel object to a C# MissionResult object."""
    obj = cs_new("MissionResult", MSG_ID)

    try_set(obj, "timestamp", body.timestamp)
    try_set(obj, "source", body.source)
    try_set(obj, "systemRecommend", body.systemRecommend)

    return obj


def make_and_push(body: CollaborativeMissionCompleteModel, node_messenger) -> bytes:
    """Creates and pushes a message from a CollaborativeMissionCompleteModel object or a compatible dict."""
    if isinstance(body, dict):
        body = CollaborativeMissionCompleteModel(**body)
    elif not isinstance(body, CollaborativeMissionCompleteModel):
        raise TypeError(
            f"body must be a CollaborativeMissionCompleteModel object or a dict, not {type(body).__name__}"
        )

    msg = _model_to_cs_object(body)
    node_messenger.Push(msg)

    log_line = f"[{MSG_ID}] PUSH 완료"
    return log_line.encode("utf-8", "ignore")
