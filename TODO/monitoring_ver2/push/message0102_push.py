# modules/monitoring/push/message0102_push.py
from nFusion.Model.msg_0102 import ModuleStatus
from data.message_models import ModuleStatusModelModel
from .push_utils import cs_new, try_set

MSG_ID = "0102"


def _model_to_cs_object(body: ModuleStatusModelModel) -> ModuleStatus:
    """Converts a ModuleStatusModelModel object to a C# ModuleStatus object."""
    obj = cs_new("ModuleStatus", MSG_ID)

    try_set(obj, "timestamp", body.timestamp)
    try_set(obj, "source", body.source)
    try_set(obj, "status", body.status)

    return obj


def make_and_push(body: ModuleStatusModelModel, node_messenger) -> bytes:
    """Creates and pushes a message from a ModuleStatusModelModel object or a compatible dict."""
    if isinstance(body, dict):
        # Convert dict to a dataclass object if needed
        body = ModuleStatusModelModel(**body)
    elif not isinstance(body, ModuleStatusModelModel):
        raise TypeError(
            f"body must be a ModuleStatusModelModel object or a dict, not {type(body).__name__}"
        )

    msg = _model_to_cs_object(body)
    node_messenger.Push(msg)

    log_line = f"[{MSG_ID}] PUSH 완료"
    return log_line.encode("utf-8", "ignore")
