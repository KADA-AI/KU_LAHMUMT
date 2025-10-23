# modules/monitoring/push/message0902_push.py
from nFusion.Model.msg_0902 import ReplanRequest
from data.message_models import ReplanRequestBodyModel, ReplanRequestTimeStampModel
from .push_utils import cs_new, try_set

MSG_ID = "0902"


def _model_to_cs_object(body: ReplanRequestBodyModel) -> ReplanRequest:
    """Converts a ReplanRequestBodyModel object to a C# ReplanRequest object."""
    obj = cs_new("ReplanRequest", MSG_ID)

    try_set(obj, "timestamp", body.timestamp)
    # try_set(obj, "source", body.source)
    # The dataclass has `replanRequest`, which seems to map to `replanReason` in the C# model.
    try_set(obj, "replanReason", body.replanRequest)

    return obj


def make_and_push(body: ReplanRequestBodyModel, node_messenger) -> bytes:
    """Creates and pushes a message from a ReplanRequestBodyModel object or a compatible dict."""
    if isinstance(body, dict):
        # 중첩된 dataclass가 dict로 들어올 경우, 수동으로 변환해줘야 합니다.
        if "replanRequestTime" in body and isinstance(body["replanRequestTime"], dict):
            body["replanRequestTime"] = ReplanRequestTimeStampModel(
                **body["replanRequestTime"]
            )
        # 다른 중첩 구조들도 필요시 여기에 추가합니다.
        body = ReplanRequestBodyModel(**body)
    elif not isinstance(body, ReplanRequestBodyModel):
        raise TypeError(
            f"body must be a ReplanRequestBodyModel object or a dict, not {type(body).__name__}"
        )

    msg = _model_to_cs_object(body)
    node_messenger.Push(msg)

    log_line = f"[{MSG_ID}] PUSH 완료"
    return log_line.encode("utf-8", "ignore")
