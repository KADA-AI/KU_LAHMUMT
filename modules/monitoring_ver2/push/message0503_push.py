from dataclasses import asdict, is_dataclass
import json

from nFusion.Model.msg_0503 import MissionResult
from data.message_models import CollaborativeMissionCompleteModel
from .push_utils import cs_new, try_set

try:
    from modules.common.push import message0503_push as _common_push
except ModuleNotFoundError:
    import importlib

    _common_push = importlib.import_module("modules.common.push.message0503_push")

MSG_ID = "0503"

_KEY_MAP = {
    "timestamp": "timestamp",
    "Timestamp": "timestamp",
    "timeStamp": "timestamp",
    "TimeStamp": "timestamp",
    "source": "source",
    "Source": "source",
    "requestModuleName": "source",
    "RequestModuleName": "source",
    "systemRecommend": "systemRecommend",
    "SystemRecommend": "systemRecommend",
    "systemRecommendResult": "systemRecommend",
}


def _normalize_dict(body_dict: dict) -> dict:
    normalized = {}
    for key, value in body_dict.items():
        mapped = _KEY_MAP.get(key, key)
        normalized[mapped] = value
    return normalized


def _model_to_cs_object(body: CollaborativeMissionCompleteModel) -> MissionResult:
    obj = cs_new("MissionResult", MSG_ID)
    try_set(obj, "timestamp", body.timestamp)
    try_set(obj, "source", body.source)
    try_set(obj, "systemRecommend", body.systemRecommend)
    return obj


def make_and_push(body, node_messenger) -> bytes:
    if isinstance(body, dict):
        body = CollaborativeMissionCompleteModel(**_normalize_dict(body))
    elif not isinstance(body, CollaborativeMissionCompleteModel):
        raise TypeError(
            f"body must be a CollaborativeMissionCompleteModel or dict, not {type(body).__name__}"
        )

    msg = _model_to_cs_object(body)
    node_messenger.Push(msg)

    body_dict = asdict(body) if is_dataclass(body) else {}
    body_json = json.dumps(body_dict, ensure_ascii=False)
    log_line = f"[{MSG_ID}] PUSH 완료 :: {body_json}"
    return log_line.encode("utf-8", "ignore")


def make_random_and_push(node_messenger) -> bytes:
    """Delegate random message generation to the shared common implementation."""
    return _common_push.make_random_and_push(node_messenger)
