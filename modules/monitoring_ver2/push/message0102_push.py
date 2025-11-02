from dataclasses import asdict, is_dataclass
import json
from nFusion.Model.msg_0102 import ModuleStatus
from data.message_models import ModuleStatusModelModel
from .push_utils import cs_new, try_set

MSG_ID = "0102"


_KEY_MAP = {
    "timestamp": "timestamp",
    "Timestamp": "timestamp",
    "timeStamp": "timestamp",
    "TimeStamp": "timestamp",
    "ts": "timestamp",
    "TS": "timestamp",
    "source": "source",
    "Source": "source",
    "requestModuleName": "source",
    "RequestModuleName": "source",
    "module": "source",
    "Module": "source",
    "sourceModule": "source",
    "SourceModule": "source",
    "status": "status",
    "Status": "status",
    "moduleStatus": "status",
    "ModuleStatus": "status",
    "state": "status",
    "State": "status",
    "healthStatus": "status",
    "HealthStatus": "status",
}


def _normalize_dict(body_dict: dict) -> dict:
    normalized = {}
    for key, value in body_dict.items():
        mapped = _KEY_MAP.get(key, key)
        normalized[mapped] = value
    return normalized


def _model_to_cs_object(body: ModuleStatusModelModel) -> ModuleStatus:
    """단일 ModuleStatusModelModel 객체를 C# ModuleStatus로 변환."""
    obj = cs_new("ModuleStatus", MSG_ID)

    try_set(obj, "timestamp", body.timestamp)
    try_set(obj, "source", body.source)
    try_set(obj, "status", body.status)

    return obj


def make_and_push(body, node_messenger) -> bytes:
    """dict 또는 ModuleStatusModelModel을 받아 메시지를 전송."""
    if isinstance(body, dict):
        body = ModuleStatusModelModel(**_normalize_dict(body))
    elif not isinstance(body, ModuleStatusModelModel):
        raise TypeError(
            f"body must be a ModuleStatusModelModel or dict, not {type(body).__name__}"
        )

    msg = _model_to_cs_object(body)
    node_messenger.Push(msg)

    body_dict = asdict(body) if is_dataclass(body) else {}
    body_json = json.dumps(body_dict, ensure_ascii=False)
    log_line = f"[{MSG_ID}] PUSH 완료 :: {body_json}"
    return log_line.encode("utf-8", "ignore")
