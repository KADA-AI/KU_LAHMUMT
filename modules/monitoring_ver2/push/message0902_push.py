# modules/monitoring/push/message0902_push.py
from dataclasses import asdict, is_dataclass
from typing import Any, Dict

from data.message_models import ReplanRequestBodyModel

try:
    from modules.common.push import message0902_push as _common_push
except ModuleNotFoundError:
    import importlib

    _common_push = importlib.import_module("modules.common.push.message0902_push")

MSG_ID = "0902"


def _normalize_body(body: Any) -> Dict[str, Any]:
    """Convert payloads into a dict understood by the shared 0902 push helper."""
    if isinstance(body, dict):
        body_dict = dict(body)
    elif is_dataclass(body):
        body_dict = asdict(body)
    elif isinstance(body, ReplanRequestBodyModel):
        body_dict = asdict(body)
    else:
        raise TypeError(
            f"body must be a ReplanRequestBodyModel, dataclass, or dict, not {type(body).__name__}"
        )

    if "replanRequest" in body_dict and "replanReason" not in body_dict:
        body_dict["replanReason"] = body_dict.pop("replanRequest")
    if "optionList" in body_dict and "pendingOptionList" not in body_dict:
        body_dict["pendingOptionList"] = body_dict.pop("optionList")
    if "IndividualMissionIDList" in body_dict:
        body_dict["individualMissionIDList"] = body_dict.pop("IndividualMissionIDList")

    return body_dict


def make_and_push(body: Any, node_messenger) -> bytes:
    """Delegate 0902 pushes to the shared common implementation."""
    return _common_push.make_and_push(_normalize_body(body), node_messenger)
