# modules/monitoring/push/message0501_push.py
from dataclasses import asdict, is_dataclass
from typing import Any, Dict

from data.message_models import MissionProgressBodyModel

try:
    from modules.common.push import message0501_push as _common_push
except ModuleNotFoundError:
    import importlib

    _common_push = importlib.import_module('modules.common.push.message0501_push')

MSG_ID = "0501"


def _prepare_body(body: Any) -> Dict[str, Any]:
    """Normalize MissionProgress payload into a dict for the common push helper."""
    if is_dataclass(body):
        body_dict = asdict(body)
    elif isinstance(body, MissionProgressBodyModel):  # defensive, dataclass already
        body_dict = asdict(body)
    elif isinstance(body, dict):
        body_dict = dict(body)
    else:
        raise TypeError(
            f"body must be a MissionProgressBodyModel, dataclass, or dict, not {type(body).__name__}"
        )

    if "sourceModuleName" in body_dict and "source" not in body_dict:
        # Legacy field name from earlier UI helpers
        body_dict["source"] = body_dict.pop("sourceModuleName")

    return body_dict


def make_and_push(body: Any, node_messenger) -> bytes:
    """Delegate MissionProgress pushes to the shared common push implementation."""
    return _common_push.make_and_push(_prepare_body(body), node_messenger)


def make_random_and_push(node_messenger) -> bytes:
    """Fall back to the shared random-body generator when needed."""
    return _common_push.make_random_and_push(node_messenger)
