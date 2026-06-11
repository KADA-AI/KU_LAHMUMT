"""PriorMissionCancelRequest (0804) receiver that converts CLR objects to Python dicts."""

from __future__ import annotations

import json
import warnings
from typing import Any

from dll_files.nFusionImports import *  # noqa: F401,F403 - provided by runtime

try:
    from nFusion.Model.msg_0804 import PriorMissionCancelRequest, PriorMission  # type: ignore
except Exception:  # pragma: no cover - environment dependent
    PriorMissionCancelRequest = None  # type: ignore
    PriorMission = None  # type: ignore
    HAS_CLR_0804 = False
else:
    HAS_CLR_0804 = True

from nFusion.Model.CommonType import *  # noqa: F401,F403 - indirect side effects

from receive_center import notify
from .database import received_db

_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)


def _extract_prior_mission_id(obj: Any) -> int | None:
    if obj is None:
        return None
    raw = _get(obj, "priorMissionID", "PriorMissionID")
    if raw is None and isinstance(obj, dict):
        raw = obj.get("priorMissionID") or obj.get("PriorMissionID")
    if raw is None:
        return None
    try:
        return int(raw)
    except Exception:
        return None


def _to_dict_cancel(obj: Any) -> dict:
    data: dict = {}

    ts = _get(obj, "timestamp", "Timestamp") if obj is not None else None
    if ts is not None:
        try:
            data["timestamp"] = int(ts)
        except Exception:
            pass

    source = _get(
        obj,
        "source",
        "Source",
        "requestModuleName",
        "RequestModuleName",
        "SourceModuleName",
    )
    if source:
        data["source"] = str(source)

    prior = _get(obj, "priorMission", "PriorMission")
    prior_id = _extract_prior_mission_id(prior)
    if prior_id is not None:
        data["priorMission"] = {"priorMissionID": prior_id}

    return data


if HAS_CLR_0804:

    class PriorMissionCancelRequestReceiver_0804(
        IFusionReceive[PriorMissionCancelRequest], IsLocal, IsSingletone
    ):
        """Receive PriorMissionCancelRequest and forward to the Python notification hub."""

        __namespace__ = "PriorMissionCancelRequestReceiver_0804"

        def Receive(self, data: PriorMissionCancelRequest, src):  # noqa: N802 - CLR signature
            try:
                received_db.set_received_0804(data)
                payload = json.dumps(_to_dict_cancel(data), ensure_ascii=False).encode("utf-8", "ignore")
                notify("0804", payload)
            except Exception:
                import sys
                import traceback

                sys.stderr.write("[ERROR][Receive-0804] traceback below\n")
                traceback.print_exc(file=sys.stderr)

else:
    PriorMissionCancelRequestReceiver_0804 = None  # type: ignore
    warnings.warn(
        "MessageLibrary does not provide msg_0804.PriorMissionCancelRequest; disabling 0804 receiver",
        ImportWarning,
    )

__all__ = ["PriorMissionCancelRequestReceiver_0804", "HAS_CLR_0804"]
