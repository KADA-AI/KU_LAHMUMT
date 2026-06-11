"""NoticeInfo (0001) receiver that converts CLR objects to Python dicts."""

from __future__ import annotations

import json
import warnings
from typing import Any

from dll_files.nFusionImports import *  # noqa: F401,F403 - provided by runtime

try:
    from nFusion.Model.msg_0001 import NoticeInfo  # type: ignore
except Exception:  # pragma: no cover - environment dependent
    NoticeInfo = None  # type: ignore
    HAS_CLR_NOTICE = False
else:
    HAS_CLR_NOTICE = True

from nFusion.Model.CommonType import *  # noqa: F401,F403 - indirect side effects

from receive_center import notify
from .database import received_db

_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)


def _to_dict_notice(obj: Any) -> dict:
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

    contents = _get(obj, "contents", "Contents")
    if contents is not None:
        data["contents"] = str(contents)

    return data


if HAS_CLR_NOTICE:

    class NoticeInfoReceiver_0001(IFusionReceive[NoticeInfo], IsLocal, IsSingletone):
        """Receive NoticeInfo messages and forward them to the Python notification hub."""

        __namespace__ = "NoticeInfoReceiver_0001"

        def Receive(self, data: NoticeInfo, src):  # noqa: N802 - CLR signature
            try:
                received_db.set_received_0001(data)
                payload = json.dumps(_to_dict_notice(data), ensure_ascii=False).encode("utf-8", "ignore")
                notify("0001", payload)
            except Exception:
                import sys
                import traceback

                sys.stderr.write("[ERROR][Receive-0001] traceback below\n")
                traceback.print_exc(file=sys.stderr)

else:
    NoticeInfoReceiver_0001 = None  # type: ignore
    warnings.warn(
        "MessageLibrary does not provide msg_0001.NoticeInfo; disabling 0001 receiver",
        ImportWarning,
    )

__all__ = ["NoticeInfoReceiver_0001", "HAS_CLR_NOTICE"]
