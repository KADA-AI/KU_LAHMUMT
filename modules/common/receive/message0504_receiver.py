"""FuelWarning (0504) receiver that converts CLR objects to Python dicts.

When the MessageLibrary assembly does not expose msg_0504 types, the receiver
is disabled gracefully so the rest of the system can continue to load.
"""

from __future__ import annotations

import json
import warnings
from typing import Any, Optional

from dll_files.nFusionImports import *  # noqa: F401,F403 - provided by runtime

try:  # Attempt to load the CLR message type; it may be absent in older builds.
    from nFusion.Model.msg_0504 import FuelWarning  # type: ignore
except Exception:  # pragma: no cover - environment dependent
    FuelWarning = None  # type: ignore
    HAS_CLR_FUELWARNING = False
else:
    HAS_CLR_FUELWARNING = True

from nFusion.Model.CommonType import *  # noqa: F401,F403 - indirect side effects

from receive_center import notify
from .database import received_db

# Helper to obtain attributes without raising if the name differs in casing
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)


def _lookup_fuel_level_from_agent_status(aid: int) -> Optional[int]:
    """Fetch the most recent fuelWarning flag for the given aircraft from 0401 data."""

    try:
        agent_status = received_db.get_received_0401()
    except Exception:
        return None

    if not agent_status:
        return None

    states = _get(agent_status, "agentStateList", "AgentStateList")
    if not states:
        return None

    for state in states:
        try:
            state_aid = _get(state, "aircraftID", "AircraftID")
            if state_aid is None:
                continue
            state_aid = int(state_aid)
        except Exception:
            continue

        if state_aid != aid:
            continue

        level = _get(state, "fuelWarning", "FuelWarning")
        if level is None:
            unmanned = _get(state, "unmannedInfo", "UnmannedInfo")
            if unmanned is not None:
                level = _get(unmanned, "fuelWarning", "FuelWarning")

        if level is None:
            return None

        try:
            level_int = int(level)
        except Exception:
            return None

        if level_int < 0:
            return None
        return level_int

    return None


def _to_dict_fuel_warning(obj: Any) -> dict:
    """Convert a payload into the canonical FuelWarning dictionary."""

    data: dict = {}

    ts = _get(obj, "timestamp", "Timestamp") if obj is not None else None
    if ts is not None:
        try:
            data["timestamp"] = int(ts)
        except Exception:
            pass

    source = None
    if obj is not None:
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

    aid_val = None
    if isinstance(obj, dict):
        aid_val = obj.get("aircraftID")
    elif obj is not None:
        aid_val = _get(obj, "aircraftID", "AircraftID")
    if aid_val is not None:
        try:
            data["aircraftID"] = int(aid_val)
        except Exception:
            pass

    level_val = None
    if isinstance(obj, dict):
        level_val = obj.get("fuelLevel")
    elif obj is not None:
        level_val = _get(obj, "fuelLevel", "FuelLevel")
    if level_val is not None:
        try:
            data["fuelLevel"] = int(level_val)
        except Exception:
            data["fuelLevel"] = 1

    aid = data.get("aircraftID")
    if aid is not None:
        try:
            aid_int = int(aid)
        except Exception:
            aid_int = None
        if aid_int is not None:
            if "fuelLevel" not in data:
                fallback = _lookup_fuel_level_from_agent_status(aid_int)
                if fallback is not None:
                    data["fuelLevel"] = int(fallback)
            elif data.get("fuelLevel") in (None, 0):
                fallback = _lookup_fuel_level_from_agent_status(aid_int)
                if fallback not in (None, 0):
                    data["fuelLevel"] = int(fallback)

    return data


if HAS_CLR_FUELWARNING:

    class FuelWarningReceiver_0504(IFusionReceive[FuelWarning], IsLocal, IsSingletone):
        """Receive FuelWarning messages and forward them to the Python notification hub."""

        __namespace__ = "FuelWarningReceiver_0504"

        def Receive(self, data: FuelWarning, src):  # noqa: N802 - CLR signature
            try:
                body = _to_dict_fuel_warning(data)
                payload = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
                notify("0504", payload)
            except Exception:
                import sys
                import traceback

                sys.stderr.write("[ERROR][Receive-0504] traceback below\n")
                traceback.print_exc(file=sys.stderr)

else:  # Graceful fallback when the CLR type does not exist
    FuelWarningReceiver_0504 = None  # type: ignore
    warnings.warn(
        "MessageLibrary does not provide msg_0504.FuelWarning; disabling 0504 receiver",
        ImportWarning,
    )

__all__ = ["FuelWarningReceiver_0504", "HAS_CLR_FUELWARNING"]
