"""Fuel-warning aware ReplanRequest (0902) receiver with graceful CLR fallback."""

from __future__ import annotations

import json
import os
import sys
import traceback
import warnings
from typing import Any

from dll_files.nFusionImports import *  # IFusionReceive, IsLocal, IsSingletone

try:  # MessageLibrary may omit msg_0902 in some builds
    from nFusion.Model.msg_0902 import ReplanRequest  # type: ignore
except Exception:
    ReplanRequest = None  # type: ignore[assignment]
    HAS_CLR_REPLANREQUEST = False
else:
    HAS_CLR_REPLANREQUEST = True

from nFusion.Model.CommonType import *  # noqa: F401,F403
from .database import received_db
from receive_center import notify

# ── Shared helpers ──────────────────────────────────────────────────────────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

TX_FIELD_WHITELIST = {
    "0201": ["timestamp", "inputMissionPackageID"],
    "0203": ["timestamp", "missionReferencePackageID"],
    "0301": ["timestamp", "missionPlanID"],
    "0302": ["timestamp", "individualMissionPackageID"],
    "0303": ["timestamp", "pathID"],
    "0304": ["timestamp", "pathID"],
}

DB_DIR_RULES = {
    "0201": "InputMissionPlan",
    "0203": "FlightReferenceInfo",
    "0301": "MissionPlan",
    "0302": "IndividualMissionPlan",
    "0303": "UAVFlightPlan",
    "0304": "FlightPath",
}

DB_FETCH_ON_RECEIVE = {"0201", "0203"}
ID_FIELD_FOR = {
    "0201": "inputMissionPackageID",
    "0203": "missionReferencePackageID",
    "0301": "missionPlanID",
    "0302": "individualMissionPackageID",
    "0303": "pathID",
    "0304": "pathID",
}


def _project_root_for_recv_file(__file_path: str) -> str:
    from pathlib import Path

    return str(Path(__file_path).resolve().parents[3])


def _db_dir_for(msgid: str, __file_path: str) -> str:
    from pathlib import Path

    env_root = os.getenv("KU_MISSION_DB_ROOT")
    name = DB_DIR_RULES.get(msgid)
    if not name:
        return _project_root_for_recv_file(__file_path)
    if env_root:
        return str(Path(env_root) / name)
    return str(Path(_project_root_for_recv_file(__file_path)) / "database" / name)


def _try_save_received(msgid: str, data_obj: Any) -> None:
    try:
        setter = getattr(received_db, f"set_received_{msgid}")
        setter(data_obj)
    except Exception:
        pass


def _try_read_db_body(msgid: str, data_obj: Any) -> Any:
    try:
        if msgid not in DB_FETCH_ON_RECEIVE:
            return None
        id_field = ID_FIELD_FOR.get(msgid)
        if not id_field:
            return None
        value = _get(data_obj, id_field, id_field[:1].upper() + id_field[1:])
        if value is None:
            return None
        vid = int(value)
        fpath = os.path.join(_db_dir_for(msgid, __file__), f"{vid}.json")
        print(f"[{msgid}] DB lookup! ({fpath})")
        if os.path.exists(fpath):
            with open(fpath, "r", encoding="utf-8") as fh:
                return json.load(fh)
        return None
    except Exception:
        return None


# ── Payload normalisers ─────────────────────────────────────────────────────────

def _to_dict_ReplanRequestTime(obj):
    d = {}
    val = _get(obj, "replanRequestTimestamp", "ReplanRequestTimestamp")
    if val is not None:
        d["replanRequestTimestamp"] = int(val)
    return d


def _to_dict_InputMissionID(obj):
    d = {}
    val = _get(obj, "inputMissionID", "InputMissionID")
    if val is not None:
        d["inputMissionID"] = int(val)
    return d


def _to_dict_IndividualMissionID(obj):
    d = {}
    val = _get(obj, "individualMissionID", "IndividualMissionID")
    if val is not None:
        d["individualMissionID"] = int(val)
    return d


def _to_dict_Coordinate(obj):
    d = {}
    lat = _get(obj, "latitude", "Latitude")
    if lat is not None:
        d["latitude"] = float(lat)
    lon = _get(obj, "longitude", "Longitude")
    if lon is not None:
        d["longitude"] = float(lon)
    alt = _get(obj, "altitude", "Altitude")
    if alt is not None:
        d["altitude"] = int(alt)
    return d


def _to_dict_CoordinateOrientation(obj):
    d = {}
    coord = _get(obj, "coordinate", "Coordinate")
    if coord is not None:
        d["coordinate"] = _to_dict_Coordinate(coord)
    return d


def _to_dict_TargetOrientation(obj):
    d = {}
    tid = _get(obj, "targetID", "TargetID")
    if tid is not None:
        d["targetID"] = int(tid)
    return d


def _to_dict_PriorMission(obj):
    d = {}
    pid = _get(obj, "priorMissionID", "PriorMissionID")
    if pid is not None:
        d["priorMissionID"] = int(pid)
    mtype = _get(obj, "missionType", "MissionType")
    if mtype is not None:
        d["missionType"] = int(mtype)
    coord = _get(obj, "coordinateOrientation", "CoordinateOrientation")
    if coord is not None:
        d["coordinateOrientation"] = _to_dict_CoordinateOrientation(coord)
    target = _get(obj, "targetOrientation", "TargetOrientation")
    if target is not None:
        d["targetOrientation"] = _to_dict_TargetOrientation(target)
    return d


def _to_dict_PendingOption(obj):
    d = {}
    oid = _get(obj, "optionID", "OptionID")
    if oid is not None:
        d["optionID"] = int(oid)
    name = _get(obj, "optionName", "OptionName")
    if name:
        d["optionName"] = str(name)
    mid = _get(obj, "missionPlanID", "MissionPlanID")
    if mid is not None:
        d["missionPlanID"] = int(mid)
    return d


def _to_dict_ReplanRequest(obj: Any) -> dict:
    d: dict = {}

    ts = _get(obj, "timestamp", "Timestamp")
    if ts is not None:
        d["timestamp"] = int(ts)

    src = _get(
        obj,
        "source",
        "Source",
        "requestModuleName",
        "RequestModuleName",
    )
    if src:
        d["source"] = str(src)

    req_time = _get(obj, "replanRequestTime", "ReplanRequestTime")
    if req_time is not None:
        d["replanRequestTime"] = _to_dict_ReplanRequestTime(req_time)

    level = _get(obj, "replanLevel", "ReplanLevel")
    if level is not None:
        d["replanLevel"] = int(level)

    missions = _get(obj, "inputMissionIDList", "InputMissionIDList") or []
    if missions:
        d["inputMissionIDList"] = [_to_dict_InputMissionID(it) for it in missions]

    indiv = _get(obj, "individualMissionIDList", "IndividualMissionIDList") or []
    if indiv:
        d["individualMissionIDList"] = [_to_dict_IndividualMissionID(it) for it in indiv]

    prior = _get(obj, "priorMissionList", "PriorMissionList") or []
    if prior:
        d["priorMissionList"] = [_to_dict_PriorMission(it) for it in prior]

    reason = _get(obj, "replanReason", "ReplanReason")
    if reason:
        d["replanReason"] = str(reason)

    pending = _get(obj, "pendingOptionList", "PendingOptionList") or []
    if pending:
        d["pendingOptionList"] = [_to_dict_PendingOption(it) for it in pending]

    detail = _get(obj, "replanDetail", "ReplanDetail")
    if detail:
        if isinstance(detail, (bytes, bytearray)):
            try:
                detail = detail.decode("utf-8", "ignore")
            except Exception:
                detail = detail.decode(errors="ignore")
        if isinstance(detail, str):
            try:
                d["replanDetail"] = json.loads(detail)
            except Exception:
                d["replanDetail"] = detail
        else:
            d["replanDetail"] = detail

    return d


# ── Receiver definition ─────────────────────────────────────────────────────────

if HAS_CLR_REPLANREQUEST:

    class ReplanRequestReceiver_0902(IFusionReceive[ReplanRequest], IsLocal, IsSingletone):
        """Receive ReplanRequest messages and dispatch them via notify."""

        __namespace__ = "ReplanRequestReceiver_0902"

        def Receive(self, data: ReplanRequest, src):  # noqa: N802
            try:
                _try_save_received("0902", data)

                body = _try_read_db_body("0902", data)
                if body is None:
                    body = _to_dict_ReplanRequest(data)

                payload = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
                notify("0902", payload)

            except Exception:
                sys.stderr.write("[ERROR][Receive-0902] traceback below\n")
                traceback.print_exc(file=sys.stderr)

else:
    ReplanRequestReceiver_0902 = None  # type: ignore
    warnings.warn(
        "MessageLibrary does not expose msg_0902.ReplanRequest; disabling 0902 receiver",
        ImportWarning,
    )

__all__ = ["ReplanRequestReceiver_0902", "HAS_CLR_REPLANREQUEST"]
