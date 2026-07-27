# modules/common/push/message0902_push.py
# auto-generated at 2025-08-24T20:13:14.043862+00:00


import json, importlib
from datetime import datetime, timezone
from pathlib import Path
from System.Collections.Generic import List
from nFusion.Model.msg_0902 import *    # C# 모델(우선)
from nFusion.Model.CommonType import *     # 공통 타입(항상)
from System import Int32, Single, String, UInt32, UInt64
from generator.message0902_generator import make_msg0902_body
from modules.common import replan_request_transport_store
try:
    from modules.common import replan_perf
except Exception:
    import sys as _sys

    _COMMON_DIR = next(
        (
            parent
            for parent in Path(__file__).resolve().parents
            if (parent / "replan_perf.py").exists()
        ),
        None,
    )
    if _COMMON_DIR is not None and str(_COMMON_DIR) not in _sys.path:
        _sys.path.insert(0, str(_COMMON_DIR))
    import replan_perf  # type: ignore
from modules.common.string_limits import limit_utf8_bytes
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
from modules.common.source_utils import get_default_source_code, override_source_fields
try:
    from modules.common.push_type_cache import resolve_csharp_type
except Exception:
    resolve_csharp_type = None
MSG_ID = "0902"
def _try_set(obj, name: str, value) -> bool:
    # lowerCamel 또는 PascalCase 둘 다 시도
    for k in (name, name[:1].upper()+name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False
def _cs(name: str):
    # 현재 전역 → msg_ID 모듈 → CommonType → 루트 순으로 검색
    if callable(resolve_csharp_type):
        return resolve_csharp_type(MSG_ID, name, globals_dict=globals())
    t = globals().get(name)
    if t is not None: return t
    for modname in (f'nFusion.Model.msg_{MSG_ID}', 'nFusion.Model.CommonType', 'nFusion.Model'):
        try:
            mod = importlib.import_module(modname)
            t = getattr(mod, name, None)
            if t is not None: return t
        except Exception:
            pass
    return None
def _new(name: str):
    t = _cs(name)
    if t is None:
        raise NameError(f'type not found: {name}')
    return t()

# ── Embedded TX/DB rules (self-contained) ──────────────────────────────────
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
    "0303": "FlightPath",
    "0304": "FlightPath",
}

def _select_tx_fields(body: dict, fields: list) -> dict:
    """화이트리스트로 선별: timestamp / source 계열 폴백 / 나머지 ID류만 남김"""
    out = {}
    low = {k.lower(): k for k in body.keys()}

    def _get(key: str):
        kl = key.lower()
        if kl in low:
            return body[low[kl]]
        return None

    ts = _get("timestamp")
    if ts is not None:
        out["timestamp"] = int(ts)

    s  = _get("source")
    sm = _get("Source") or _get("Source")
    rq = _get("requestModuleName") or _get("requestmodulename")
    src_val = s or sm or rq
    if src_val:
        out["Source"] = str(src_val)

    for f in fields:
        if f in ("timestamp","source","Source","requestModuleName"):
            continue
        v = _get(f)
        if v is not None:
            try:
                out[f] = int(v)
            except Exception:
                out[f] = v
    return out

def _project_root_for_push_file(__file_path: str):
    from pathlib import Path
    return Path(__file_path).resolve().parents[3]

def _db_dir_for(msgid: str, __file_path: str) -> str:
    import os
    from pathlib import Path
    env_root = os.getenv("KU_MISSION_DB_ROOT")
    name = DB_DIR_RULES.get(msgid, f"msg_{msgid}")
    if env_root:
        return str(Path(env_root) / name)
    return str(_project_root_for_push_file(__file_path) / "temp" / "database" / name)

def _list_numeric_ids(dirname: str, prefix_first_char: str | None = None) -> list[int]:
    import os, glob
    ids = []
    for p in glob.glob(os.path.join(dirname, "*.json")):
        stem = os.path.splitext(os.path.basename(p))[0]
        if stem.isdigit():
            if prefix_first_char and stem[0] not in prefix_first_char:
                continue
            ids.append(int(stem))
    ids.sort()
    return ids

def _dict_to_ReplanRequestTime(data: dict):
    obj = _new('ReplanRequestTime')
    if "replanRequestTimestamp" in data: _try_set(obj, "replanRequestTimestamp", int(data["replanRequestTimestamp"]))
    return obj

def _dict_to_InputMissionID(data: dict):
    obj = _new('InputMissionID')
    if "inputMissionID" in data: _try_set(obj, "inputMissionID", int(data["inputMissionID"]))
    return obj

def _dict_to_IndividualMissionID(data: dict):
    obj = _new('IndividualMissionID')
    if "individualMissionID" in data: _try_set(obj, "individualMissionID", int(data["individualMissionID"]))
    return obj

def _dict_to_Coordinate(data: dict):
    obj = _new('Coordinate')
    if "latitude" in data: _try_set(obj, "latitude", float(data["latitude"]))
    if "longitude" in data: _try_set(obj, "longitude", float(data["longitude"]))
    if "altitude" in data: _try_set(obj, "altitude", int(data["altitude"]))
    return obj

def _dict_to_CoordinateOrientation(data: dict):
    obj = _new('CoordinateOrientation')
    if "coordinate" in data and isinstance(data["coordinate"], dict):
        _try_set(obj, "coordinate", _dict_to_Coordinate(data["coordinate"]))
    return obj

def _dict_to_TargetOrientation(data: dict):
    obj = _new('TargetOrientation')
    if "targetID" in data: _try_set(obj, "targetID", int(data["targetID"]))
    return obj

def _dict_to_PriorMission(data: dict):
    obj = _new('PriorMission')
    if "priorMissionID" in data: _try_set(obj, "priorMissionID", int(data["priorMissionID"]))
    if "missionType" in data: _try_set(obj, "missionType", int(data["missionType"]))
    if "coordinateOrientation" in data and isinstance(data["coordinateOrientation"], dict):
        _try_set(obj, "coordinateOrientation", _dict_to_CoordinateOrientation(data["coordinateOrientation"]))
    if "targetOrientation" in data and isinstance(data["targetOrientation"], dict):
        _try_set(obj, "targetOrientation", _dict_to_TargetOrientation(data["targetOrientation"]))
    return obj

def _dict_to_PendingOption(data: dict):
    obj = _new('PendingOption')
    if "optionID" in data: _try_set(obj, "optionID", int(data["optionID"]))
    if "optionName" in data:
        option_name = limit_utf8_bytes(data["optionName"])
        data["optionName"] = option_name
        _try_set(obj, "optionName", option_name)
    if "missionPlanID" in data: _try_set(obj, "missionPlanID", int(data["missionPlanID"]))
    return obj

def _normalize_human_text_fields(data: dict) -> dict:
    reason_value = data.get("replanRequest", data.get("replanReason"))
    if reason_value is not None:
        reason_text = limit_utf8_bytes(reason_value)
        data["replanRequest"] = reason_text
        data["replanReason"] = reason_text

    for key in ("optionList", "pendingOptionList"):
        items = data.get(key)
        if not isinstance(items, list):
            continue
        for item in items:
            if isinstance(item, dict) and "optionName" in item:
                item["optionName"] = limit_utf8_bytes(item["optionName"])
    return data

def _dict_to_ReplanRequest(data: dict):
    obj = _new('ReplanRequest')
    _normalize_human_text_fields(data)
    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    val_src = data.get("source", data.get("source", data.get("Source", data.get("requestModuleName", ""))))
    if val_src != "":
        if not _try_set(obj, "source", str(val_src)):
            _try_set(obj, "Source", str(val_src))
    if "replanRequestTime" in data and isinstance(data["replanRequestTime"], dict):
        _try_set(obj, "replanRequestTime", _dict_to_ReplanRequestTime(data["replanRequestTime"]))
    if "replanLevel" in data: _try_set(obj, "replanLevel", int(data["replanLevel"]))
    if "inputMissionIDList" in data and isinstance(data["inputMissionIDList"], list):
        T = _cs('InputMissionID') or object
        lst = List[T]()
        for item in data["inputMissionIDList"]: lst.Add(_dict_to_InputMissionID(item if isinstance(item, dict) else {}))
        _try_set(obj, "inputMissionIDList", lst)
    individual_list = data.get("individualMissionIDList")
    if individual_list is None:
        individual_list = data.get("IndividualMissionIDList")
    if individual_list is not None and isinstance(individual_list, list):
        T = _cs('IndividualMissionID') or object
        lst = List[T]()
        for item in individual_list: lst.Add(_dict_to_IndividualMissionID(item if isinstance(item, dict) else {}))
        _try_set(obj, "individualMissionIDList", lst)
    if "priorMissionList" in data and isinstance(data["priorMissionList"], list):
        T = _cs('PriorMission') or object
        lst = List[T]()
        for item in data["priorMissionList"]: lst.Add(_dict_to_PriorMission(item if isinstance(item, dict) else {}))
        _try_set(obj, "priorMissionList", lst)
    reason_value = data.get("replanRequest", data.get("replanReason"))
    if reason_value is not None:
        _try_set(obj, "replanReason", str(reason_value))
    pending_list = data.get("optionList")
    if pending_list is None:
        pending_list = data.get("pendingOptionList")
    if pending_list is not None and isinstance(pending_list, list):
        T = _cs('PendingOption') or object
        lst = List[T]()
        for item in pending_list: lst.Add(_dict_to_PendingOption(item if isinstance(item, dict) else {}))
        _try_set(obj, "pendingOptionList", lst)
    if "replanDetail" in data:
        detail_payload = data["replanDetail"]
        detail_start = replan_perf.start_timer()
        if not isinstance(detail_payload, str):
            try:
                detail_payload = json.dumps(detail_payload, ensure_ascii=False)
            except Exception:
                detail_payload = str(detail_payload)
        replan_perf.add_elapsed(
            "common.replan_0902_push.detail_json",
            detail_start,
            was_string=isinstance(data["replanDetail"], str),
        )
        _try_set(obj, "replanDetail", detail_payload)
    return obj




def _dict_to_obj(body_dict: dict):
    perf_start = replan_perf.start_timer()
    try:
        return _dict_to_ReplanRequest(body_dict)
    finally:
        replan_perf.add_elapsed("common.replan_0902_push.dict_to_obj", perf_start)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    if isinstance(body_dict, dict) and body_dict:
        _normalize_human_text_fields(body_dict)
        sidecar_start = replan_perf.start_timer()
        try:
            replan_request_transport_store.save_payload(body_dict)
        except Exception:
            pass
        finally:
            replan_perf.add_elapsed("common.replan_0902_push.sidecar_save_call", sidecar_start)
    # TX 화이트리스트가 있으면 최종 전송 전 선별(제너레이터가 풍부하게 만들어도 최소필드만 보냄)
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = _select_tx_fields(body_dict, wl)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0902] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0902] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    source = get_default_source_code()
    # DB 기반 메시지는 DB의 파일명(숫자).json을 ID로 사용하여 최소 필드만 전송
    if MSG_ID in DB_DIR_RULES:
        dbdir = _db_dir_for(MSG_ID, __file__)
        # 0304(유인기 pathID)는 1/2/3 시작만 전송(기존 규칙 유지)
        needs_prefix = "123" if MSG_ID == "0304" else None
        ids = _list_numeric_ids(dbdir, needs_prefix)
        logs = []
        for vid in ids:
            wl = TX_FIELD_WHITELIST.get(MSG_ID, [])
            body = {
                "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                "Source": source,
            }
            # ID 필드 결정
            if "inputMissionPackageID" in wl:          body["inputMissionPackageID"] = vid
            if "missionReferencePackageID" in wl:      body["missionReferencePackageID"] = vid
            if "missionPlanID" in wl:                  body["missionPlanID"] = vid
            if "individualMissionPackageID" in wl:     body["individualMissionPackageID"] = vid
            if "pathID" in wl:                         body["pathID"] = vid
            logs.append(make_and_push(body, node_messenger))
        return b"\n".join(logs) if logs else b""
    else:
        # 비 DB 메시지는 제너레이터 → 필요 시 화이트리스트로 선별
        body = make_msg0902_body()
        override_source_fields(body, source)
        # ★ 0102 방어: body가 비거나 dict가 아니면 최소 세트로 채움
        if MSG_ID == "0102":
            if not isinstance(body, dict) or not body:
                body = {
                    "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                    "status": 1,  # 정상
                    "Source": source,
                }
        wl = TX_FIELD_WHITELIST.get(MSG_ID)
        if wl and isinstance(body, dict):
            body = _select_tx_fields(body, wl)
        return make_and_push(body, node_messenger)

