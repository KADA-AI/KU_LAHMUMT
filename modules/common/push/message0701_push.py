# modules/common/push/message0701_push.py
# auto-generated at 2025-08-24T20:13:14.031500+00:00


import os, sys, json, importlib
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
COMMON_ROOT = os.path.normpath(os.path.join(HERE, ".."))
PROJECT_ROOT = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
for path in (COMMON_ROOT, PROJECT_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from System.Collections.Generic import List
from nFusion.Model.msg_0701 import *    # C# 모델(우선)
from nFusion.Model.CommonType import *     # 공통 타입(항상)
from System import Boolean, Int32, String, UInt32, UInt64
from generator.message0701_generator import make_msg0701_body
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
from modules.common.source_utils import get_default_source_code, override_source_fields
MSG_ID = "0701"
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

def _get_ci(data: dict, *names):
    lowered = {str(k).lower(): k for k in data.keys()}
    for name in names:
        key = lowered.get(str(name).lower())
        if key is not None:
            return data.get(key)
    return None

def _target_id_value(item):
    if isinstance(item, dict):
        item = _get_ci(item, "targetID")
        if item is None:
            return None
    try:
        value = int(item)
    except Exception:
        return None
    return value if value > 0 else None

def _target_id_items(data: dict) -> list[dict]:
    raw_list = _get_ci(data, "targetIDList", "targetIDs", "targetIds", "targetList")
    ids = []
    if isinstance(raw_list, list):
        for item in raw_list:
            value = _target_id_value(item)
            if value is not None and value not in ids:
                ids.append(value)
    if not ids:
        for key in ("targetID", "targetId", "target"):
            value = _target_id_value(_get_ci(data, key))
            if value is not None and value not in ids:
                ids.append(value)
    if not ids:
        count = _target_id_value(_get_ci(data, "targetCount"))
        if count is not None:
            ids.extend(range(1, count + 1))
    return [{"targetID": value} for value in ids]

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

def _dict_to_Option(data: dict):
    obj = _new('Option')
    if "optionID" in data: _try_set(obj, "optionID", int(data["optionID"]))
    if "recommend" in data: _try_set(obj, "recommend", bool(data["recommend"]))
    if "optionName" in data:
        value = data["optionName"]
        if value is not None:
            try:
                _try_set(obj, "optionName", int(value))
            except Exception:
                _try_set(obj, "optionName", value)
    if "missionPlanID" in data: _try_set(obj, "missionPlanID", int(data["missionPlanID"]))
    if "survivalRate" in data: _try_set(obj, "survivalRate", int(data["survivalRate"]))
    if "timeContraction" in data: _try_set(obj, "timeContraction", int(data["timeContraction"]))
    if "recogEffectiveness" in data: _try_set(obj, "recogEffectiveness", int(data["recogEffectiveness"]))
    if "fuelWarning" in data: _try_set(obj, "fuelWarning", int(data["fuelWarning"]))
    if "distance" in data: _try_set(obj, "distance", int(data["distance"]))
    target_items = _target_id_items(data)
    _try_set(obj, "targetIDListN", len(target_items))
    T = _cs('TargetID') or object
    lst = List[T]()
    for item in target_items:
        target_obj = _new('TargetID')
        _try_set(target_obj, "targetID", int(item["targetID"]))
        lst.Add(target_obj)
    _try_set(obj, "targetIDList", lst)
    return obj

def _dict_to_MissionPlanOptionInfo(data: dict):
    obj = _new('MissionPlanOptionInfo')
    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    val_src = data.get("source", data.get("source", data.get("Source", data.get("requestModuleName", ""))))
    if val_src != "":
        if not _try_set(obj, "source", str(val_src)):
            _try_set(obj, "Source", str(val_src))
    if "autoExecution" in data: _try_set(obj, "autoExecution", bool(data["autoExecution"]))
    if "optionList" in data and isinstance(data["optionList"], list):
        T = _cs('Option') or object
        lst = List[T]()
        for item in data["optionList"]: lst.Add(_dict_to_Option(item if isinstance(item, dict) else {}))
        _try_set(obj, "optionList", lst)
    return obj




def _dict_to_obj(body_dict: dict):
    return _dict_to_MissionPlanOptionInfo(body_dict)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    # TX 화이트리스트가 있으면 최종 전송 전 선별(제너레이터가 풍부하게 만들어도 최소필드만 보냄)
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = _select_tx_fields(body_dict, wl)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0701] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0701] PUSH 완료"
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
        body = make_msg0701_body()
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

