# modules/common/push/message0302_push.py
# auto-generated at 2025-08-24T20:13:14.011940+00:00


import json, importlib
from datetime import datetime, timezone
from System.Collections.Generic import List
from nFusion.Model.msg_0302 import *    # C# 모델(우선)
from nFusion.Model.CommonType import *     # 공통 타입(항상)
from System import Boolean, Int32, Single, UInt32, UInt64
from generator.message0302_generator import make_msg0302_body
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
MSG_ID = "0302"
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
    sm = _get("sourceModuleName") or _get("sourcemodulename")
    rq = _get("requestModuleName") or _get("requestmodulename")
    src_val = s or sm or rq
    if src_val:
        out["sourceModuleName"] = str(src_val)

    for f in fields:
        if f in ("timestamp","source","sourceModuleName","requestModuleName"):
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
    return str(_project_root_for_push_file(__file_path) / "database" / name)

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

def _dict_to_RelatedMission(data: dict):
    obj = _new('RelatedMission')
    if "relatedMissionType" in data: _try_set(obj, "relatedMissionType", int(data["relatedMissionType"]))
    if "inputMissionID" in data: _try_set(obj, "inputMissionID", int(data["inputMissionID"]))
    if "priorMissionID" in data: _try_set(obj, "priorMissionID", int(data["priorMissionID"]))
    return obj

def _dict_to_Coordinate(data: dict):
    obj = _new('Coordinate')
    if "latitude" in data: _try_set(obj, "latitude", float(data["latitude"]))
    if "longitude" in data: _try_set(obj, "longitude", float(data["longitude"]))
    if "altitude" in data: _try_set(obj, "altitude", int(data["altitude"]))
    return obj

def _dict_to_Line(data: dict):
    obj = _new('Line')
    if "width" in data: _try_set(obj, "width", float(data["width"]))
    if "coordinateList" in data and isinstance(data["coordinateList"], list):
        T = _cs('Coordinate') or object
        lst = List[T]()
        for item in data["coordinateList"]: lst.Add(_dict_to_Coordinate(item if isinstance(item, dict) else {}))
        _try_set(obj, "coordinateList", lst)
    return obj

def _dict_to_Area(data: dict):
    obj = _new('Area')
    if "isHole" in data: _try_set(obj, "isHole", bool(data["isHole"]))
    if "coordinateList" in data and isinstance(data["coordinateList"], list):
        T = _cs('Coordinate') or object
        lst = List[T]()
        for item in data["coordinateList"]: lst.Add(_dict_to_Coordinate(item if isinstance(item, dict) else {}))
        _try_set(obj, "coordinateList", lst)
    return obj

def _dict_to_IndividualMissionInfo(data: dict):
    obj = _new('IndividualMissionInfo')
    if "individualMissionType" in data: _try_set(obj, "individualMissionType", int(data["individualMissionType"]))
    if "patternType" in data: _try_set(obj, "patternType", int(data["patternType"]))
    if "autoZoomIn" in data: _try_set(obj, "autoZoomIn", bool(data["autoZoomIn"]))
    if "coordinateList" in data and isinstance(data["coordinateList"], list):
        T = _cs('Coordinate') or object
        lst = List[T]()
        for item in data["coordinateList"]: lst.Add(_dict_to_Coordinate(item if isinstance(item, dict) else {}))
        _try_set(obj, "coordinateList", lst)
    if "lineList" in data and isinstance(data["lineList"], list):
        T = _cs('Line') or object
        lst = List[T]()
        for item in data["lineList"]: lst.Add(_dict_to_Line(item if isinstance(item, dict) else {}))
        _try_set(obj, "lineList", lst)
    if "areaList" in data and isinstance(data["areaList"], list):
        T = _cs('Area') or object
        lst = List[T]()
        for item in data["areaList"]: lst.Add(_dict_to_Area(item if isinstance(item, dict) else {}))
        _try_set(obj, "areaList", lst)
    if "targetID" in data: _try_set(obj, "targetID", int(data["targetID"]))
    return obj

def _dict_to_IndividualMission(data: dict):
    obj = _new('IndividualMission')
    if "individualMissionID" in data: _try_set(obj, "individualMissionID", int(data["individualMissionID"]))
    if "isDone" in data: _try_set(obj, "isDone", bool(data["isDone"]))
    if "relatedMission" in data and isinstance(data["relatedMission"], dict):
        _try_set(obj, "relatedMission", _dict_to_RelatedMission(data["relatedMission"]))
    if "individualMissionInfo" in data and isinstance(data["individualMissionInfo"], dict):
        _try_set(obj, "individualMissionInfo", _dict_to_IndividualMissionInfo(data["individualMissionInfo"]))
    if "pathID" in data: _try_set(obj, "pathID", int(data["pathID"]))
    return obj

def _dict_to_IndividualMissionPlanData(data: dict):
    obj = _new('IndividualMissionPlanData')
    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    if "individualMissionPackageID" in data: _try_set(obj, "individualMissionPackageID", int(data["individualMissionPackageID"]))
    if "aircraftID" in data: _try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "individualMissionList" in data and isinstance(data["individualMissionList"], list):
        T = _cs('IndividualMission') or object
        lst = List[T]()
        for item in data["individualMissionList"]: lst.Add(_dict_to_IndividualMission(item if isinstance(item, dict) else {}))
        _try_set(obj, "individualMissionList", lst)
    return obj

def _dict_to_IndividualMissionPlan(data: dict):
    obj = _new('IndividualMissionPlan')
    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    if "individualMissionPackageID" in data: _try_set(obj, "individualMissionPackageID", int(data["individualMissionPackageID"]))
    if "aircraftID" in data: _try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "individualMissionList" in data and isinstance(data["individualMissionList"], list):
        T = _cs('IndividualMission') or object
        lst = List[T]()
        for item in data["individualMissionList"]: lst.Add(_dict_to_IndividualMission(item if isinstance(item, dict) else {}))
        _try_set(obj, "individualMissionList", lst)
    return obj




def _dict_to_obj(body_dict: dict):
    return _dict_to_IndividualMissionPlan(body_dict)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    # TX 화이트리스트가 있으면 최종 전송 전 선별(제너레이터가 풍부하게 만들어도 최소필드만 보냄)
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = _select_tx_fields(body_dict, wl)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0302] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0302] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
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
                "sourceModuleName": "DSC",
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
        body = make_msg0302_body()
        # ★ 0102 방어: body가 비거나 dict가 아니면 최소 세트로 채움
        if MSG_ID == "0102":
            if not isinstance(body, dict) or not body:
                body = {
                    "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                    "status": 1,  # 정상
                    "sourceModuleName": "DSC",
                }
        wl = TX_FIELD_WHITELIST.get(MSG_ID)
        if wl and isinstance(body, dict):
            body = _select_tx_fields(body, wl)
        return make_and_push(body, node_messenger)
