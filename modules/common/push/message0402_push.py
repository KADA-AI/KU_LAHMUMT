# modules/common/push/message0402_push.py
# rewritten at 2025-09-18

import json, importlib
from datetime import datetime, timezone
from System.Collections.Generic import List
from System import Int32, Single, UInt32
from nFusion.Model.msg_0402 import *       # C# 모델(우선)
from nFusion.Model.CommonType import *     # 공통 타입(항상)
from generator.message0402_generator import make_msg0402_body

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
from modules.common.source_utils import get_default_source_code, override_source_fields
MSG_ID = "0402"

# ──────────────────────────────────────────────────────────────────────────
# 공용 유틸
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
    if t is not None:
        return t
    for modname in (f"nFusion.Model.msg_{MSG_ID}", "nFusion.Model.CommonType", "nFusion.Model"):
        try:
            mod = importlib.import_module(modname)
            t = getattr(mod, name, None)
            if t is not None:
                return t
        except Exception:
            pass
    return None

def _new(name: str):
    t = _cs(name)
    if t is None:
        raise NameError(f"type not found: {name}")
    return t()

def _to_bool(v):
    if isinstance(v, bool):
        return v
    if v is None:
        return False
    if isinstance(v, (int, float)):
        return v != 0
    s = str(v).strip().lower()
    return s in ("1", "true", "t", "y", "yes", "on")

# ──────────────────────────────────────────────────────────────────────────
# (레거시와 호환을 위한 TX/DB 규칙 - 0402는 화이트리스트/DB 규칙 없음)
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
    "0303": "UAVFlightPlan",   # ← push/receive 모두 맞춤
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
    sm = _get("Source")
    rq = _get("requestModuleName") or _get("requestmodulename")
    src_val = s or sm or rq
    if src_val:
        out["Source"] = str(src_val)

    for f in fields:
        if f in ("timestamp", "source", "Source", "requestModuleName"):
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

# ──────────────────────────────────────────────────────────────────────────
# 0402 변환기 (스키마 준수)
def _dict_to_Coordinate(data: dict):
    obj = _new("Coordinate")
    if data is None:
        return obj
    if "latitude"  in data: _try_set(obj, "latitude",  float(data["latitude"]))
    if "longitude" in data: _try_set(obj, "longitude", float(data["longitude"]))
    if "altitude"  in data: _try_set(obj, "altitude",  int(data["altitude"]))
    return obj

def _dict_to_Watcher(data: dict):
    obj = _new("Watcher")
    if data is None:
        return obj
    if "aircraftID" in data: _try_set(obj, "aircraftID", int(data["aircraftID"]))
    return obj

def _dict_to_Target(data: dict):
    obj = _new("Target")
    if data is None:
        return obj
    if "targetID"      in data: _try_set(obj, "targetID",      int(data["targetID"]))
    if "targetType"    in data: _try_set(obj, "targetType",    int(data["targetType"]))
    if "coordinate"    in data and isinstance(data["coordinate"], dict):
        _try_set(obj, "coordinate", _dict_to_Coordinate(data["coordinate"]))
    if "watcher"       in data and isinstance(data["watcher"], dict):
        _try_set(obj, "watcher", _dict_to_Watcher(data["watcher"]))
    if "targetInFrame" in data: _try_set(obj, "targetInFrame", _to_bool(data["targetInFrame"]))
    if "isDestroyed"   in data: _try_set(obj, "isDestroyed",   _to_bool(data["isDestroyed"]))
    if "threat"        in data: _try_set(obj, "threat",        float(data["threat"]))
    return obj

def _dict_to_ROIInfo(data: dict):
    obj = _new("ROIInfo")
    if data is None:
        return obj
    if "aircraftID" in data: _try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "coordinate" in data and isinstance(data["coordinate"], dict):
        _try_set(obj, "coordinate", _dict_to_Coordinate(data["coordinate"]))
    if "fov" in data: _try_set(obj, "fov", float(data["fov"]))
    return obj

def _ensure_src_ts(body_dict: dict):
    """timestamp/source 기본값 보강"""
    if not isinstance(body_dict, dict):
        return {"timestamp": _now_ms(), "source": "DSC"}
    out = dict(body_dict)
    # timestamp
    ts = out.get("timestamp") or out.get("Timestamp")
    if ts is None:
        out["timestamp"] = _now_ms()
    else:
        out["timestamp"] = int(ts)
    # source
    src = out.get("source") or out.get("Source") or out.get("requestModuleName") or out.get("requestmodulename")
    out["source"] = str(src) if src is not None else "DSC"
    return out

def _dict_to_SituationAwarenessInfo(body_dict: dict):
    """0402 최상위 객체 생성"""
    body = _ensure_src_ts(body_dict)
    obj = _new("SituationAwarenessInfo")
    _try_set(obj, "timestamp", int(body.get("timestamp")))
    _try_set(obj, "source",    str(body.get("source")))

    # roiInfo (키 변형 허용)
    roi = None
    for k in ("roiInfo", "ROIInfo", "roiinfo"):
        if k in body and isinstance(body[k], dict):
            roi = body[k]; break
    _try_set(obj, "roiInfo", _dict_to_ROIInfo(roi) if roi else _new("ROIInfo"))

    # targetList
    TargetT = _cs("Target")
    target_list = List[TargetT]() if TargetT is not None else List[object]()
    raw_targets = None
    for k in ("targetList", "TargetList", "targets", "target"):
        if k in body:
            raw_targets = body[k]
            break
    if isinstance(raw_targets, dict):
        # 단일 객체도 허용
        raw_targets = [raw_targets]
    if isinstance(raw_targets, (list, tuple)):
        for t in raw_targets:
            if isinstance(t, dict):
                target_list.Add(_dict_to_Target(t))
    _try_set(obj, "targetList", target_list)
    return obj

# ──────────────────────────────────────────────────────────────────────────
# 엔드포인트
def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """딕셔너리를 C# 모델로 변환 후 Push"""
    # 0402는 화이트리스트 적용 대상 아님 (전체 스키마 전송)
    msg = _dict_to_SituationAwarenessInfo(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0402] BODY  : {json.dumps(_ensure_src_ts(body_dict or {}), ensure_ascii=False)}\n"
        f"[0402] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    source = get_default_source_code()
    """제너레이터 기반 랜덤 메시지 생성 → 실패 시 최소 바디로 폴백"""
    try:
        body = make_msg0402_body()
        override_source_fields(body, source)
    except Exception:
        body = None
    # 폴백(최소 세트)
    if not isinstance(body, dict) or not body:
        body = {
            "timestamp": _now_ms(),
            "source": source,
            "roiInfo": {
                "aircraftID": 0,
                "coordinate": {"latitude": 0.0, "longitude": 0.0, "altitude": 0},
                "fov": 0.0,
            },
            "targetList": [],
        }
    body = _ensure_src_ts(body)
    return make_and_push(body, node_messenger)

