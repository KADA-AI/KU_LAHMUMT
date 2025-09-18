# modules/common/push/message0303_push.py
# auto-generated at 2025-08-24T20:13:14.013940+00:00


import json, importlib
from datetime import datetime, timezone
from System.Collections.Generic import List
from nFusion.Model.msg_0303 import *    # C# 모델(우선)
from nFusion.Model.CommonType import *     # 공통 타입(항상)
from System import Boolean, Int32, Single, UInt32, UInt64
from generator.message0303_generator import make_msg0303_body
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
MSG_ID = "0303"
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

def _dict_to_Formation(data: dict):
    obj = _new('Formation')
    if "dX" in data: _try_set(obj, "dX", int(data["dX"]))
    if "dY" in data: _try_set(obj, "dY", int(data["dY"]))
    if "dZ" in data: _try_set(obj, "dZ", int(data["dZ"]))
    return obj

def _dict_to_FormationInfo(data: dict):
    obj = _new('FormationInfo')
    if "leaderAircraftID" in data: _try_set(obj, "leaderAircraftID", int(data["leaderAircraftID"]))
    if "formation" in data and isinstance(data["formation"], dict):
        _try_set(obj, "formation", _dict_to_Formation(data["formation"]))
    return obj

def _dict_to_Coordinate(data: dict):
    obj = _new('Coordinate')
    if "latitude" in data: _try_set(obj, "latitude", float(data["latitude"]))
    if "longitude" in data: _try_set(obj, "longitude", float(data["longitude"]))
    if "altitude" in data: _try_set(obj, "altitude", int(data["altitude"]))
    return obj

def _dict_to_LoiterProperty(data: dict):
    obj = _new('LoiterProperty')
    if "radius" in data: _try_set(obj, "radius", int(data["radius"]))
    if "direction" in data: _try_set(obj, "direction", int(data["direction"]))
    if "time" in data: _try_set(obj, "time", int(data["time"]))
    if "speed" in data: _try_set(obj, "speed", float(data["speed"]))
    return obj

def _dict_to_CoordinateOrientation(data: dict):
    obj = _new('CoordinateOrientation')
    if "coordinate" in data and isinstance(data["coordinate"], dict):
        _try_set(obj, "coordinate", _dict_to_Coordinate(data["coordinate"]))
    return obj

def _dict_to_LineSearch(data: dict):
    obj = _new('LineSearch')
    if "coordinateList" in data and isinstance(data["coordinateList"], list):
        T = _cs('Coordinate') or object
        lst = List[T]()
        for item in data["coordinateList"]: lst.Add(_dict_to_Coordinate(item if isinstance(item, dict) else {}))
        _try_set(obj, "coordinateList", lst)
    if "searchSpeed" in data: _try_set(obj, "searchSpeed", float(data["searchSpeed"]))
    return obj

def _dict_to_AutoTracking(data: dict):
    obj = _new('AutoTracking')
    if "targetID" in data: _try_set(obj, "targetID", int(data["targetID"]))
    return obj

def _dict_to_AircraftFixed(data: dict):
    obj = _new('AircraftFixed')
    if "gimbalPitch" in data: _try_set(obj, "gimbalPitch", float(data["gimbalPitch"]))
    if "gimbalYaw" in data: _try_set(obj, "gimbalYaw", float(data["gimbalYaw"]))
    return obj

def _dict_to_GimbalYawLimits(data: dict):
    obj = _new('GimbalYawLimits')
    if "leftLimit" in data: _try_set(obj, "leftLimit", float(data["leftLimit"]))
    if "rightLimit" in data: _try_set(obj, "rightLimit", float(data["rightLimit"]))
    return obj

def _dict_to_AutoScan(data: dict):
    obj = _new('AutoScan')
    if "gimbalPitch" in data: _try_set(obj, "gimbalPitch", float(data["gimbalPitch"]))
    if "gimbalYawLimits" in data and isinstance(data["gimbalYawLimits"], dict):
        _try_set(obj, "gimbalYawLimits", _dict_to_GimbalYawLimits(data["gimbalYawLimits"]))
    if "gimbalYawAngularSpeed" in data: _try_set(obj, "gimbalYawAngularSpeed", float(data["gimbalYawAngularSpeed"]))
    return obj

def _dict_to_FilmingProperty(data: dict):
    obj = _new('FilmingProperty')
    if "fieldOfView" in data: _try_set(obj, "fieldOfView", float(data["fieldOfView"]))
    if "sensorType" in data: _try_set(obj, "sensorType", int(data["sensorType"]))
    if "operationMode" in data: _try_set(obj, "operationMode", int(data["operationMode"]))
    if "coordinateOrientation" in data and isinstance(data["coordinateOrientation"], dict):
        _try_set(obj, "coordinateOrientation", _dict_to_CoordinateOrientation(data["coordinateOrientation"]))
    if "lineSearch" in data and isinstance(data["lineSearch"], dict):
        _try_set(obj, "lineSearch", _dict_to_LineSearch(data["lineSearch"]))
    if "autoTracking" in data and isinstance(data["autoTracking"], dict):
        _try_set(obj, "autoTracking", _dict_to_AutoTracking(data["autoTracking"]))
    if "aircraftFixed" in data and isinstance(data["aircraftFixed"], dict):
        _try_set(obj, "aircraftFixed", _dict_to_AircraftFixed(data["aircraftFixed"]))
    if "autoScan" in data and isinstance(data["autoScan"], dict):
        _try_set(obj, "autoScan", _dict_to_AutoScan(data["autoScan"]))
    return obj

def _dict_to_Waypoint(data: dict):
    obj = _new('Waypoint')
    if "waypointID" in data: _try_set(obj, "waypointID", int(data["waypointID"]))
    if "coordinate" in data and isinstance(data["coordinate"], dict):
        _try_set(obj, "coordinate", _dict_to_Coordinate(data["coordinate"]))
    if "speed" in data: _try_set(obj, "speed", float(data["speed"]))
    if "eta" in data: _try_set(obj, "eta", int(data["eta"]))
    if "ecf" in data: _try_set(obj, "ecf", float(data["ecf"]))
    if "nextWaypointID" in data: _try_set(obj, "nextWaypointID", int(data["nextWaypointID"]))
    if "waypointPassType" in data: _try_set(obj, "waypointPassType", int(data["waypointPassType"]))
    if "loiterProperty" in data and isinstance(data["loiterProperty"], dict):
        _try_set(obj, "loiterProperty", _dict_to_LoiterProperty(data["loiterProperty"]))
    if "filmingProperty" in data and isinstance(data["filmingProperty"], dict):
        _try_set(obj, "filmingProperty", _dict_to_FilmingProperty(data["filmingProperty"]))
    return obj

def _dict_to_UAVFlightPlanData(data: dict):
    obj = _new('UAVFlightPlanData')
    # ★ source 계열 매핑
    for k in ("Source", "source", "requestModuleName"):
        if k in data: _try_set(obj, k, str(data[k]))

    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    if "pathID" in data: _try_set(obj, "pathID", int(data["pathID"]))
    if "aircraftID" in data: _try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "isFormationFlight" in data: _try_set(obj, "isFormationFlight", bool(data["isFormationFlight"]))
    if "formationInfo" in data and isinstance(data["formationInfo"], dict):
        _try_set(obj, "formationInfo", _dict_to_FormationInfo(data["formationInfo"]))
    if "waypointList" in data and isinstance(data["waypointList"], list):
        T = _cs('Waypoint') or object
        lst = List[T]()
        for item in data["waypointList"]:
            lst.Add(_dict_to_Waypoint(item if isinstance(item, dict) else {}))
        _try_set(obj, "waypointList", lst)
    return obj

def _dict_to_UAVFlightPlan(data: dict):
    obj = _new('UAVFlightPlan')
    # ★ source 계열 매핑
    for k in ("Source", "source", "requestModuleName"):
        if k in data: _try_set(obj, k, str(data[k]))

    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    if "pathID" in data: _try_set(obj, "pathID", int(data["pathID"]))
    if "aircraftID" in data: _try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "isFormationFlight" in data: _try_set(obj, "isFormationFlight", bool(data["isFormationFlight"]))
    if "formationInfo" in data and isinstance(data["formationInfo"], dict):
        _try_set(obj, "formationInfo", _dict_to_FormationInfo(data["formationInfo"]))
    if "waypointList" in data and isinstance(data["waypointList"], list):
        T = _cs('Waypoint') or object
        lst = List[T]()
        for item in data["waypointList"]:
            lst.Add(_dict_to_Waypoint(item if isinstance(item, dict) else {}))
        _try_set(obj, "waypointList", lst)
    return obj





def _dict_to_obj(body_dict: dict):
    return _dict_to_UAVFlightPlan(body_dict)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    # TX 화이트리스트가 있으면 최종 전송 전 선별(제너레이터가 풍부하게 만들어도 최소필드만 보냄)
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = _select_tx_fields(body_dict, wl)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0303] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0303] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    # DB 기반 메시지는 DB의 파일명(숫자).json을 ID로 사용하여 최소 필드만 전송
    if MSG_ID in DB_DIR_RULES:
        dbdir = _db_dir_for(MSG_ID, __file__)
        # ★ UAV(0303)는 4/5/6 시작만 전송
        needs_prefix = "456" if MSG_ID == "0303" else None
        ids = _list_numeric_ids(dbdir, needs_prefix)
        logs = []
        for vid in ids:
            wl = TX_FIELD_WHITELIST.get(MSG_ID, [])
            body = {
                "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                "Source": "DSC",
            }
            if "pathID" in wl:
                body["pathID"] = vid
            logs.append(make_and_push(body, node_messenger))
        return b"\n".join(logs) if logs else b""
    else:
        # 비 DB 메시지는 제너레이터 → 필요 시 화이트리스트로 선별
        body = make_msg0303_body()
        # ★ 0102 방어: body가 비거나 dict가 아니면 최소 세트로 채움
        if MSG_ID == "0102":
            if not isinstance(body, dict) or not body:
                body = {
                    "timestamp": int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000),
                    "status": 1,  # 정상
                    "Source": "DSC",
                }
        wl = TX_FIELD_WHITELIST.get(MSG_ID)
        if wl and isinstance(body, dict):
            body = _select_tx_fields(body, wl)
        return make_and_push(body, node_messenger)