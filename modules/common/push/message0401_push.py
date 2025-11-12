# modules/common/push/message0401_push.py
# auto-generated at 2025-08-24T20:13:14.019443+00:00


import json, importlib
from datetime import datetime, timezone
from System.Collections.Generic import List
from System import Array
from nFusion.Model.msg_0401 import *    # C# 모델(우선)
from nFusion.Model.CommonType import *     # 공통 타입(항상)
from System import Boolean, Int32, Single, String, UInt32, UInt64
from generator.message0401_generator import make_msg0401_body
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
MSG_ID = "0401"
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

def _dict_to_Coordinate(data: dict):
    obj = _new('Coordinate')
    if not isinstance(data, dict):
        return obj
    lat = data.get("latitude")
    if lat is not None: _try_set(obj, "latitude", float(lat))
    lon = data.get("longitude")
    if lon is not None: _try_set(obj, "longitude", float(lon))
    alt = data.get("altitude")
    if alt is not None: _try_set(obj, "altitude", int(alt))
    return obj

def _dict_to_Velocity(data: dict):
    obj = _new('Velocity')
    if not isinstance(data, dict):
        return obj
    speed = data.get("speed")
    if speed is not None: _try_set(obj, "speed", float(speed))
    heading = data.get("heading")
    if heading is not None: _try_set(obj, "heading", float(heading))
    return obj

def _dict_to_Weapons(data: dict):
    obj = _new('Weapons')
    if not isinstance(data, dict):
        return obj
    type1 = data.get("type1")
    if type1 is not None: _try_set(obj, "type1", int(type1))
    type2 = data.get("type2")
    if type2 is not None: _try_set(obj, "type2", int(type2))
    type3 = data.get("type3")
    if type3 is not None: _try_set(obj, "type3", int(type3))
    return obj

def _dict_to_DatalinkStatus(data: dict):
    obj = _new('DatalinkStatus')
    if not isinstance(data, dict):
        return obj
    c1 = data.get("isConnectedToUAV1")
    if c1 is not None: _try_set(obj, "isConnectedToUAV1", bool(c1))
    c2 = data.get("isConnectedToUAV2")
    if c2 is not None: _try_set(obj, "isConnectedToUAV2", bool(c2))
    c3 = data.get("isConnectedToUAV3")
    if c3 is not None: _try_set(obj, "isConnectedToUAV3", bool(c3))
    return obj

def _dict_to_MannedInfo(data: dict):
    obj = _new('MannedInfo')
    if "weapons" in data and isinstance(data["weapons"], dict):
        _try_set(obj, "weapons", _dict_to_Weapons(data["weapons"]))
    if "datalinkStatus" in data and isinstance(data["datalinkStatus"], dict):
        _try_set(obj, "datalinkStatus", _dict_to_DatalinkStatus(data["datalinkStatus"]))
    return obj

def _dict_to_CurrentWaypointID(data: dict):
    obj = _new('CurrentWaypointID')
    if not isinstance(data, dict):
        return obj
    waypoint = data.get("waypointID")
    if waypoint is not None: _try_set(obj, "waypointID", int(waypoint))
    return obj

def _dict_to_LoiterCoordinate(data: dict):
    obj = _new('LoiterCoordinate')
    if not isinstance(data, dict):
        return obj
    lat = data.get("latitude")
    if lat is not None: _try_set(obj, "latitude", float(lat))
    lon = data.get("longitude")
    if lon is not None: _try_set(obj, "longitude", float(lon))
    alt = data.get("altitude")
    if alt is not None: _try_set(obj, "altitude", int(alt))
    return obj

def _dict_to_TargetFollowing(data: dict):
    obj = _new('TargetFollowing')
    if not isinstance(data, dict):
        return obj
    target_id = data.get("targetID")
    if target_id is not None: _try_set(obj, "targetID", int(target_id))
    return obj

def _dict_to_LeaderAircraftID(data: dict):
    obj = _new('LeaderAircraftID')
    if not isinstance(data, dict):
        return obj
    aircraft = data.get("aircraftID")
    if aircraft is not None: _try_set(obj, "aircraftID", int(aircraft))
    return obj

def _dict_to_CenterCoordinate(data: dict):
    obj = _new('CenterCoordinate')
    if not isinstance(data, dict):
        return obj
    lat = data.get("latitude")
    if lat is not None: _try_set(obj, "latitude", float(lat))
    lon = data.get("longitude")
    if lon is not None: _try_set(obj, "longitude", float(lon))
    alt = data.get("altitude")
    if alt is not None: _try_set(obj, "altitude", int(alt))
    return obj

def _dict_to_FootprintCorner(data: dict):
    obj = _new('FootprintCorner')
    if not isinstance(data, dict):
        return obj
    lat = data.get("latitude")
    if lat is not None: _try_set(obj, "latitude", float(lat))
    lon = data.get("longitude")
    if lon is not None: _try_set(obj, "longitude", float(lon))
    alt = data.get("altitude")
    if alt is not None: _try_set(obj, "altitude", int(alt))
    return obj

def _first_key_ci(d: dict, *candidates: str):
    """dict에서 대/소문자 구분 없이 첫 번째로 매칭되는 키의 값을 돌려준다."""
    if not isinstance(d, dict):
        return None
    low = {k.lower(): k for k in d.keys()}
    for c in candidates:
        if c is None: 
            continue
        # exact / PascalCase 변형 모두 시도
        for probe in (c, c[:1].upper()+c[1:] if c else c):
            if probe is None: 
                continue
            k = low.get(probe.lower())
            if k is not None:
                return d[k]
    return None

def _make_cs_list_or_array(elem_type, items_py):
    """elem_type(T)에 대해 List[T] -> Array[T] -> fallback(python list) 순으로 컬렉션을 만든다."""
    if elem_type is None:
        return items_py  # T를 못 찾으면 최후수단
    # 1) C# List[T]
    try:
        lst = List[elem_type]()
        for it in items_py:
            lst.Add(it)
        return lst
    except Exception:
        pass
    # 2) .NET Array[T]
    try:
        return Array[elem_type](items_py)
    except Exception:
        pass
    # 3) 파이썬 리스트
    return items_py

def _materialize_corners_as(elem_type_name: str, raw_list):
    """elem_type_name: 'FootprintCorner' 또는 'Coordinate'로 요소를 생성해 컬렉션으로 만든다."""
    T = _cs(elem_type_name)
    items = []
    for item in (raw_list or []):
        if isinstance(item, dict):
            if elem_type_name == 'FootprintCorner':
                items.append(_dict_to_FootprintCorner(item))
            else:  # 'Coordinate'
                items.append(_dict_to_Coordinate(item))
        else:
            # 이미 .NET 객체일 수 있으니 그대로
            items.append(item)
    return _make_cs_list_or_array(T, items)

def _try_set_any(obj, names, value):
    """여러 프로퍼티 이름 후보에 대해 순차 시도."""
    for name in names:
        if _try_set(obj, name, value):
            return True
    return False


def _set_footprint_list(obj, items: list[dict]) -> bool:
    # 후보 속성명(소문자/파스칼 전부 시도)
    props = ('footprintCornerList','FootprintCornerList',
             'footprintCorners','FootprintCorners',
             'footprintCorner','FootprintCorner')

    # (요소 타입명, 변환함수) 후보
    candidates = (
        ('FootprintCorner', _dict_to_FootprintCorner),
        ('Coordinate',      _dict_to_Coordinate),
    )

    # 1) C# List<T> 우선
    for tname, conv in candidates:
        T = _cs(tname)
        if not T:
            continue
        try:
            lst = List[T]()
            for it in items:
                lst.Add(conv(it if isinstance(it, dict) else {}))
            for p in props:
                if _try_set(obj, p, lst):
                    return True
        except Exception:
            pass

    # 2) Array<T>도 시도
    for tname, conv in candidates:
        T = _cs(tname)
        if not T:
            continue
        try:
            arr = Array[T]([conv(it if isinstance(it, dict) else {}) for it in items])
            for p in props:
                if _try_set(obj, p, arr):
                    return True
        except Exception:
            pass

    # 3) 최후의 수단: 파이썬 리스트 그대로
    py_items = [ _dict_to_FootprintCorner(it if isinstance(it, dict) else {}) for it in items ]
    for p in props:
        if _try_set(obj, p, py_items):
            return True
    return False

def _dict_to_SensorInfo(data: dict):
    obj = _new('SensorInfo')
    if not isinstance(data, dict):
        return obj

    operational_mode = data.get("operationalMode")
    if operational_mode is not None: _try_set(obj, "operationalMode", int(operational_mode))
    sensor_type = data.get("sensorType")
    if sensor_type is not None: _try_set(obj, "sensorType", int(sensor_type))
    fov = data.get("fov")
    if fov is not None: _try_set(obj, "fov", float(fov))

    if "centerCoordinate" in data and isinstance(data["centerCoordinate"], dict):
        _try_set(obj, "centerCoordinate", _dict_to_CenterCoordinate(data["centerCoordinate"]))

    if "footprintCornerList" in data and isinstance(data["footprintCornerList"], list):
        _set_footprint_list(obj, data["footprintCornerList"])

    return obj

def _dict_to_UnmannedInfo(data: dict):
    obj = _new('UnmannedInfo')
    if not isinstance(data, dict):
        return obj
    if "currentWaypointID" in data and isinstance(data["currentWaypointID"], dict):
        _try_set(obj, "currentWaypointID", _dict_to_CurrentWaypointID(data["currentWaypointID"]))
    flight_mode = data.get("flightMode")
    if flight_mode is not None: _try_set(obj, "flightMode", int(flight_mode))
    if "loiterCoordinate" in data and isinstance(data["loiterCoordinate"], dict):
        _try_set(obj, "loiterCoordinate", _dict_to_LoiterCoordinate(data["loiterCoordinate"]))
    if "targetFollowing" in data and isinstance(data["targetFollowing"], dict):
        _try_set(obj, "targetFollowing", _dict_to_TargetFollowing(data["targetFollowing"]))
    if "leaderAircraftID" in data and isinstance(data["leaderAircraftID"], dict):
        _try_set(obj, "leaderAircraftID", _dict_to_LeaderAircraftID(data["leaderAircraftID"]))
    if "sensorInfo" in data and isinstance(data["sensorInfo"], dict):
        _try_set(obj, "sensorInfo", _dict_to_SensorInfo(data["sensorInfo"]))
    payload_health = data.get("payloadHealth")
    if payload_health is not None: _try_set(obj, "payloadHealth", int(payload_health))
    fuel_warning = data.get("fuelWarning")
    if fuel_warning is not None: _try_set(obj, "fuelWarning", int(fuel_warning))
    return obj

def _dict_to_AgentState(data: dict):
    obj = _new('AgentState')
    if not isinstance(data, dict):
        return obj
    aircraft = data.get("aircraftID")
    if aircraft is not None: _try_set(obj, "aircraftID", int(aircraft))
    is_unmanned = data.get("isUnmanned")
    if is_unmanned is not None: _try_set(obj, "isUnmanned", bool(is_unmanned))
    if "coordinate" in data and isinstance(data["coordinate"], dict):
        _try_set(obj, "coordinate", _dict_to_Coordinate(data["coordinate"]))
    if "velocity" in data and isinstance(data["velocity"], dict):
        _try_set(obj, "velocity", _dict_to_Velocity(data["velocity"]))
    fuel = data.get("fuel")
    if fuel is not None: _try_set(obj, "fuel", float(fuel))
    health = data.get("health")
    if health is not None: _try_set(obj, "health", int(health))
    if "mannedInfo" in data and isinstance(data["mannedInfo"], dict):
        _try_set(obj, "mannedInfo", _dict_to_MannedInfo(data["mannedInfo"]))
    if "unmannedInfo" in data and isinstance(data["unmannedInfo"], dict):
        _try_set(obj, "unmannedInfo", _dict_to_UnmannedInfo(data["unmannedInfo"]))
    return obj

def _dict_to_AgentStatus(data: dict):
    obj = _new('AgentStatus')
    if not isinstance(data, dict):
        return obj
    timestamp = data.get("timestamp")
    if timestamp is not None: _try_set(obj, "timestamp", int(timestamp))
    val_src = data.get("source", data.get("source", data.get("Source", data.get("requestModuleName", ""))))
    if val_src != "":
        if not _try_set(obj, "source", str(val_src)):
            _try_set(obj, "Source", str(val_src))
    if "agentStateList" in data and isinstance(data["agentStateList"], list):
        T = _cs('AgentState') or object
        lst = List[T]()
        for item in data["agentStateList"]: lst.Add(_dict_to_AgentState(item if isinstance(item, dict) else {}))
        _try_set(obj, "agentStateList", lst)
    return obj




def _dict_to_obj(body_dict: dict):
    return _dict_to_AgentStatus(body_dict)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    # TX 화이트리스트가 있으면 최종 전송 전 선별(제너레이터가 풍부하게 만들어도 최소필드만 보냄)
    wl = TX_FIELD_WHITELIST.get(MSG_ID)
    if wl and isinstance(body_dict, dict):
        body_dict = _select_tx_fields(body_dict, wl)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0401] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0401] PUSH 완료"
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
                "Source": "DSC",
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
        body = make_msg0401_body()
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
