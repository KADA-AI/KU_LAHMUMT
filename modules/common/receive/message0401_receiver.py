# modules/common/receive/message0401_receiver.py
# auto-generated at 2025-08-24T16:37:13.097866+00:00

from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0401 import *            # C# 모델
from nFusion.Model.CommonType import *             # 공통 타입
from .database import received_db
from receive_center import notify
import json, traceback, sys, os, importlib

# 대/소문자 안전 접근
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ── Embedded rules (TX/DB 공용) ──────────────────────────────────────────
TX_FIELD_WHITELIST = {'0201': ['timestamp', 'inputMissionPackageID'], '0203': ['timestamp', 'missionReferencePackageID'], '0301': ['timestamp', 'missionPlanID'], '0302': ['timestamp', 'individualMissionPackageID'], '0303': ['timestamp', 'pathID'], '0304': ['timestamp', 'pathID']}
DB_DIR_RULES        = {'0201': 'InputMissionPlan', '0203': 'FlightReferenceInfo', '0301': 'MissionPlan', '0302': 'IndividualMissionPlan', '0303': 'UAVFlightPlan', '0304': 'FlightPath'}
DB_FETCH_ON_RECEIVE = {'0201', '0203'}
ID_FIELD_FOR        = {'0201': 'inputMissionPackageID', '0203': 'missionReferencePackageID', '0301': 'missionPlanID', '0302': 'individualMissionPackageID', '0303': 'pathID', '0304': 'pathID'}

def _project_root_for_recv_file(__file_path: str):
    from pathlib import Path
    return Path(__file_path).resolve().parents[3]

def _db_dir_for(msgid: str, __file_path: str) -> str:
    from pathlib import Path
    env_root = os.getenv("KU_MISSION_DB_ROOT")
    name = DB_DIR_RULES.get(msgid)
    if not name:
        return str(_project_root_for_recv_file(__file_path))
    if env_root:
        return str(Path(env_root) / name)
    return str(_project_root_for_recv_file(__file_path) / "temp" / "database" / name)

def _try_save_received(msgid: str, data_obj):
    try:
        fn = getattr(received_db, f"set_received_{msgid}")
        fn(data_obj)
    except Exception:
        pass

def _try_read_db_body(msgid: str, data_obj):
    """DB_FETCH_ON_RECEIVE에 포함된 메시지는 ID 필드로 DB JSON을 찾아 반환(없으면 None)."""
    try:
        if msgid not in DB_FETCH_ON_RECEIVE:
            return None
        id_field = ID_FIELD_FOR.get(msgid)
        if not id_field:
            return None
        # 객체에서 ID 값을 추출(대/소문자 안전)
        _val = _get(data_obj, id_field, id_field[:1].upper()+id_field[1:])
        if _val is None:
            return None
        vid = int(_val)
        dbdir = _db_dir_for(msgid, __file__)
        fpath = os.path.join(dbdir, f"{vid}.json")
        print(f"[{msgid}] DB 참조! ({fpath})")
        if os.path.exists(fpath):
            with open(fpath, "r", encoding="utf-8") as f:
                return json.load(f)
        return None
    except Exception:
        return None

def _to_dict_Coordinate(obj):
    d = {}
    _v = _get(obj, 'latitude', 'Latitude')
    if _v is not None: d['latitude'] = float(_v)
    _v = _get(obj, 'longitude', 'Longitude')
    if _v is not None: d['longitude'] = float(_v)
    _v = _get(obj, 'altitude', 'Altitude')
    if _v is not None: d['altitude'] = int(_v)
    return d

def _to_dict_Velocity(obj):
    d = {}
    _v = _get(obj, 'speed', 'Speed')
    if _v is not None: d['speed'] = float(_v)
    _v = _get(obj, 'heading', 'Heading')
    if _v is not None: d['heading'] = float(_v)
    return d

def _to_dict_Weapons(obj):
    d = {}
    _v = _get(obj, 'type1', 'Type1')
    if _v is not None: d['type1'] = int(_v)
    _v = _get(obj, 'type2', 'Type2')
    if _v is not None: d['type2'] = int(_v)
    _v = _get(obj, 'type3', 'Type3')
    if _v is not None: d['type3'] = int(_v)
    return d

def _to_dict_DatalinkStatus(obj):
    d = {}
    _v = _get(obj, 'isConnectedToUAV1', 'IsConnectedToUAV1')
    if _v is not None: d['isConnectedToUAV1'] = bool(_v)
    _v = _get(obj, 'isConnectedToUAV2', 'IsConnectedToUAV2')
    if _v is not None: d['isConnectedToUAV2'] = bool(_v)
    _v = _get(obj, 'isConnectedToUAV3', 'IsConnectedToUAV3')
    if _v is not None: d['isConnectedToUAV3'] = bool(_v)
    return d

def _to_dict_MannedInfo(obj):
    d = {}
    _sub = _get(obj, 'weapons', 'Weapons')
    if _sub is not None: d['weapons'] = _to_dict_Weapons(_sub)
    _sub = _get(obj, 'datalinkStatus', 'DatalinkStatus')
    if _sub is not None: d['datalinkStatus'] = _to_dict_DatalinkStatus(_sub)
    return d

def _to_dict_CurrentWaypointID(obj):
    d = {}
    _v = _get(obj, 'waypointID', 'WaypointID')
    if _v is not None: d['waypointID'] = int(_v)
    return d

def _to_dict_LoiterCoordinate(obj):
    d = {}
    _v = _get(obj, 'latitude', 'Latitude')
    if _v is not None: d['latitude'] = float(_v)
    _v = _get(obj, 'longitude', 'Longitude')
    if _v is not None: d['longitude'] = float(_v)
    _v = _get(obj, 'altitude', 'Altitude')
    if _v is not None: d['altitude'] = int(_v)
    return d

def _to_dict_TargetFollowing(obj):
    d = {}
    _v = _get(obj, 'targetID', 'TargetID')
    if _v is not None: d['targetID'] = int(_v)
    return d

def _to_dict_LeaderAircraftID(obj):
    d = {}
    _v = _get(obj, 'aircraftID', 'AircraftID')
    if _v is not None: d['aircraftID'] = int(_v)
    return d

def _to_dict_CenterCoordinate(obj):
    d = {}
    _v = _get(obj, 'latitude', 'Latitude')
    if _v is not None: d['latitude'] = float(_v)
    _v = _get(obj, 'longitude', 'Longitude')
    if _v is not None: d['longitude'] = float(_v)
    _v = _get(obj, 'altitude', 'Altitude')
    if _v is not None: d['altitude'] = int(_v)
    return d

def _to_dict_FootprintCorner(obj):
    d = {}
    _v = _get(obj, 'latitude', 'Latitude')
    if _v is not None: d['latitude'] = float(_v)
    _v = _get(obj, 'longitude', 'Longitude')
    if _v is not None: d['longitude'] = float(_v)
    _v = _get(obj, 'altitude', 'Altitude')
    if _v is not None: d['altitude'] = int(_v)
    return d

def _to_dict_SensorInfo(obj):
    d = {}
    _v = _get(obj, 'operationalMode', 'OperationalMode')
    if _v is not None: d['operationalMode'] = int(_v)
    _v = _get(obj, 'sensorType', 'SensorType')
    if _v is not None: d['sensorType'] = int(_v)
    _v = _get(obj, 'filming', 'Filming')
    if _v is not None: d['filming'] = int(_v)
    _v = _get(obj, 'fov', 'Fov')
    if _v is not None: d['fov'] = float(_v)
    _sub = _get(obj, 'centerCoordinate', 'CenterCoordinate')
    if _sub is not None: d['centerCoordinate'] = _to_dict_CenterCoordinate(_sub)

    _corners = _get(
        obj,
        'footprintCornerList', 'FootprintCornerList',
        'footprintCorner', 'FootprintCorner',
        'footprintCorners', 'FootprintCorners',
    )

    if _corners is not None:
        try:
            iterable = list(_corners)
        except TypeError:
            iterable = [_corners]
        d['footprintCornerList'] = [_to_dict_FootprintCorner(it) for it in iterable]

    return d

def _to_dict_UnmannedInfo(obj):
    d = {}
    _sub = _get(obj, 'currentWaypointID', 'CurrentWaypointID')
    if _sub is not None: d['currentWaypointID'] = _to_dict_CurrentWaypointID(_sub)
    _v = _get(obj, 'flightMode', 'FlightMode')
    if _v is not None: d['flightMode'] = int(_v)
    _v = _get(obj, 'flying', 'Flying', 'onMission', 'OnMission')
    if _v is not None:
        d['flying'] = int(_v)
        d['onMission'] = int(_v)
    _sub = _get(obj, 'loiterCoordinate', 'LoiterCoordinate')
    if _sub is not None: d['loiterCoordinate'] = _to_dict_LoiterCoordinate(_sub)
    _sub = _get(obj, 'targetFollowing', 'TargetFollowing')
    if _sub is not None: d['targetFollowing'] = _to_dict_TargetFollowing(_sub)
    _sub = _get(obj, 'leaderAircraftID', 'LeaderAircraftID')
    if _sub is not None: d['leaderAircraftID'] = _to_dict_LeaderAircraftID(_sub)
    _sub = _get(obj, 'sensorInfo', 'SensorInfo')
    if _sub is not None: d['sensorInfo'] = _to_dict_SensorInfo(_sub)
    _v = _get(obj, 'payloadHealth', 'PayloadHealth')
    if _v is not None: d['payloadHealth'] = int(_v)
    _v = _get(obj, 'fuelWarning', 'FuelWarning')
    if _v is not None: d['fuelWarning'] = int(_v)
    return d

def _to_dict_AgentState(obj):
    d = {}
    _v = _get(obj, 'aircraftID', 'AircraftID')
    if _v is not None: d['aircraftID'] = int(_v)
    _v = _get(obj, 'isUnmanned', 'IsUnmanned')
    if _v is not None: d['isUnmanned'] = bool(_v)
    _sub = _get(obj, 'coordinate', 'Coordinate')
    if _sub is not None: d['coordinate'] = _to_dict_Coordinate(_sub)
    _sub = _get(obj, 'velocity', 'Velocity')
    if _sub is not None: d['velocity'] = _to_dict_Velocity(_sub)
    _v = _get(obj, 'fuel', 'Fuel')
    if _v is not None: d['fuel'] = float(_v)
    _v = _get(obj, 'health', 'Health')
    if _v is not None: d['health'] = int(_v)
    _v = _get(obj, 'lastSignalTime', 'LastSignalTime')
    if _v is not None: d['lastSignalTime'] = int(_v)
    _v = _get(obj, 'flying', 'Flying', 'onMission', 'OnMission')
    if _v is not None:
        d['flying'] = int(_v)
        d['onMission'] = int(_v)
    _sub = _get(obj, 'mannedInfo', 'MannedInfo')
    if _sub is not None: d['mannedInfo'] = _to_dict_MannedInfo(_sub)
    _sub = _get(obj, 'unmannedInfo', 'UnmannedInfo')
    if _sub is not None: d['unmannedInfo'] = _to_dict_UnmannedInfo(_sub)
    return d

def _to_dict_AgentStatus(obj):
    d = {}
    _v = _get(obj, 'timestamp', 'Timestamp')
    if _v is not None: d['timestamp'] = int(_v)
    _sval = _get(obj, 'source', 'Source', 'source','Source','Source','Source','requestModuleName','RequestModuleName')
    if _sval is not None and _sval != '': d['source'] = str(_sval)
    _coll = _get(obj, 'agentStateList', 'AgentStateList') or []
    if _coll:
        d['agentStateList'] = [_to_dict_AgentState(it) for it in _coll]
    return d

class AgentStatusReceiver_0401(IFusionReceive[AgentStatus], IsLocal, IsSingletone):
    """0401 AgentStatus 메시지 수신 리시버"""
    __namespace__ = "AgentStatusReceiver_0401"

    def Receive(self, data: AgentStatus, src):
        try:
            _try_save_received('0401', data)

            body = _try_read_db_body('0401', data)
            if body is None:
                body = _to_dict_AgentStatus(data)

            notify("0401", json.dumps(body, ensure_ascii=False).encode("utf-8","ignore"))

        except Exception:
            print("[ERROR][Receive-0401] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)

