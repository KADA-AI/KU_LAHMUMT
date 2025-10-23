# modules/common/receive/message0602_receiver.py
# auto-generated at 2025-08-24T16:37:13.106869+00:00

from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0602 import *            # C# 모델
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
    return str(_project_root_for_recv_file(__file_path) / "database" / name)

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

def _to_dict_PathFollowing(obj):
    d = {}
    _v = _get(obj, 'startWaypointID', 'StartWaypointID')
    if _v is not None: d['startWaypointID'] = int(_v)
    return d

def _to_dict_TargetTracking(obj):
    d = {}
    _v = _get(obj, 'targetID', 'TargetID')
    if _v is not None: d['targetID'] = int(_v)
    return d

def _to_dict_Coordinate(obj):
    d = {}
    _v = _get(obj, 'latitude', 'Latitude')
    if _v is not None: d['latitude'] = float(_v)
    _v = _get(obj, 'longitude', 'Longitude')
    if _v is not None: d['longitude'] = float(_v)
    _v = _get(obj, 'altitude', 'Altitude')
    if _v is not None: d['altitude'] = int(_v)
    return d

def _to_dict_LoiterProperty(obj):
    d = {}
    _sub = _get(obj, 'coordinate', 'Coordinate')
    if _sub is not None: d['coordinate'] = _to_dict_Coordinate(_sub)
    _v = _get(obj, 'loiterTime', 'LoiterTime')
    if _v is not None: d['loiterTime'] = float(_v)
    _v = _get(obj, 'loiterRadius', 'LoiterRadius')
    if _v is not None: d['loiterRadius'] = float(_v)
    _v = _get(obj, 'loiterDirection', 'LoiterDirection')
    if _v is not None: d['loiterDirection'] = int(_v)
    _v = _get(obj, 'loiterSpeed', 'LoiterSpeed')
    if _v is not None: d['loiterSpeed'] = float(_v)
    return d

def _to_dict_Formation(obj):
    d = {}
    _v = _get(obj, 'dX', 'DX')
    if _v is not None: d['dX'] = int(_v)
    _v = _get(obj, 'dY', 'DY')
    if _v is not None: d['dY'] = int(_v)
    _v = _get(obj, 'dZ', 'DZ')
    if _v is not None: d['dZ'] = int(_v)
    return d

def _to_dict_FormationInfo(obj):
    d = {}
    _v = _get(obj, 'leaderAircraftID', 'LeaderAircraftID')
    if _v is not None: d['leaderAircraftID'] = int(_v)
    _sub = _get(obj, 'formation', 'Formation')
    if _sub is not None: d['formation'] = _to_dict_Formation(_sub)
    return d

def _to_dict_FlightModeCommand(obj):
    d = {}
    _v = _get(obj, 'flightMode', 'FlightMode')
    if _v is not None: d['flightMode'] = int(_v)
    _sub = _get(obj, 'pathFollowing', 'PathFollowing')
    if _sub is not None: d['pathFollowing'] = _to_dict_PathFollowing(_sub)
    _sub = _get(obj, 'targetTracking', 'TargetTracking')
    if _sub is not None: d['targetTracking'] = _to_dict_TargetTracking(_sub)
    _sub = _get(obj, 'loiterProperty', 'LoiterProperty')
    if _sub is not None: d['loiterProperty'] = _to_dict_LoiterProperty(_sub)
    _sub = _get(obj, 'formationInfo', 'FormationInfo')
    if _sub is not None: d['formationInfo'] = _to_dict_FormationInfo(_sub)
    return d

def _to_dict_CoordinateOrientation(obj):
    d = {}
    _sub = _get(obj, 'coordinate', 'Coordinate')
    if _sub is not None: d['coordinate'] = _to_dict_Coordinate(_sub)
    return d

def _to_dict_LineSearch(obj):
    d = {}
    _coll = _get(obj, 'coordinateList', 'CoordinateList') or []
    if _coll:
        d['coordinateList'] = [_to_dict_Coordinate(it) for it in _coll]
    _v = _get(obj, 'searchSpeed', 'SearchSpeed')
    if _v is not None: d['searchSpeed'] = float(_v)
    return d

def _to_dict_AutoTracking(obj):
    d = {}
    _v = _get(obj, 'targetID', 'TargetID')
    if _v is not None: d['targetID'] = int(_v)
    return d

def _to_dict_AircraftFixed(obj):
    d = {}
    _v = _get(obj, 'gimbalPitch', 'GimbalPitch')
    if _v is not None: d['gimbalPitch'] = float(_v)
    _v = _get(obj, 'gimbalYaw', 'GimbalYaw')
    if _v is not None: d['gimbalYaw'] = float(_v)
    return d

def _to_dict_GimbalYawLimits(obj):
    d = {}
    _v = _get(obj, 'leftLimit', 'LeftLimit')
    if _v is not None: d['leftLimit'] = float(_v)
    _v = _get(obj, 'rightLimit', 'RightLimit')
    if _v is not None: d['rightLimit'] = float(_v)
    return d

def _to_dict_AutoScan(obj):
    d = {}
    _v = _get(obj, 'gimbalPitch', 'GimbalPitch')
    if _v is not None: d['gimbalPitch'] = float(_v)
    _sub = _get(obj, 'gimbalYawLimits', 'GimbalYawLimits')
    if _sub is not None: d['gimbalYawLimits'] = _to_dict_GimbalYawLimits(_sub)
    _v = _get(obj, 'gimbalYawAngularSpeed', 'GimbalYawAngularSpeed')
    if _v is not None: d['gimbalYawAngularSpeed'] = float(_v)
    return d

def _to_dict_FilmingModeCommand(obj):
    d = {}
    _v = _get(obj, 'fieldOfView', 'FieldOfView')
    if _v is not None: d['fieldOfView'] = float(_v)
    _v = _get(obj, 'sensorType', 'SensorType')
    if _v is not None: d['sensorType'] = int(_v)
    _v = _get(obj, 'operationMode', 'OperationMode')
    if _v is not None: d['operationMode'] = int(_v)
    _sub = _get(obj, 'coordinateOrientation', 'CoordinateOrientation')
    if _sub is not None: d['coordinateOrientation'] = _to_dict_CoordinateOrientation(_sub)
    _sub = _get(obj, 'lineSearch', 'LineSearch')
    if _sub is not None: d['lineSearch'] = _to_dict_LineSearch(_sub)
    _sub = _get(obj, 'autoTracking', 'AutoTracking')
    if _sub is not None: d['autoTracking'] = _to_dict_AutoTracking(_sub)
    _sub = _get(obj, 'aircraftFixed', 'AircraftFixed')
    if _sub is not None: d['aircraftFixed'] = _to_dict_AircraftFixed(_sub)
    _sub = _get(obj, 'autoScan', 'AutoScan')
    if _sub is not None: d['autoScan'] = _to_dict_AutoScan(_sub)
    return d

def _to_dict_UAVCommand(obj):
    d = {}
    _v = _get(obj, 'timestamp', 'Timestamp')
    if _v is not None: d['timestamp'] = int(_v)
    _sval = _get(obj, 'source', 'Source', 'source','Source','Source','Source','requestModuleName','RequestModuleName')
    if _sval is not None and _sval != '': d['source'] = str(_sval)
    _v = _get(obj, 'uavCommandModeType', 'UavCommandModeType')
    if _v is not None: d['uavCommandModeType'] = int(_v)
    _v = _get(obj, 'aircraftID', 'AircraftID')
    if _v is not None: d['aircraftID'] = int(_v)
    _sub = _get(obj, 'flightModeCommand', 'FlightModeCommand')
    if _sub is not None: d['flightModeCommand'] = _to_dict_FlightModeCommand(_sub)
    _sub = _get(obj, 'filmingModeCommand', 'FilmingModeCommand')
    if _sub is not None: d['filmingModeCommand'] = _to_dict_FilmingModeCommand(_sub)
    return d

class UAVCommandReceiver_0602(IFusionReceive[UAVCommand], IsLocal, IsSingletone):
    """0602 UAVCommand 메시지 수신 리시버"""
    __namespace__ = "UAVCommandReceiver_0602"

    def Receive(self, data: UAVCommand, src):
        try:
            _try_save_received('0602', data)

            body = _try_read_db_body('0602', data)
            if body is None:
                body = _to_dict_UAVCommand(data)

            notify("0602", json.dumps(body, ensure_ascii=False).encode("utf-8","ignore"))

        except Exception:
            print("[ERROR][Receive-0602] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
