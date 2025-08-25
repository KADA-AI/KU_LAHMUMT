# modules/common/receive/message0902_receiver.py
# auto-generated at 2025-08-24T16:37:13.120885+00:00

from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0902 import *            # C# 모델
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

def _to_dict_ReplanRequestTime(obj):
    d = {}
    _v = _get(obj, 'replanRequestTimestamp', 'ReplanRequestTimestamp')
    if _v is not None: d['replanRequestTimestamp'] = int(_v)
    return d

def _to_dict_InputMissionID(obj):
    d = {}
    _v = _get(obj, 'inputMissionID', 'InputMissionID')
    if _v is not None: d['inputMissionID'] = int(_v)
    return d

def _to_dict_IndividualMissionID(obj):
    d = {}
    _v = _get(obj, 'individualMissionID', 'IndividualMissionID')
    if _v is not None: d['individualMissionID'] = int(_v)
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

def _to_dict_CoordinateOrientation(obj):
    d = {}
    _sub = _get(obj, 'coordinate', 'Coordinate')
    if _sub is not None: d['coordinate'] = _to_dict_Coordinate(_sub)
    return d

def _to_dict_TargetOrientation(obj):
    d = {}
    _v = _get(obj, 'targetID', 'TargetID')
    if _v is not None: d['targetID'] = int(_v)
    return d

def _to_dict_PriorMission(obj):
    d = {}
    _v = _get(obj, 'priorMissionID', 'PriorMissionID')
    if _v is not None: d['priorMissionID'] = int(_v)
    _v = _get(obj, 'missionType', 'MissionType')
    if _v is not None: d['missionType'] = int(_v)
    _sub = _get(obj, 'coordinateOrientation', 'CoordinateOrientation')
    if _sub is not None: d['coordinateOrientation'] = _to_dict_CoordinateOrientation(_sub)
    _sub = _get(obj, 'targetOrientation', 'TargetOrientation')
    if _sub is not None: d['targetOrientation'] = _to_dict_TargetOrientation(_sub)
    return d

def _to_dict_PendingOption(obj):
    d = {}
    _v = _get(obj, 'optionID', 'OptionID')
    if _v is not None: d['optionID'] = int(_v)
    _v = _get(obj, 'optionName', 'OptionName')
    if _v is not None and _v != '': d['optionName'] = str(_v)
    _v = _get(obj, 'missionPlanID', 'MissionPlanID')
    if _v is not None: d['missionPlanID'] = int(_v)
    return d

def _to_dict_ReplanRequest(obj):
    d = {}
    _v = _get(obj, 'timestamp', 'Timestamp')
    if _v is not None: d['timestamp'] = int(_v)
    _sval = _get(obj, 'source', 'Source', 'source','Source','sourceModuleName','SourceModuleName','requestModuleName','RequestModuleName')
    if _sval is not None and _sval != '': d['source'] = str(_sval)
    _sub = _get(obj, 'replanRequestTime', 'ReplanRequestTime')
    if _sub is not None: d['replanRequestTime'] = _to_dict_ReplanRequestTime(_sub)
    _v = _get(obj, 'replanLevel', 'ReplanLevel')
    if _v is not None: d['replanLevel'] = int(_v)
    _coll = _get(obj, 'inputMissionIDList', 'InputMissionIDList') or []
    if _coll:
        d['inputMissionIDList'] = [_to_dict_InputMissionID(it) for it in _coll]
    _coll = _get(obj, 'individualMissionIDList', 'IndividualMissionIDList') or []
    if _coll:
        d['individualMissionIDList'] = [_to_dict_IndividualMissionID(it) for it in _coll]
    _coll = _get(obj, 'priorMissionList', 'PriorMissionList') or []
    if _coll:
        d['priorMissionList'] = [_to_dict_PriorMission(it) for it in _coll]
    _v = _get(obj, 'replanReason', 'ReplanReason')
    if _v is not None and _v != '': d['replanReason'] = str(_v)
    _coll = _get(obj, 'pendingOptionList', 'PendingOptionList') or []
    if _coll:
        d['pendingOptionList'] = [_to_dict_PendingOption(it) for it in _coll]
    return d

class ReplanRequestReceiver_0902(IFusionReceive[ReplanRequest], IsLocal, IsSingletone):
    """0902 ReplanRequest 메시지 수신 리시버"""
    __namespace__ = "ReplanRequestReceiver_0902"

    def Receive(self, data: ReplanRequest, src):
        try:
            _try_save_received('0902', data)

            body = _try_read_db_body('0902', data)
            if body is None:
                body = _to_dict_ReplanRequest(data)

            notify("0902", json.dumps(body, ensure_ascii=False).encode("utf-8","ignore"))

        except Exception:
            print("[ERROR][Receive-0902] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
