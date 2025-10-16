# modules/common/receive/message0402_receiver.py
# rewritten at 2025-09-18

from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0402 import *              # C# 모델
from nFusion.Model.CommonType import *            # 공통 타입
from .database import received_db
from receive_center import notify
import json, traceback, sys, os

# 대/소문자 안전 접근
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ── Embedded rules (TX/DB 공용: 0402는 DB fetch 없음) ─────────────────────
TX_FIELD_WHITELIST  = {'0201': ['timestamp', 'inputMissionPackageID'],
                       '0203': ['timestamp', 'missionReferencePackageID'],
                       '0301': ['timestamp', 'missionPlanID'],
                       '0302': ['timestamp', 'individualMissionPackageID'],
                       '0303': ['timestamp', 'pathID'],
                       '0304': ['timestamp', 'pathID']}
DB_DIR_RULES        = {'0201': 'InputMissionPlan',
                       '0203': 'FlightReferenceInfo',
                       '0301': 'MissionPlan',
                       '0302': 'IndividualMissionPlan',
                       '0303': 'UAVFlightPlan',     # ← push와 통일
                       '0304': 'FlightPath'}
DB_FETCH_ON_RECEIVE = {'0201', '0203'}
ID_FIELD_FOR        = {'0201': 'inputMissionPackageID',
                       '0203': 'missionReferencePackageID',
                       '0301': 'missionPlanID',
                       '0302': 'individualMissionPackageID',
                       '0303': 'pathID',
                       '0304': 'pathID'}

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

# ──────────────────────────────────────────────────────────────────────────
# 0402 변환기 (수신 C# 객체 → dict)
def _to_bool(v):
    if isinstance(v, bool):
        return v
    if v is None:
        return False
    if isinstance(v, (int, float)):
        return v != 0
    s = str(v).strip().lower()
    return s in ("1", "true", "t", "y", "yes", "on")

def _to_dict_Coordinate(obj):
    d = {}
    _v = _get(obj, 'latitude', 'Latitude')
    if _v is not None: d['latitude'] = float(_v)
    _v = _get(obj, 'longitude', 'Longitude')
    if _v is not None: d['longitude'] = float(_v)
    _v = _get(obj, 'altitude', 'Altitude')
    if _v is not None: d['altitude'] = int(_v)
    return d

def _to_dict_Watcher(obj):
    d = {}
    _v = _get(obj, 'aircraftID', 'AircraftID')
    if _v is not None: d['aircraftID'] = int(_v)
    return d

def _to_dict_Target(obj):
    d = {}
    _v = _get(obj, 'targetID', 'TargetID')
    if _v is not None: d['targetID'] = int(_v)
    _v = _get(obj, 'targetType', 'TargetType')
    if _v is not None: d['targetType'] = int(_v)
    _sub = _get(obj, 'coordinate', 'Coordinate')
    if _sub is not None: d['coordinate'] = _to_dict_Coordinate(_sub)
    _sub = _get(obj, 'watcher', 'Watcher')
    if _sub is not None: d['watcher'] = _to_dict_Watcher(_sub)
    _v = _get(obj, 'targetInFrame', 'TargetInFrame')
    if _v is not None: d['targetInFrame'] = _to_bool(_v)
    _v = _get(obj, 'isDestroyed', 'IsDestroyed')
    if _v is not None: d['isDestroyed'] = _to_bool(_v)
    _v = _get(obj, 'threat', 'Threat')
    if _v is not None: d['threat'] = float(_v)
    return d

def _to_dict_ROIInfo(obj):
    d = {}
    _v = _get(obj, 'aircraftID', 'AircraftID')
    if _v is not None: d['aircraftID'] = int(_v)
    _sub = _get(obj, 'coordinate', 'Coordinate')
    if _sub is not None: d['coordinate'] = _to_dict_Coordinate(_sub)
    _v = _get(obj, 'fov', 'Fov')
    if _v is not None: d['fov'] = float(_v)
    return d

def _to_dict_SituationAwarenessInfo(obj):
    d = {}
    # top-level
    _v = _get(obj, 'timestamp', 'Timestamp')
    if _v is not None: d['timestamp'] = int(_v)
    _v = _get(obj, 'source', 'Source')
    if _v is not None: d['source'] = str(_v)

    # roiInfo
    _sub = _get(obj, 'roiInfo', 'ROIInfo', 'roiinfo')
    if _sub is not None:
        d['roiInfo'] = _to_dict_ROIInfo(_sub)

    # targetList
    tgt_list = []
    _lst = _get(obj, 'targetList', 'TargetList', 'targets')
    if _lst is not None:
        try:
            for it in _lst:
                tgt_list.append(_to_dict_Target(it))
        except TypeError:
            # 단일 객체일 가능성
            tgt_list.append(_to_dict_Target(_lst))
    d['targetList'] = tgt_list
    return d

# ──────────────────────────────────────────────────────────────────────────
class SituationAwarenessInfoReceiver_0402(IFusionReceive[SituationAwarenessInfo], IsLocal, IsSingletone):
    """0402 SituationAwarenessInfo 메시지 수신 리시버"""
    __namespace__ = "SituationAwarenessInfoReceiver_0402"

    def Receive(self, data: SituationAwarenessInfo, src):
        try:
            _try_save_received('0402', data)

            body = _try_read_db_body('0402', data)   # 0402는 기본적으로 None
            if body is None:
                body = _to_dict_SituationAwarenessInfo(data)

            notify("0402", json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore"))

        except Exception:
            print("[ERROR][Receive-0402] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
