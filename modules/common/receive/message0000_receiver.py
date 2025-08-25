# modules/common/receive/message0000_receiver.py
# auto-generated at 2025-08-24T16:37:13.075601+00:00

from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0000 import *            # C# 모델
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

def _to_dict_RequestData(obj):
    d = {}
    _v = _get(obj, 'timestamp', 'Timestamp')
    if _v is not None: d['timestamp'] = int(_v)
    _sval = _get(obj, 'source', 'Source', 'source','Source','sourceModuleName','SourceModuleName','requestModuleName','RequestModuleName')
    if _sval is not None and _sval != '': d['source'] = str(_sval)
    _v = _get(obj, 'messageID', 'MessageID')
    if _v is not None: d['messageID'] = int(_v)
    return d

def _to_dict_RequestData_min(obj):
    return {
        "timestamp": _get(obj, "timestamp","Timestamp"),
        "source":    _get(obj, "source","Source","requestModuleName","RequestModuleName","sourceModuleName","SourceModuleName"),
        "messageID": _get(obj, "messageID","MessageID"),
    }

class ResponseReceiver_0000(IFusionReceive[RequestData], IsLocal, IsSingletone):
    """0000 RequestData 메시지 수신 리시버"""
    __namespace__ = "ResponseReceiver_0000"

    def Receive(self, data: RequestData, src):
        try:
            _try_save_received('0000', data)

            body = _try_read_db_body('0000', data)
            if body is None:
                body = _to_dict_RequestData_min(data)

            notify("0000", json.dumps(body, ensure_ascii=False).encode("utf-8","ignore"))

        except Exception:
            print("[ERROR][Receive-0000] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
