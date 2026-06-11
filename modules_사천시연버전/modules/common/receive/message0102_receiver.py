# modules/common/receive/message0102_receiver.py
# auto-fixed at 2025-09-19

from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0102 import *              # C# 모델
from nFusion.Model.CommonType import *            # 공통 타입
from .database import received_db
from receive_center import notify
import json, traceback, sys, os

# 대/소문자 안전 접근
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

def _coerce_int(v):
    try:
        return int(v)
    except Exception:
        try:
            return int(getattr(v, "value"))
        except Exception:
            try:
                return int(str(v))
            except Exception:
                return None

def _to_dict_ModuleStatus(obj):
    """
    0102 수신 → 표준 바디(dict):
      { "timestamp": int(ms_2000), "status": int(0|1|2), "source": "MMR|MSM|MOB|..." }
    - 다양한 대/소문자·별칭을 모두 흡수하여 위 3키로 통일
    - 값 생성/가공 없음(있을 때만 추출)
    """
    d = {}

    # timestamp
    ts = _get(obj, 'timestamp','Timestamp','timeStamp','TimeStamp','ts','TS')
    ts_i = _coerce_int(ts)
    if ts_i is not None:
        d['timestamp'] = ts_i

    # source (SourceModuleName은 쓰지 않음)
    src = _get(
        obj,
        'source','Source',               # 표준
        'requestModuleName','RequestModuleName',  # 옛 별칭
        'module','Module','sourceModule','SourceModule'
    )
    if src is not None and str(src) != '':
        d['source'] = str(src)

    # status
    st = _get(
        obj,
        'status','Status',
        'moduleStatus','ModuleStatus',
        'healthStatus','HealthStatus',
        'state','State'
    )
    st_i = _coerce_int(st)
    if st_i is not None:
        d['status'] = st_i

    return d

class ModuleStatusReceiver_0102(IFusionReceive[ModuleStatus], IsLocal, IsSingletone):
    """0102 ModuleStatus 메시지 수신 리시버"""
    __namespace__ = "ModuleStatusReceiver_0102"

    def Receive(self, data: ModuleStatus, src):
        try:
            # 선택: 최근 수신 원본을 저장 (있으면)
            try:
                received_db.set_received_0102(data)
            except Exception:
                pass

            body = _to_dict_ModuleStatus(data)
            # 항상 표준 키들만 notify
            notify("0102", json.dumps(body, ensure_ascii=False).encode("utf-8","ignore"))

        except Exception:
            print("[ERROR][Receive-0102] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
