# modules/common/receive/message0501_receiver.py
# MissionProgress(0501) 수신 리시버 - 안정 변환 & notify

from dll_files.nFusionImports import *          # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0501 import *            # C# 모델
from nFusion.Model.CommonType import *          # 공통 타입
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────────────────────── 유틸 ──────────────────────────
def _get(obj, *names):
    """대/소문자 혼용 속성 안전 접근"""
    for n in names:
        try:
            if hasattr(obj, n):
                return getattr(obj, n)
        except Exception:
            pass
    return None

def _as_int(v):
    try:
        # UInt32/UInt64/문자열 모두 수용
        return int(str(v))
    except Exception:
        return None

def _iter_safe(coll):
    """IEnumerable(.NET) / list 모두 안전 순회"""
    if coll is None:
        return []
    try:
        for it in coll:
            yield it
    except Exception:
        return []

# ─────────────────────── C# → dict 변환 ───────────────────────
def _to_dict_CurrentIndividualMission(obj):
    d = {}
    v = _get(obj, 'individualMissionID', 'IndividualMissionID')
    iv = _as_int(v)
    if iv is not None:
        d['individualMissionID'] = iv
    return d

def _to_dict_IndividualMissionProgressStatus(obj):
    d = {}
    v = _get(obj, 'aircraftID', 'AircraftID')
    iv = _as_int(v)
    if iv is not None:
        d['aircraftID'] = iv

    sub = _get(obj, 'currentIndividualMission', 'CurrentIndividualMission')
    if sub is not None:
        d['currentIndividualMission'] = _to_dict_CurrentIndividualMission(sub)

    v = _get(obj, 'currentIndividualMissionProgress', 'CurrentIndividualMissionProgress')
    iv = _as_int(v)
    if iv is not None:
        d['currentIndividualMissionProgress'] = iv
    return d

def _to_dict_MissionProgress(obj, src_hint=None):
    d = {}

    v = _get(obj, 'timestamp', 'Timestamp')
    iv = _as_int(v)
    if iv is not None:
        d['timestamp'] = iv

    # source는 여러 후보에서 폴백 (모델 속성 → 요청자 이름/문자열 → RequestModuleName)
    s = _get(obj, 'source', 'Source', 'requestModuleName', 'RequestModuleName')
    if s is None or str(s).strip() == '':
        if src_hint is not None:
            try:
                s = getattr(src_hint, 'Name', None) or str(src_hint)
            except Exception:
                s = None
    if s is not None and str(s).strip() != '':
        d['source'] = str(s)

    v = _get(obj, 'currentMissionPlanID', 'CurrentMissionPlanID')
    iv = _as_int(v)
    if iv is not None:
        d['currentMissionPlanID'] = iv

    v = _get(obj, 'currentInputMissionID', 'CurrentInputMissionID')
    iv = _as_int(v)
    if iv is not None:
        d['currentInputMissionID'] = iv

    coll = _get(obj, 'individualMissionProgressStatusList', 'IndividualMissionProgressStatusList')
    items = []
    for it in _iter_safe(coll):
        try:
            items.append(_to_dict_IndividualMissionProgressStatus(it))
        except Exception:
            # 개별 아이템 오류는 스킵
            pass
    if items:
        d['individualMissionProgressStatusList'] = items

    return d

# ───────────────────────── Receiver ─────────────────────────
class MissionProgressReceiver_0501(IFusionReceive[MissionProgress], IsLocal, IsSingletone):
    """0501 MissionProgress 메시지 수신 리시버"""

    __namespace__ = "MissionProgressReceiver_0501"

    def Receive(self, data: MissionProgress, src):
        try:
            # 원본 보관 (있으면)
            try:
                received_db.set_received_0501(data)
            except Exception:
                pass

            body = _to_dict_MissionProgress(data, src_hint=src)

            # notify로 바이너리 전송(JSON UTF-8)
            payload = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
            notify("0501", payload)

        except Exception:
            print("[ERROR][Receive-0501] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
