# 파일: modules/common/push/message0305_push.py
# -*- coding: utf-8 -*-
# 0305 재계획 수행상태 정보 PUSH (안정화: 타입 탐색 범위/생성 안정성 보강)

import json, importlib, traceback
from datetime import datetime, timezone
from System import Activator

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)
MSG_ID = "0305"

def _try_set(obj, name: str, value) -> bool:
    """lowerCamel / PascalCase 모두 시도하여 .NET 프로퍼티를 세팅"""
    for k in (name, name[:1].upper()+name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False

def _cs(name: str):
    """
    타입 탐색 범위 확대:
      - nFusion.Model.msg_0305
      - nFusion.Model.CommonType
      - nFusion.Model
    """
    for modname in ('nFusion.Model.msg_0305', 'nFusion.Model.CommonType', 'nFusion.Model'):
        try:
            mod = importlib.import_module(modname)
            t = getattr(mod, name, None)
            if t is not None:
                return t
        except Exception:
            pass
    return None

def _new_msg0305():
    """
    안전한 인스턴스 생성:
      1) 알려진 후보명을 순차 시도 (실패 시 계속)
      2) 모듈 내 public 타입들을 돌아가며 기본생성자로 Activator.CreateInstance 시도
         → 생성 성공 & 'status' 계열 프로퍼티 보유 시 채택
    """
    # ① 알려진 후보명(라이브러리별 네이밍 편차 고려)
    CANDIDATE = (
        "ReplanStatusInfo", "ReplanningStatusInfo",
        "MissionPlanningStatusInfo", "MissionPlanningStateInfo",
        "MissionPlanningStatus", "MissionPlanReplanStatus"
    )
    for n in CANDIDATE:
        t = _cs(n)
        if t is None:
            continue
        try:
            obj = Activator.CreateInstance(t)  # t() 대신 Activator로 생성 안정화
            # 후보 프로퍼티 존재 확인(둘 중 하나만 있어도 OK)
            if any(hasattr(obj, k) for k in ("missionPlanningStatus", "MissionPlanningStatus", "status", "Status")):
                return obj
        except Exception:
            continue

    # ② 전체 public 타입 스캔: 생성 성공 + status 계열 프로퍼티 보유 시 채택
    tried = []
    for modname in ('nFusion.Model.msg_0305', 'nFusion.Model.CommonType', 'nFusion.Model'):
        try:
            mod = importlib.import_module(modname)
        except Exception:
            continue
        for name, t in list(mod.__dict__.items()):
            if not isinstance(t, type):
                continue
            tried.append(f"{modname}.{name}")
            try:
                obj = Activator.CreateInstance(t)
                if any(hasattr(obj, k) for k in ("missionPlanningStatus", "MissionPlanningStatus", "status", "Status")):
                    return obj
            except Exception:
                continue

    raise NameError("msg_0305 type not found (tried: " + ", ".join(tried[:20]) + ("..." if len(tried) > 20 else "") + ")")

def _dict_to_obj(body: dict):
    obj = _new_msg0305()
    # timestamp
    _try_set(obj, "timestamp", int(body.get("timestamp", _now_ms())))
    # source 호환
    src = body.get("source") or body.get("Source") or body.get("requestModuleName") or "MMR"
    if not _try_set(obj, "source", str(src)):
        _try_set(obj, "Source", str(src))
    # status 계열(최우선: missionPlanningStatus)
    st = int(body.get("missionPlanningStatus", body.get("status", 0)))
    if not (_try_set(obj, "missionPlanningStatus", st) or _try_set(obj, "status", st)):
        # 그래도 못 넣으면 마지막으로 StatusCode 등도 시도
        _try_set(obj, "StatusCode", st)
    # reason(옵션)
    if "replanReason" in body:
        _try_set(obj, "replanReason", str(body["replanReason"]))
    return obj

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    try:
        msg = _dict_to_obj(body_dict or {})
        node_messenger.Push(msg)
        log = f"[0305] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n[0305] PUSH 완료"
        return log.encode("utf-8","ignore")
    except Exception as e:
        # 예외를 로깅 문자열에 포함시켜 상위 로거로도 보이게
        err = f"[0305][ERR] {e}\n{traceback.format_exc()}"
        return (err + "\n").encode("utf-8","ignore")

def make_random_and_push(node_messenger) -> bytes:
    body = {"timestamp": _now_ms(), "source": "MMR", "missionPlanningStatus": 2, "replanReason":"초기임무재계획"}
    return make_and_push(body, node_messenger)
