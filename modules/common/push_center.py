import importlib, socket, json, os

_DASH_PORT = int(os.getenv("KU_DASHBOARD_PORT", "45991"))

# 필요시 동적으로 늘릴 수 있게 리스트로 관리
SEARCH_PREFIXES = [
    "generator",                          # ✅ 새로 추가: 최우선으로 generator/ 를 탐색
    "push_info",                          # 공용 push_info
    "push",                               # 공용 push
    "modules.common.push_info",           # 명시 common
    "modules.common.push",
    "modules.decision_support.push_info", # DS fallback
    "modules.decision_support.push",
]

def push_message(msg_id: str, messenger, *, on_done=None, body_dict=None) -> bool:
    import importlib

    last_exc = None
    mod = None
    for pref in SEARCH_PREFIXES:
        try:
            mod = importlib.import_module(f"{pref}.message{msg_id}_push")
            # 디버깅에 도움: 실제 로딩된 모듈 경로를 한번 찍어두면 혼동이 없습니다.
            # print(f"[push_center] resolved: {mod.__name__} ({getattr(mod, '__file__', '?')})")
            break
        except Exception as e:
            last_exc = e

    if mod is None:
        raise last_exc or ImportError(f"message{msg_id}_push not found in {SEARCH_PREFIXES}")

    # 👇 변경 포인트: '비어있지 않은 dict'일 때만 make_and_push 사용
    use_manual = isinstance(body_dict, dict) and len(body_dict) > 0

    if use_manual and hasattr(mod, "make_and_push"):
        raw = mod.make_and_push(body_dict, messenger)
    elif hasattr(mod, "make_random_and_push"):
        raw = mod.make_random_and_push(messenger)
    elif hasattr(mod, "push"):
        raw = mod.push(messenger)
    else:
        raise AttributeError(f"{mod.__name__} has no push entrypoint (make_and_push/make_random_and_push/push)")

    if on_done:
        on_done(msg_id, raw)
    return True