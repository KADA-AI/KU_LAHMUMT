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
    """
    message{msg_id}_push 모듈을 여러 네임스페이스에서 순차 탐색 후 전송 실행.
    공용/DS 어디에 있든 동작하도록 함.
    """
    last_exc = None
    mod = None
    for pref in SEARCH_PREFIXES:
        try:
            mod = importlib.import_module(f"{pref}.message{msg_id}_push")
            break
        except Exception as e:
            last_exc = e

    if mod is None:
        raise last_exc or ImportError(f"message{msg_id}_push not found in {SEARCH_PREFIXES}")

    # 전송 엔트리포인트 우선순위: make_and_push(body) > make_random_and_push() > push()
    if body_dict is not None and hasattr(mod, "make_and_push"):
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
