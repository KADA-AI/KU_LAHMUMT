# [push_center.py / top-level 함수] ─ 동적 임포트 with 다중 prefix
import importlib

def push_message(msg_id: str, messenger, *, on_done=None, body_dict=None) -> bool:
    prefixes = [
        "push_info",                       # DS 폴더가 sys.path 최상단이면 여기로 끝
        "push",
        "modules.decision_support.push_info",
        "modules.decision_support.push",
    ]

    last_exc = None
    mod = None
    for pref in prefixes:
        try:
            mod = importlib.import_module(f"{pref}.message{msg_id}_push")
            break
        except Exception as e:
            last_exc = e

    if mod is None:
        # 어디에서도 못 찾으면 원인 노출
        raise last_exc or ImportError(f"message{msg_id}_push not found in {prefixes}")

    # 랜덤 생성/전송 or 명시 body_dict 전송 모두 지원
    if body_dict is not None and hasattr(mod, "make_and_push"):
        raw = mod.make_and_push(body_dict, messenger)
    elif hasattr(mod, "make_random_and_push"):
        raw = mod.make_random_and_push(messenger)
    elif hasattr(mod, "push"):
        raw = mod.push(messenger)
    else:
        raise AttributeError(f"{mod.__name__} has no push entrypoint")

    if on_done:
        on_done(msg_id, raw)
    return True
