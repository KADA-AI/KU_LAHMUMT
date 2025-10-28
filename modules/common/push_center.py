import importlib, socket, json, os

_DASH_PORT = int(os.getenv("KU_DASHBOARD_PORT", "45991"))

# 필요시 동적으로 늘릴 수 있게 리스트로 관리
SEARCH_PREFIXES = [
    "generator",                          # ✅ 새로 추가: 최우선으로 generator/ 를 탐색
    "push_info",                          # 공용 push_info
    "push",                               # 공용 push
    "modules.common.push_info",           # 명시 common
    "modules.common.push",
]

def push_message(msg_id: str, messenger, *, on_done=None, body_dict=None) -> bool:
    import importlib
    import traceback

    last_exc = None
    mod = None
    # 유효한 경로만 남깁니다.
    search_prefixes = [
        "generator",
        "push_info",
        "push",
        "modules.common.push_info",
        "modules.common.push",
    ]

    for pref in search_prefixes:
        module_name = f"{pref}.message{msg_id}_push"
        try:
            mod = importlib.import_module(module_name)
            # 성공 시 디버깅 로그 추가
            # print(f"[push_center] Successfully imported: {module_name}")
            break
        except Exception as e:
            # 실패 시, 어떤 예외가 발생했는지 정확히 출력합니다.
            print(f"[DEBUG] Failed to import '{module_name}'. Error: {e}")
            # traceback.print_exc() # 더 상세한 스택 트레이스가 필요하면 주석 해제
            last_exc = e

    if mod is None:
        raise last_exc or ImportError(f"message{msg_id}_push not found in {search_prefixes}")

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