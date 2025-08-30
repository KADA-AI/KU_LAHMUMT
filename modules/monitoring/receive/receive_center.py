from typing import Dict, List, Callable
from functools import partial
from PyQt5.QtCore import QTimer

# msg_id 별 리스너(핸들러 함수) 보관
_listener_registry: Dict[str, List[Callable]] = {}

def _norm(mid) -> str:
    s = str(mid)
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s

def register_listener(msg_id: str, handler: Callable) -> None:
    """특정 메시지 ID를 처리할 핸들러 함수를 등록합니다."""
    key = _norm(msg_id)
    _listener_registry.setdefault(key, []).append(handler)

def unregister_listener(msg_id: str, handler: Callable) -> None:
    """등록했던 핸들러 함수를 제거합니다."""
    key = _norm(msg_id)
    lst = _listener_registry.get(key)
    if not lst:
        return
    try:
        lst.remove(handler)
        if not lst:
            del _listener_registry[key]
    except ValueError:
        pass

def notify(msg_id: str, data_object: object) -> None:
    """다른 스레드(C# 콜백)에서 호출되어, 등록된 리스너에게 데이터 객체를 전달합니다.
    GUI 스레드에서 안전하게 실행되도록 QTimer.singleShot을 사용합니다."""
    key = _norm(msg_id)
    for handler in _listener_registry.get(key, []):
        # 등록된 핸들러(lambda obj: manager.handle_message_reception(key, obj))를 호출합니다.
        QTimer.singleShot(0, partial(handler, data_object))
