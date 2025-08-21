from typing import Dict, List, Optional
from functools import partial
from PyQt5.QtCore import QTimer

# msg_id 별 리스너(탭 인스턴스) 보관
_listener_registry: Dict[str, List] = {}

def register_listener(msg_id: str, tab) -> None:
    _listener_registry.setdefault(msg_id, []).append(tab)

def unregister_listener(msg_id: str, tab_instance) -> None:
    lst = _listener_registry.get(msg_id)
    if not lst:
        return
    try:
        lst.remove(tab_instance)
        if not lst:
            del _listener_registry[msg_id]
    except ValueError:
        pass

def notify(msg_id: str, raw: Optional[bytes] = None) -> None:
    """
    다른 스레드(C# 콜백) → GUI 스레드로 안전 큐잉.
    각 탭은 mark_received(msg_id, raw) 메서드를 구현해야 함.
    """
    for tab in _listener_registry.get(msg_id, []):
        QTimer.singleShot(0, partial(tab.mark_received, msg_id, raw))
