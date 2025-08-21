# receive_center.py

from typing        import Dict, List, Optional
from PyQt5.QtCore  import QTimer
from functools     import partial

# ─────────────────────────────────────────────────────────
# msg_id 별 리스너(탭 인스턴스)들을 보관하는 레지스트리
_listener_registry: Dict[str, List] = {}

def register_listener(msg_id: str, tab):
    _listener_registry.setdefault(msg_id, []).append(tab)

def unregister_listener(msg_id: str, tab_instance) -> None:
    """
    (필요 시) msg_id 에서 해당 탭 인스턴스 제거
    """
    if msg_id in _listener_registry:
        try:
            _listener_registry[msg_id].remove(tab_instance)
            if not _listener_registry[msg_id]:
                del _listener_registry[msg_id]
        except ValueError:
            pass

def notify(msg_id: str, raw: Optional[bytes] = None):
    """
    다른 스레드(C# 콜백) → GUI 스레드로 안전하게 큐잉
    """
    for tab in _listener_registry.get(msg_id, []):
        # PyQt5에서는 receiver 인자 없이 lambda/partial 로 캡처해 주면 됨
        QTimer.singleShot(
            0,
            partial(tab.mark_received, msg_id, raw)   # ← callable 만 전달
        )