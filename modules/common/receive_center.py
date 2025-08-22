from typing import Dict, List, Optional
from functools import partial
from PyQt5.QtCore import QTimer

# msg_id 별 리스너(탭 인스턴스) 보관
_listener_registry: Dict[str, List] = {}

def _norm(mid) -> str:
    s = str(mid)
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s

# ── 교체: register_listener ──
def register_listener(msg_id: str, tab) -> None:
    key = _norm(msg_id)
    _listener_registry.setdefault(key, []).append(tab)

# ── 교체: unregister_listener ──
def unregister_listener(msg_id: str, tab_instance) -> None:
    key = _norm(msg_id)
    lst = _listener_registry.get(key)
    if not lst:
        return
    try:
        lst.remove(tab_instance)
        if not lst:
            del _listener_registry[key]
    except ValueError:
        pass

# ── 교체: notify ──
def notify(msg_id: str, raw: Optional[bytes] = None) -> None:
    """
    다른 스레드(C# 콜백) → GUI 스레드로 안전 큐잉.
    각 탭은 mark_received(msg_id, raw) 메서드를 구현해야 함.
    """
    key = _norm(msg_id)  # ← 여기서 4자리로 통일 (예: '502' → '0502')
    for tab in _listener_registry.get(key, []):
        QTimer.singleShot(0, partial(tab.mark_received, key, raw))