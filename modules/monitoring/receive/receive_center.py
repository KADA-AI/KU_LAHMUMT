from typing import Dict, List, Callable
from functools import partial
from PyQt5.QtCore import QTimer, QObject, pyqtSignal

# msg_id 별 리스너(핸들러 함수) 보관
_listener_registry: Dict[str, List[Callable]] = {}

# --- 새로운 시그널 이미터 클래스 ---
class SignalEmitter(QObject):
    # msg_id와 data_object를 전달하는 시그널
    message_received = pyqtSignal(str, object)

# 전역 시그널 이미터 인스턴스 (GUI 스레드에서 초기화되어야 함)
global_signal_emitter: SignalEmitter = None

def set_global_signal_emitter(emitter: SignalEmitter):
    global global_signal_emitter
    global_signal_emitter = emitter

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

def notify_to_manager(msg_id: str, data_object: object) -> None:
    """다른 스레드(C# 콜백)에서 호출되어, 등록된 리스너에게 데이터 객체를 전달합니다.
    GUI 스레드에서 안전하게 실행되도록 QTimer.singleShot을 사용합니다."""
    key = _norm(msg_id)
    if global_signal_emitter:
        global_signal_emitter.message_received.emit(msg_id, data_object)
    else:
        print(f"[ERROR][receive_center] global_signal_emitter not set! Cannot emit signal for msg_id: {msg_id}")
