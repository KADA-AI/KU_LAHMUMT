# modules/monitoring_ver2/push/message0902_push.py
from typing import Any, Dict

# 0501 push 모듈과 동일한 구조로, common 모듈의 push 기능에 작업을 위임합니다.
try:
    from modules.common.push import message0902_push as _common_push
except (ModuleNotFoundError, ImportError):
    import importlib
    _common_push = importlib.import_module('modules.common.push.message0902_push')

def _prepare_body(body: Any) -> Dict[str, Any]:
    """
    replan_actual_logic에서 생성된 간단한 dict를
    common push 헬퍼가 이해할 수 있도록 그대로 반환합니다.
    """
    if isinstance(body, dict):
        return body
    
    # dataclass 등 다른 타입에 대한 처리 로직이 필요하다면 여기에 추가할 수 있습니다.
    # 예: if is_dataclass(body): return asdict(body)
    
    raise TypeError(f"body must be a dict, not {type(body).__name__}")


def make_and_push(body: Any, node_messenger) -> bytes:
    """ReplanRequest(0902) 메시지 전송을 공용 push 구현에 위임합니다."""
    # _prepare_body를 통과한 딕셔너리를 common push 모듈로 전달합니다.
    return _common_push.make_and_push(_prepare_body(body), node_messenger)


def make_random_and_push(node_messenger) -> bytes:
    """필요 시, 공용 랜덤 메시지 생성기에 작업을 위임합니다."""
    return _common_push.make_random_and_push(node_messenger)