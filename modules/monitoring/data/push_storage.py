# data/push_storage.py: 외부로 전송(Push)한 메시지의 이력을 저장하고 관리하는 클래스를 정의합니다.

# data/push_storage.py
from typing import Dict, List, Union

# --- 데이터 모델 import ---
from .message_models import (
    ModuleStatusModelModel,
    MissionPerformanceStatusBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
    CollaborativeMissionCompleteModel,
)

# --- 저장 가능한 모든 Push 메시지 본문 타입을 정의 ---
PushBody = Union[
    ModuleStatusModelModel,
    MissionPerformanceStatusBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
    CollaborativeMissionCompleteModel,
]


class PushStorage:
    """외부로 전송(Push)한 메시지의 이력을 데이터 클래스 객체로 저장합니다."""

    def __init__(self, max_history_per_key: int = 50):
        # 이제 저장소는 메시지 ID를 키로, PushBody 객체들의 리스트를 값으로 가집니다.
        self._data: Dict[str, List[PushBody]] = {}
        self.max_history = max_history_per_key

    def add_data(self, key: str, value: PushBody):
        """새로운 전송 기록(데이터 클래스 인스턴스)을 추가합니다."""
        if key not in self._data:
            self._data[key] = []

        self._data[key].insert(0, value)  # 새 항목을 맨 앞에 추가

        # 최대 기록 수를 초과하면 가장 오래된 항목을 삭제합니다.
        if len(self._data[key]) > self.max_history:
            self._data[key].pop()

    def get_history(self, key: str) -> List[PushBody]:
        """특정 키의 전송 이력을 반환합니다."""
        return self._data.get(key, [])

    def get_all_data(self) -> Dict[str, List[PushBody]]:
        return self._data.copy()