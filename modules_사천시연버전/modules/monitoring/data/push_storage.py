from typing import Dict, List, Union

from .message_models import (
    CollaborativeMissionCompleteModel,
    MissionEndRequestBodyModel,
    MissionProgressBodyModel,
    ModuleStatusModelModel,
    ReplanRequestBodyModel,
)

PushBody = Union[
    ModuleStatusModelModel,
    MissionProgressBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
    CollaborativeMissionCompleteModel,
]


class PushStorage:
    """Keeps a bounded history of outbound push payloads."""

    def __init__(self, max_history_per_key: int = 50):
        self._data: Dict[str, List[PushBody]] = {}
        self.max_history = max_history_per_key

    def add_data(self, key: str, value: PushBody) -> None:
        history = self._data.setdefault(key, [])
        history.insert(0, value)
        if len(history) > self.max_history:
            history.pop()

    def get_history(self, key: str) -> List[PushBody]:
        return self._data.get(key, [])

    def get_all_data(self) -> Dict[str, List[PushBody]]:
        return self._data.copy()
