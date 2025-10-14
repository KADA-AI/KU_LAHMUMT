# data/logic_storage.py: 내부 로직에 의해 처리/생성된 결과 데이터를 인메모리에 저장하고 관리하는 클래스를 정의합니다.

# data/logic_storage.py
from typing import Any, Dict

class LogicStorage:
    """내부 로직에 의해 처리/생성된 결과 데이터를 저장합니다."""
    def __init__(self):
        self._data: Dict[str, Any] = {}
        self._data["fuel_data"] = None # Initialize fuel_data
        self._data["fuel_warning_prev"] = {}

    def set_data(self, key: str, value: Any):
        self._data[key] = value

    def get_data(self, key: str) -> Any:
        return self._data.get(key)

    def get_all_data(self) -> Dict[str, Any]:
        return self._data.copy()