from typing import Any, Dict


class LogicStorage:
    """In-memory cache for logic outputs."""

    def __init__(self):
        self._data: Dict[str, Any] = {}
        self._data["fuel_data"] = None

    def set_data(self, key: str, value: Any) -> None:
        self._data[key] = value

    def get_data(self, key: str) -> Any:
        return self._data.get(key)

    def get_all_data(self) -> Dict[str, Any]:
        return self._data.copy()
