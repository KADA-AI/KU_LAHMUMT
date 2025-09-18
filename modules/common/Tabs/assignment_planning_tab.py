# assignment_planning_tab.py
from typing import Callable, Optional
from Tabs.csc_tab_base import CSCTabBase


class AssignmentPlanningTab(CSCTabBase):
    TITLE = "업무 할당 및 계획수립 CSC"
    
    # **BD → Receive**
    RECEIVE_MESSAGES = [
        ("0101", "시스템 운용 모드"),
        ("0201", "협업기저임무 계획"),
        ("0202", "선행임무정보"),
        ("0203", "비행참조정보"),
        ("0401", "유무인기 상태정보"),
        ("0402", "전장상황인지정보"),
        ("0501", "임무수행상태정보"),
        ("0806", "시스템 부팅 명령"),
        ("0902", "재계획 요청"),
    ]
    
    # **DB → Push**
    PUSH_MESSAGES = [
        ("0102", "모듈 상태 정보"),              # manage_info_tab.py 기준 이름 반영
        ("0301", "임무 계획"),
        ("0302", "개별 임무 계획"),
        ("0303", "무인기 비행 계획"),
        ("0304", "LAH 비행 계획"),
        ("0305", "재계획 수행 상태 정보"),
        ("0901", "옵션 정보 생성 요청"),
        ("0903", "수행임무갱신요청"),
    ]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._replan_callback: Optional[Callable[[str, Optional[bytes]], None]] = None

    def set_replan_callback(self, callback: Optional[Callable[[str, Optional[bytes]], None]]) -> None:
        self._replan_callback = callback

    def mark_received(self, msg_id: str, raw: Optional[bytes] = None):
        super().mark_received(msg_id, raw)
        if str(msg_id).zfill(4) == "0902" and callable(self._replan_callback):
            try:
                self._replan_callback(msg_id, raw)
            except Exception:
                pass

