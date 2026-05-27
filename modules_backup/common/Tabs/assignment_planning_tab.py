from __future__ import annotations

from typing import Callable, Optional

from PyQt5.QtWidgets import QDialog

from modules.common import db_paths
from modules.common.message_payload_dialog import MessagePayloadDialog
from modules.common.option_codes import DEFAULT_OPTION_CODE_SEQUENCE

from .csc_tab_base import CSCTabBase, _now_ms_since_2000


class AssignmentPlanningTab(CSCTabBase):
    TITLE = "업무 할당 및 계획수립 CSC (MMR)"

    RECEIVE_MESSAGES = [
        ("0001", "공지"),
        ("0101", "시스템 운용 모드"),
        ("0201", "협업기저임무 계획"),
        ("0202", "선행임무정보"),
        ("0203", "비행참조정보"),
        ("0104", "공격통제모듈 상태정보"),
        ("0401", "유무인기 상태정보"),
        ("0402", "전장상황인지정보"),
        ("0501", "임무수행상태정보"),
        ("0806", "시스템 부팅 명령"),
        ("0902", "재계획 요청"),
    ]

    PUSH_MESSAGES = [
        ("0001", "공지"),
        ("0102", "모듈 상태 정보"),
        ("0301", "임무 계획"),
        ("0302", "개별 임무 계획"),
        ("0303", "무인기 비행 계획"),
        ("0304", "LAH 비행 계획"),
        ("0305", "재계획 수행 상태 정보"),
        ("0901", "옵션 정보 생성 요청"),
        ("0903", "수행임무갱신요청"),
    ]

    _DB_ID_RULES = {
        "0301": ("MissionPlan", "missionPlanID", None),
        "0302": ("IndividualMissionPlan", "individualMissionPackageID", None),
        "0303": ("FlightPath", "pathID", None),
        "0304": ("FlightPath", "pathID", "123"),
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._replan_callback: Optional[Callable[[str, Optional[bytes]], None]] = None
        self._push_message_names = {str(msg_id).zfill(4): name for msg_id, name in self.PUSH_MESSAGES}

    def set_replan_callback(self, callback: Optional[Callable[[str, Optional[bytes]], None]]) -> None:
        self._replan_callback = callback

    def mark_received(self, msg_id: str, raw: Optional[bytes] = None):
        super().mark_received(msg_id, raw)
        if str(msg_id).zfill(4) == "0902" and callable(self._replan_callback):
            try:
                self._replan_callback(msg_id, raw)
            except Exception:
                pass

    def default_tx_payload(self, msg_id: str) -> dict | None:
        return self._build_overridden_body(msg_id)

    def edit_tx_payload(
        self,
        msg_id: str,
        body: dict | None,
        *,
        periodic_rate_hz: float | None = None,
    ) -> dict | None:
        mid = str(msg_id).zfill(4)
        name = self._push_message_names.get(mid, mid)
        dialog = MessagePayloadDialog(
            mid,
            name,
            body or {},
            periodic_rate_hz=periodic_rate_hz,
            parent=self,
        )
        if dialog.exec_() != QDialog.Accepted:
            return None
        return dialog.payload

    def _should_confirm_tx(self, msg_id: str, interactive: bool) -> bool:
        return bool(interactive and str(msg_id).zfill(4) in self._push_message_names)

    def _confirm_tx_payload(
        self,
        msg_id: str,
        row: int,
        body: dict | None,
        freq_hz: float | None,
    ) -> dict | None:
        return self.edit_tx_payload(msg_id, body, periodic_rate_hz=freq_hz)

    def _build_overridden_body(self, msg_id: str):
        mid = str(msg_id).strip().zfill(4)
        timestamp = _now_ms_since_2000()
        source = self._sw_code()

        if mid == "0001":
            return {
                "timestamp": timestamp,
                "source": source,
                "contents": "",
            }

        if mid == "0102":
            return {
                "timestamp": timestamp,
                "source": source,
                "status": 1,
            }

        if mid in self._DB_ID_RULES:
            directory_name, field_name, prefix_chars = self._DB_ID_RULES[mid]
            return {
                "timestamp": timestamp,
                "source": source,
                field_name: self._latest_numeric_id(directory_name, prefix_chars=prefix_chars),
            }

        if mid == "0305":
            return {
                "timestamp": timestamp,
                "source": source,
                "missionPlanningStatus": 2,
                "replanReason": "초기임무재계획",
            }

        if mid == "0901":
            plan_id = self._latest_numeric_id("MissionPlan")
            option_name = (
                DEFAULT_OPTION_CODE_SEQUENCE[0]
                if DEFAULT_OPTION_CODE_SEQUENCE
                else "OPTION_A"
            )
            return {
                "timestamp": timestamp,
                "source": source,
                "requestTime": timestamp,
                "pendingOptionList": [
                    {
                        "optionID": 1,
                        "optionName": option_name,
                        "missionPlanID": plan_id,
                    }
                ],
            }

        if mid == "0903":
            return {
                "timestamp": timestamp,
                "source": source,
                "missionPlanID": self._latest_numeric_id("MissionPlan"),
            }

        return super()._build_overridden_body(mid)

    def _latest_numeric_id(self, directory_name: str, *, prefix_chars: str | None = None) -> int:
        try:
            directory = db_paths.get_db_subpath(directory_name)
        except Exception:
            return 0

        latest = 0
        try:
            for path in directory.glob("*.json"):
                stem = path.stem
                if not stem.isdigit():
                    continue
                if prefix_chars and stem[:1] not in prefix_chars:
                    continue
                latest = max(latest, int(stem))
        except Exception:
            return latest
        return latest
