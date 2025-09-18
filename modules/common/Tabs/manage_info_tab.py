# manage_info_tab.py
from typing import Optional

from PyQt5.QtWidgets import QButtonGroup, QHBoxLayout, QPushButton, QWidget

from Tabs.csc_tab_base import CSCTabBase, _now_ms_since_2000
from push_center import push_message


class ManageInfo(CSCTabBase):
    TITLE = "Info Management CSC"

    # Message definitions (0000 ~ 0904)
    PUSH_MESSAGES = [
        ("0000", "응답(Response)"),
        ("0101", "시스템 운용 모드"),
        ("0102", "모듈 상태 정보"),
        ("0103", "SW 상태정보"),
        ("0201", "협업기저임무 계획"),
        ("0202", "선행임무정보"),
        ("0203", "비행참조정보"),
        ("0301", "임무 계획"),
        ("0302", "개별 임무 계획"),
        ("0303", "무인기 비행 계획"),
        ("0304", "LAH 비행 계획"),
        ("0305", "재계획 수행 상태 정보"),
        ("0401", "유무인기 상태정보"),
        ("0402", "전장상황인지정보"),
        ("0501", "임무수행상태정보"),
        ("0502", "임무종료 요청"),
        ("0503", "협업기저임무 완료 알림"),
        ("0601", "기저행위"),
        ("0602", "무인기 통제 명령"),
        ("0701", "의사결정 옵션정보"),
        ("0702", "의사결정 결과"),
        ("0801", "운용자 임무재계획 명령"),
        ("0802", "강제명령"),
        ("0803", "다음 협업기저임무 수행 명령"),
        ("0805", "운용 이벤트"),
        ("0806", "시스템 부팅 명령"),
        ("0901", "옵션 정보 생성 요청"),
        ("0902", "재계획 요청"),
        ("0903", "수행임무갱신요청"),
        ("0904", "행동트리 서비스 제공 요청"),
    ]

    RECEIVE_MESSAGES = [
        ("0000", "응답(Response)"),
        ("0101", "시스템 운용 모드"),
        ("0102", "모듈 상태 정보"),
        ("0103", "SW 상태정보"),
        ("0201", "협업기저임무 계획"),
        ("0202", "선행임무정보"),
        ("0203", "비행참조정보"),
        ("0301", "임무 계획"),
        ("0302", "개별 임무 계획"),
        ("0303", "무인기 비행 계획"),
        ("0304", "LAH 비행 계획"),
        ("0305", "재계획 수행 상태 정보"),
        ("0401", "유무인기 상태정보"),
        ("0402", "전장상황인지정보"),
        ("0501", "임무수행상태정보"),
        ("0502", "임무종료 요청"),
        ("0503", "협업기저임무 완료 알림"),
        ("0601", "기저행위"),
        ("0602", "무인기 통제 명령"),
        ("0701", "의사결정 옵션정보"),
        ("0702", "의사결정 결과"),
        ("0801", "운용자 임무재계획 명령"),
        ("0802", "강제명령"),
        ("0803", "다음 협업기저임무 수행 명령"),
        ("0805", "운용 이벤트"),
        ("0806", "시스템 부팅 명령"),
        ("0901", "옵션 정보 생성 요청"),
        ("0902", "재계획 요청"),
        ("0903", "수행임무갱신요청"),
        ("0904", "행동트리 서비스 제공 요청"),
    ]

    def __init__(self, *, messenger, parent: Optional[QWidget] = None):
        super().__init__(messenger=messenger, parent=parent)
        self._mode_group: Optional[QButtonGroup] = None
        self._current_system_mode: int = 0
        self._system_mode_row: int = self._find_tx_row("0101")
        self._install_system_mode_buttons()

    def _find_tx_row(self, msg_id: str) -> int:
        for row in range(self.tbl_tx.rowCount()):
            item = self.tbl_tx.item(row, 0)
            if item and item.text().strip() == msg_id:
                return row
        return -1

    def _install_system_mode_buttons(self) -> None:
        row = self._system_mode_row
        if row < 0:
            return

        self.tbl_tx.setColumnWidth(3, 240)

        container = QWidget(self.tbl_tx)
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self._mode_group = QButtonGroup(container)
        self._mode_group.setExclusive(True)

        buttons = [
            ("Init", 0),
            ("Standby", 1),
            ("Planning", 2),
            ("Mission", 3),
        ]

        for label, value in buttons:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setToolTip(f"SystemMode={value}")
            btn.clicked.connect(lambda _=False, v=value: self._send_system_mode(row, v))
            layout.addWidget(btn)
            self._mode_group.addButton(btn, value)

        layout.addStretch(1)
        self.tbl_tx.setCellWidget(row, 3, container)

        default_btn = self._mode_group.button(self._current_system_mode)
        if default_btn:
            default_btn.setChecked(True)

    def _send_system_mode(self, row: int, mode: int) -> None:
        self._current_system_mode = int(mode)
        if self._mode_group:
            btn = self._mode_group.button(mode)
            if btn and not btn.isChecked():
                btn.setChecked(True)

        body = {
            "timestamp": _now_ms_since_2000(),
            "source": "IDM",
            "systemMode": int(mode),
        }

        state_item = self.tbl_tx.item(row, 2)
        if state_item:
            state_item.setText("Sending")

        ok = push_message(
            "0101",
            self.messenger,
            on_done=lambda mid, raw: self._mark_single_sent(row, mid, raw),
            body_dict=body,
        )
        if not ok and state_item:
            state_item.setText("Send Failed")

    def _build_overridden_body(self, msg_id: str):
        if str(msg_id).strip() == "0101":
            mode = self._current_system_mode
            if self._mode_group:
                checked = self._mode_group.checkedId()
                if checked >= 0:
                    mode = checked
            return {
                "timestamp": _now_ms_since_2000(),
                "source": "IDM",
                "systemMode": int(mode),
            }
        return super()._build_overridden_body(msg_id)

    def _on_tx_double_clicked(self, row: int, col: int):
        item = self.tbl_tx.item(row, 0)
        msg_id = item.text().strip() if item else ""
        if msg_id == "0101":
            mode = self._current_system_mode
            if self._mode_group:
                checked = self._mode_group.checkedId()
                if checked >= 0:
                    mode = checked
            self._send_system_mode(row, mode)
            return
        super()._on_tx_double_clicked(row, col)
