# 파일: Tabs/manage_info_tab.py
from typing import Optional
import json

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox, QHeaderView, QSizePolicy, QWidget
)

from .csc_tab_base import CSCTabBase, _now_ms_since_2000


class ManageInfo(CSCTabBase):
    TITLE = "정보관리 CSC (INF)"

    # Message definitions (0000 ~ 0904)
    PUSH_MESSAGES = [
        ("0000", "응답(Response)"),
        ("0001", "공지"),
        ("0101", "시스템 운용 모드"),
        ("0102", "모듈 상태 정보"),
        ("0103", "SW 상태정보"),
        ("0104", "공격통제모듈 상태정보"),
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
        ("0504", "연료량 경고"),
        ("0601", "기저행위"),
        ("0602", "무인기 통제 명령"),
        ("0701", "의사결정 옵션정보"),
        ("0702", "의사결정 결과"),
        ("0801", "운용자 임무재계획 명령"),
        ("0802", "강제명령"),
        ("0803", "다음 협업기저임무 수행 명령"),
        ("0804", "선행임무 취소"),
        ("0805", "운용 이벤트"),
        ("0806", "시스템 부팅 명령"),
        ("0901", "옵션 정보 생성 요청"),
        ("0902", "재계획 요청"),
        ("0903", "수행임무갱신요청"),
        ("0904", "행동트리 서비스 제공 요청"),
    ]

    RECEIVE_MESSAGES = [
        ("0000", "응답(Response)"),
        ("0001", "공지"),
        ("0101", "시스템 운용 모드"),
        ("0102", "모듈 상태 정보"),
        ("0103", "SW 상태정보"),
        ("0104", "공격통제모듈 상태정보"),
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
        ("0504", "연료량 경고"),
        ("0601", "기저행위"),
        ("0602", "무인기 통제 명령"),
        ("0701", "의사결정 옵션정보"),
        ("0702", "의사결정 결과"),
        ("0801", "운용자 임무재계획 명령"),
        ("0802", "강제명령"),
        ("0803", "다음 협업기저임무 수행 명령"),
        ("0804", "선행임무 취소"),
        ("0805", "운용 이벤트"),
        ("0806", "시스템 부팅 명령"),
        ("0901", "옵션 정보 생성 요청"),
        ("0902", "재계획 요청"),
        ("0903", "수행임무갱신요청"),
        ("0904", "행동트리 서비스 제공 요청"),
    ]

    def mark_received(self, msg_id: str, raw: bytes | None = None):
        super().mark_received(msg_id, raw)
        if str(msg_id).strip().zfill(4) != "0504" or not raw:
            return

        payload = ""
        try:
            payload = raw.decode(errors="ignore")
        except Exception:
            if isinstance(raw, str):
                payload = raw

        body = {}
        if payload:
            try:
                body = json.loads(payload)
            except Exception:
                if '{' in payload and '}' in payload:
                    try:
                        body = json.loads(payload[payload.index('{'):payload.rindex('}') + 1])
                    except Exception:
                        body = {}

        try:
            level = int(body.get("fuelLevel", body.get("fuellevel", body.get("aircraftID", 0))))
        except Exception:
            level = 0
        level_name = {1: "YELLOW", 2: "RED"}.get(level, "UNKNOWN")
        aid = body.get("aircraftID", "-")
        try:
            self.append_log(f"[0504] Fuel warning: {level_name} (level={level}) - UAV {aid}")
        except Exception:
            pass

    def __init__(self, *, messenger, parent: Optional[QWidget] = None):
        super().__init__(messenger=messenger, parent=parent)
        self.tbl_tx.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태", "설정", "데이터"])
        self.tbl_rx.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태", "데이터"])
        self._mode_combo: Optional[QComboBox] = None
        self._current_system_mode: int = 0
        self._system_mode_row: int = self._find_tx_row("0101")
        self._install_system_mode_selector()

    def _find_tx_row(self, msg_id: str) -> int:
        for row in range(self.tbl_tx.rowCount()):
            item = self.tbl_tx.item(row, 0)
            if item and item.text().strip() == msg_id:
                return row
        return -1

    # ─────────────────────────────────────────────────────────────
    # 0101: 시스템 운용 모드 → 드롭다운(화살표) 선택으로 변경
    # ─────────────────────────────────────────────────────────────
    def _install_system_mode_selector(self) -> None:
        row = self._system_mode_row
        if row < 0:
            return

        # 0101 행은 버튼 대신 모드 콤보가 들어가므로 action 열을 별도로 넓힌다.
        header = self.tbl_tx.horizontalHeader()
        header.setSectionResizeMode(3, QHeaderView.Fixed)
        self.tbl_tx.setColumnWidth(3, 124)
        self.tbl_tx.setColumnWidth(4, 78)

        # 행 높이만 살짝 확보(테이블이 작아도 콤보는 안정적으로 보임)
        self.tbl_tx.setRowHeight(row, max(40, self.tbl_tx.rowHeight(row)))

        combo = QComboBox(self.tbl_tx)
        combo.setFocusPolicy(Qt.StrongFocus)
        combo.setEditable(False)
        combo.setMaxVisibleItems(6)
        combo.setSizeAdjustPolicy(QComboBox.AdjustToContentsOnFirstShow)
        combo.setMinimumWidth(116)
        combo.setMaximumHeight(34)
        combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        # 한국어 라벨 ↔ 모드값 매핑
        # 0: 초기화, 1: 대기, 2: 계획, 3: 임무
        combo.addItem("초기화(0)", 0)
        combo.addItem("대기(1)",   1)
        combo.addItem("초기임무계획(2)",   2)
        combo.addItem("임무수행(3)",   3)

        # 현재 모드 반영
        def _apply_current():
            # _current_system_mode가 목록에 있으면 해당 인덱스로
            for i in range(combo.count()):
                if int(combo.itemData(i)) == int(self._current_system_mode):
                    combo.setCurrentIndex(i)
                    return
            combo.setCurrentIndex(0)

        _apply_current()

        # 선택 즉시 전송
        combo.activated.connect(lambda _idx: self._on_mode_selected(row))

        self.tbl_tx.setCellWidget(row, 3, combo)
        self._mode_combo = combo

    def _on_mode_selected(self, row: int) -> None:
        if self._mode_combo is None:
            return
        mode = int(self._mode_combo.currentData())
        self._send_system_mode(row, mode, interactive=True)

    def _sync_mode_combo(self, mode: int) -> None:
        if self._mode_combo is None:
            return
        for i in range(self._mode_combo.count()):
            try:
                if int(self._mode_combo.itemData(i)) != int(mode):
                    continue
            except Exception:
                continue
            if self._mode_combo.currentIndex() != i:
                self._mode_combo.blockSignals(True)
                self._mode_combo.setCurrentIndex(i)
                self._mode_combo.blockSignals(False)
            break

    def _send_system_mode(self, row: int, mode: int, *, interactive: bool = False) -> bool:
        previous_mode = int(self._current_system_mode)
        self._current_system_mode = int(mode)
        self._sync_mode_combo(mode)

        body = {
            "timestamp": _now_ms_since_2000(),
            "source": "IDM",
            "systemMode": int(mode),
        }

        if interactive:
            body = self._confirm_tx_payload("0101", row, body, self.periodic_config.get("0101"))
            if body is None:
                self._current_system_mode = previous_mode
                self._sync_mode_combo(previous_mode)
                state_item = self.tbl_tx.item(row, 2)
                if state_item:
                    state_item.setText("전송 취소")
                return False
            try:
                confirmed_mode = int(body.get("systemMode", mode))
            except Exception:
                confirmed_mode = int(mode)
            self._current_system_mode = confirmed_mode
            self._sync_mode_combo(confirmed_mode)

        state_item = self.tbl_tx.item(row, 2)
        if state_item:
            state_item.setText("발신 중")

        return self._push_tx_once(row, "0101", body)
    def _build_overridden_body(self, msg_id: str):
        if str(msg_id).strip() == "0101":
            mode = self._current_system_mode
            if self._mode_combo is not None:
                data = self._mode_combo.currentData()
                if data is not None:
                    mode = int(data)
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
            # 현재 드롭다운 선택값으로 즉시 전송
            if self._mode_combo is not None:
                mode = int(self._mode_combo.currentData())
            else:
                mode = int(self._current_system_mode)
            self._send_system_mode(row, mode, interactive=True)
            return
        super()._on_tx_double_clicked(row, col)
