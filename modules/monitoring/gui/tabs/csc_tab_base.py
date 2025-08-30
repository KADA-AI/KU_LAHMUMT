# csc_tab_base.py
from __future__ import annotations
from datetime import datetime
from typing import Optional, Sequence, Tuple, Dict

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QTextEdit, QLabel, QSizePolicy, QHeaderView, QPushButton, QDialog, QDialogButtonBox
)
import json
import re

from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt, QTimer
from push_center import push_message
from receive_center import register_listener

_EPOCH2000_MS = 946684800000
def _now_ms_since_2000():
    import time
    return int(time.time() * 1000) - _EPOCH2000_MS

class CSCTabBase(QWidget):
    """
    공용 CSC 탭 베이스
    ──────────────────────────────────────────────
    서브클래스에서 TITLE / PUSH_MESSAGES / RECEIVE_MESSAGES 만 바꿔주면
    UI·로직은 자동 재사용된다.
    """

    # 서브클래스에서 오버라이드할 상수 -----------------
    TITLE: str = "CSC"
    PUSH_MESSAGES: Sequence[Tuple[str, str]] = ()
    RECEIVE_MESSAGES: Sequence[Tuple[str, str]] = ()
    # -------------------------------------------------

    def __init__(self, *, messenger, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.messenger = messenger

        self.periodic_config: Dict[str, Optional[float]] = {
            '0000': None,  
            '0101': None,
            '0102': 5,
            '0103': 5,
            '0201': None,
            '0202': None,
            '0203': None,
            '0301': None,
            '0302': None,
            '0303': None,
            '0304': None,
            '0401': 5,
            '0402': None,
            '0501': 15,
            '0502': None,
            '0503': None,
            '0601': None,
            '0602': None,
            '0701': None,
            '0702': None,
            '0801': None,
            '0802': None,
            '0803': None,
            '0804': None,
            '0805': None,
            '0806': None,
            '0901': None,
            '0902': None,
            '0903': None,
            '0904': None,
        }

        # 2) 동작 중인 타이머 사전: { msg_id: QTimer }
        self.periodic_timers: Dict[str, QTimer] = {}
        # ─────────────────────────────────────────────────────────

        # 3) 수신 자동완료 타이머 사전: { msg_id: QTimer }
        self.receive_timers: Dict[str, QTimer] = {}

        # 4) 비주기 수신 횟수 카운트용 사전: { msg_id: int }
        self.receive_counts: Dict[str, int] = {}

        self._init_ui()
        self.tbl_tx.cellDoubleClicked.connect(self._on_tx_double_clicked)

        # ───────── 수신 메시지(tab) 등록 ────────────────────
        # self.RECEIVE_MESSAGES: List[Tuple[msg_id, msg_name]]
        for msg_id, _ in getattr(self, "RECEIVE_MESSAGES", []):
            register_listener(msg_id, self)

        # ── [추가] 자체점검 간이모드(2초 후 정상 보고) ────────────────────
        self._selfcheck_simple = True   # 지금은 간단 모드: 2초 후 → status=1
        self._selfcheck_ready  = False  # 초기엔 0(Unknown)
        QTimer.singleShot(2000, self._mark_selfcheck_ready)
        # ─────────────────────────────────────────────────────────────

    # ── [추가] 2초 후 Ready 마킹 ───────────────────────────────────────
    def _mark_selfcheck_ready(self):
        self._selfcheck_ready = True
    # ────────────────────────────────────────────────────────────────

    # ──────────────── UI 빌드 ───────────────────────
    def _init_ui(self):
        # 테이블 생성
        self.tbl_tx = self._make_tx_table()   # ← 변경: 4열 테이블
        self.tbl_rx = self._make_rx_table()      # 3열 그대로
        self._populate(self.tbl_tx, self.PUSH_MESSAGES, "발신 전")
        self._populate(self.tbl_rx, self.RECEIVE_MESSAGES, "수신 전")

        # 로그
        self.log_tx = self._make_log()
        self.log_rx = self._make_log()

        # 레이아웃
        left = self._side("발신", self.tbl_tx, self.log_tx)
        right = self._side("수신", self.tbl_rx, self.log_rx)
        body = QHBoxLayout()
        body.addWidget(left)
        body.addWidget(right)

        root = QVBoxLayout(self)
        title = QLabel(self.TITLE)
        title.setStyleSheet("font-size:18px;font-weight:600;")
        root.addWidget(title)
        root.addLayout(body)
        root.setContentsMargins(4, 4, 4, 4)

    # ──────────────── Helper ───────────────────────
    def _make_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 3)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태"])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tbl.setStyleSheet("font-size:12px;")
        return tbl

    def _module_human_name(self) -> str:
        """
        KU_ROLE 기준으로 모듈 표시명을 반환.
        monitoring → Mission State Monitor
        mission    → Multi-agent Mission Planner
        decision   → Mission Option Builder
        """
        import os
        role = (os.environ.get("KU_ROLE") or "").lower()
        return {
            "monitoring": "Mission State Monitor",
            "mission":    "Multi-agent Mission Planner",
            "decision":   "Mission Option Builder",
        }.get(role, "Multi-agent Mission Planner")

    # ── [교체] SW 코드(MMR/MSM/MOB) 반환 ─────────────────────────────
    def _sw_code(self) -> str:
        import os
        role = (os.environ.get("KU_ROLE") or "").lower()
        return {"mission": "MMR", "monitoring": "MSM", "decision": "MOB"}.get(role, "MMR")
    # ────────────────────────────────────────────────────────────────

    # ── [교체] 자체 진단 결과(0/1/2) ──────────────────────────────────
    def _self_diag_status(self) -> int:
        """
        0: Unknown(초기 2초)
        1: 정상(간이모드: 2초 경과)
        2: 비정상(간이모드 해제 시에만 실제 점검 실패 시 보고)
        """
        # 2초 전에는 무조건 0
        if not self._selfcheck_ready:
            return 0

        # 간이모드면 2초 경과 후 바로 정상(=1)
        if self._selfcheck_simple:
            return 1

        # ── 아래는 '간이모드 해제'했을 때만 쓰는 실제 점검(지금은 사용 안 함)
        try:
            import importlib.util as _iu
            try:
                from push_center import SEARCH_PREFIXES as _PFX
            except Exception:
                _PFX = ["generator", "push_info", "push", "modules.common.push_info", "modules.common.push"]
            found_push = any(_iu.find_spec(f"{pref}.message0102_push") is not None for pref in _PFX)

            import receive_center as _rc
            reg = getattr(_rc, "_listener_registry", {})
            def z4(mid): 
                s = str(mid)
                return s.zfill(4) if s.isdigit() and len(s) < 4 else s
            rx_ok = True
            for mid, _ in getattr(self, "RECEIVE_MESSAGES", []):
                key = z4(mid)
                lst = reg.get(key) or []
                if self not in lst:
                    rx_ok = False
                    break

            messenger_ok = hasattr(self, "messenger") and (self.messenger is not None)
            return 1 if (found_push and rx_ok and messenger_ok) else 2
        except Exception:
            return 2
    # ────────────────────────────────────────────────────────────────

    def _make_rx_table(self) -> QTableWidget:
        """
        수신 테이블: 4열(메시지ID·이름·상태·데이터) 구성
        """
        tbl = QTableWidget(0, 4)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태", "데이터"])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tbl.setStyleSheet("font-size:12px;")
        tbl.setColumnWidth(3, 60)   # '데이터' 버튼
        return tbl

    def _make_log(self) -> QTextEdit:
        log = QTextEdit()
        log.setReadOnly(True)
        log.setStyleSheet("border:1px solid #345; font-family:Consolas; font-size:11px;")
        log.setPlaceholderText("▶ 로그가 출력됩니다…")
        return log

    def _side(self, caption: str, tbl: QTableWidget, log: QTextEdit) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lbl = QLabel(f"<b>{caption} 데이터 목록</b>")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("font-size:14px;")
        lay.addWidget(lbl)
        lay.addWidget(tbl, 5)
        lay.addWidget(log, 3)
        lay.setContentsMargins(0, 0, 0, 0)
        return w

    def _populate(self,
                tbl: QTableWidget,
                data: Sequence[Tuple[str, str]],
                default_state: str):
        """
        테이블에 (msg_id, msg_name) 목록을 채운다.
        * 발신 테이블(5열) → '발신'·'데이터' 버튼
        * 수신 테이블(4열) → '데이터' 버튼만
        """
        col_cnt     = tbl.columnCount()
        is_tx_table = col_cnt == 5
        is_rx_table = col_cnt == 4

        for mid, name in data:
            r = tbl.rowCount()
            tbl.insertRow(r)
            tbl.setItem(r, 0, QTableWidgetItem(mid))
            tbl.setItem(r, 1, QTableWidgetItem(name))
            tbl.setItem(r, 2, QTableWidgetItem(default_state))

            if is_tx_table:
                btn_send = QPushButton("발신")
                btn_send.clicked.connect(lambda _, row=r: self._on_tx_button_clicked(row))
                tbl.setCellWidget(r, 3, btn_send)

                btn_view = QPushButton("보기")
                btn_view.clicked.connect(lambda _, row=r: self._on_tx_view_button_clicked(row))
                tbl.setCellWidget(r, 4, btn_view)

            elif is_rx_table:
                btn_view = QPushButton("보기")
                btn_view.clicked.connect(lambda _, row=r: self._on_rx_view_button_clicked(row))
                tbl.setCellWidget(r, 3, btn_view)

    def _on_rx_view_button_clicked(self, row: int):
        """
        수신 테이블 '보기' 버튼 클릭 → 최근 RAW(JSON) 팝업
        """
        item = self.tbl_rx.item(row, 0)
        mid  = item.text()
        raw  = item.data(Qt.UserRole)

        if not raw:
            self._show_data_dialog(mid, "(데이터 없음)")
            return

        txt = raw.decode(errors="ignore")
        m   = re.search(r"\{.*\}", txt, flags=re.S)
        if not m:
            self._show_data_dialog(mid, txt.strip())
            return

        try:
            obj     = json.loads(m.group(0))
            pretty  = json.dumps(obj, indent=2, ensure_ascii=False)
            self._show_data_dialog(mid, pretty)
        except Exception:
            self._show_data_dialog(mid, m.group(0).strip())


    def _on_tx_view_button_clicked(self, row: int):
        """
        데이터 보기 버튼 클릭: 저장된 RAW 문자열에서 JSON만 추출해
        pretty-print(indent=2) 형태로 팝업 표시.
        """
        item = self.tbl_tx.item(row, 0)
        mid  = item.text()
        raw  = item.data(Qt.UserRole)

        # RAW 데이터가 없을 때
        if not raw:
            self._show_data_dialog(mid, "(데이터 없음)")
            return

        # ① bytes → str
        txt = raw.decode(errors="ignore")

        # ② JSON 블록 추출: BODY 뒤 중괄호 … 마지막 중괄호까지
        m = re.search(r"\{.*\}", txt, flags=re.S)
        if not m:
            # JSON 못 찾으면 원문 그대로
            self._show_data_dialog(mid, txt.strip())
            return

        try:
            obj = json.loads(m.group(0))
            pretty = json.dumps(obj, indent=2, ensure_ascii=False)
            self._show_data_dialog(mid, pretty)
        except Exception:
            # 파싱 실패 시 원문 그대로
            self._show_data_dialog(mid, m.group(0).strip())


    def _show_data_dialog(self, msg_id: str, text: str):
        """
        메시지별 상세 데이터 팝업
        """
        dlg = QDialog(self)
        dlg.setWindowTitle(f"{msg_id} – 발신 데이터")
        vbox = QVBoxLayout(dlg)

        txt = QTextEdit()
        txt.setReadOnly(True)
        txt.setPlainText(text)
        txt.setStyleSheet("font-family:Consolas; font-size:11px;")
        vbox.addWidget(txt)

        btns = QDialogButtonBox(QDialogButtonBox.Close)
        btns.rejected.connect(dlg.reject)
        vbox.addWidget(btns)

        dlg.resize(540, 320)
        dlg.exec_()


    def _on_tx_button_clicked(self, row: int):
        """
        발신 버튼 클릭 시: 더블-클릭과 동일한 처리 경로로 보낸다.
        """
        self._on_tx_double_clicked(row, 0)   # col=0(dummy)

    def _make_tx_table(self) -> QTableWidget:
        """
        발신 테이블: 5열(메시지ID·이름·상태·발신·보기) 구성
        """
        tbl = QTableWidget(0, 5)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태", "발신", "데이터"])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tbl.setStyleSheet("font-size:12px;")
        tbl.setColumnWidth(3, 60)   # 발신 버튼
        tbl.setColumnWidth(4, 60)   # 데이터 버튼
        return tbl

    # ──────────── Public API (메시지 완료) ───────────
    def mark_sent(self, msg_id: str, raw: bytes | None = None):
        self._update_state(self.tbl_tx, msg_id, "발신 완료")
        self._write_log(self.log_tx, "SEND", msg_id, raw)

    def mark_received(self, msg_id: str, raw: bytes | None = None):
        freq = self.periodic_config.get(msg_id, None)

        # 1) 테이블 상태 업데이트: 수신 중 또는 수신 완료
        for r in range(self.tbl_rx.rowCount()):
            if self.tbl_rx.item(r, 0).text() == msg_id:
                item = self.tbl_rx.item(r, 2)
                from PyQt5.QtGui import QColor

                if freq:
                    # 주기 수신일 때: "수신 중(XXHz)"
                    item.setText(f"수신 중({freq}Hz)")
                    item.setForeground(QColor("blue"))

                    # 기존 타이머 있으면 중지
                    if msg_id in self.receive_timers:
                        self.receive_timers[msg_id].stop()
                        self.receive_timers[msg_id].deleteLater()
                        del self.receive_timers[msg_id]

                    # 새 타이머: 2/freq 초 후에 자동으로 "수신 완료"로 전환
                    timer = QTimer(self)
                    timer.setSingleShot(True)
                    timeout_ms = int((2000.0 / freq))  # 2초/freq 면 충분합니다
                    timer.setInterval(timeout_ms)
                    timer.timeout.connect(lambda mid=msg_id: self._receive_timeout(mid))
                    timer.start()
                    self.receive_timers[msg_id] = timer
                else:
                    # 비주기 수신일 때: 횟수 카운트 후 "수신 완료(n회)"
                    count = self.receive_counts.get(msg_id, 0) + 1
                    self.receive_counts[msg_id] = count
                    item.setText(f"수신 완료({count})")
                    item.setForeground(QColor("blue"))
                if raw:
                    self.tbl_rx.item(r, 0).setData(Qt.UserRole, raw)
                break

        # 2) 로그 기록
        self._write_log(self.log_rx, "RECV", msg_id, raw)

    # ──────────── 더블클릭 → Push (주기/비주기) ────────────────
    def _on_tx_double_clicked(self, row: int, _col: int):
        msg_id = self.tbl_tx.item(row, 0).text()
        freq = self.periodic_config.get(msg_id, None)
        body = self._build_overridden_body(msg_id)  # ★ 0102 바디 강제

        if freq is None:
            ok = push_message(
                msg_id, self.messenger,
                on_done=lambda mid, raw: self._mark_single_sent(row, mid, raw),
                body_dict=body
            )
            if not ok:
                self.tbl_tx.item(row, 2).setText("발신 실패")
        else:
            if msg_id in self.periodic_timers:
                self._stop_periodic_send(msg_id, row)
            else:
                self._start_periodic_send(msg_id, row, freq)

    def _receive_timeout(self, msg_id: str):
        """
        주기 수신 중 타이머가 만료되면 호출됩니다.
        tbl_rx 상태를 '수신 완료'(파란색)로 바꾸고, 타이머 삭제
        """
        for r in range(self.tbl_rx.rowCount()):
            if self.tbl_rx.item(r, 0).text() == msg_id:
                item = self.tbl_rx.item(r, 2)
                from PyQt5.QtGui import QColor
                item.setText("수신 완료")
                item.setForeground(QColor("blue"))
                break

        # 타이머 객체 정리
        if msg_id in self.receive_timers:
            self.receive_timers[msg_id].deleteLater()
            del self.receive_timers[msg_id]


    def _mark_single_sent(self, row: int, msg_id: str, raw: bytes | None):
        """
        비주기 전송이 완료된 후 호출: 상태를 '발신 완료'로 업데이트하고 로그 기록
        """
        self.tbl_tx.item(row, 2).setText("발신 완료")
        if raw:
            self.tbl_tx.item(row, 0).setData(Qt.UserRole, raw)
        self._write_log(self.log_tx, "SEND", msg_id, raw)

    # ──────────── 주기 전송 관리 메서드 ─────────────────────
    def _start_periodic_send(self, msg_id: str, row: int, freq_hz: float):
        """
        주기 전송 시작: 지정된 주파수(freq_hz)마다 push_message 호출.
        """
        interval_ms = int(1000.0 / freq_hz)
        timer = QTimer(self)
        timer.setInterval(interval_ms)
        timer.timeout.connect(lambda: self._periodic_timeout(msg_id, row))
        timer.start()

        self.periodic_timers[msg_id] = timer
        # 타이머 시작 직후 상태를 '전송중'으로 표시
        self.tbl_tx.item(row, 2).setText(f"전송중 ({freq_hz}Hz)")

    def _stop_periodic_send(self, msg_id: str, row: int):
        """
        주기 전송 중지: 해당 msg_id의 QTimer를 멈추고 제거.
        전송이 중지되면, tbl_tx 상태는 '전송 정지'로, 
        tbl_rx 상태는 '수신 완료'(파란색) 로 각각 업데이트.
        """
        # 1) 주기 전송용 타이머 정지
        timer = self.periodic_timers.get(msg_id)
        if timer:
            timer.stop()
            timer.deleteLater()
            del self.periodic_timers[msg_id]

        # 2) tbl_tx 상태 업데이트
        self.tbl_tx.item(row, 2).setText("전송 정지")

        # 3) 같은 msg_id에 대해 tbl_rx 상태를 '수신 완료'(파란색) 로 변경
        for r in range(self.tbl_rx.rowCount()):
            if self.tbl_rx.item(r, 0).text() == msg_id:
                recv_item = self.tbl_rx.item(r, 2)
                recv_item.setText("수신 완료")
                from PyQt5.QtGui import QColor
                recv_item.setForeground(QColor("blue"))
                break

    def _build_overridden_body(self, msg_id: str):
        """
        0102 전송 시 바디를 표준 스키마로 생성:
        { "timestamp": ms_since_2000, "source": "MMR|MSM|MOB", "status": 0|1|2 }
        """
        if str(msg_id).strip() != "0102":
            return None
        return {
            "timestamp": _now_ms_since_2000(),
            "source": self._sw_code(),
            "status": self._self_diag_status(),
        }
    
    def _periodic_timeout(self, msg_id: str, row: int):
        """
        주기 타이머 만료 시 호출. 상태는 '전송중' 유지, 로그만 기록.
        """
        body = self._build_overridden_body(msg_id)  # ★ 0102 바디 강제
        ok = push_message(
            msg_id, self.messenger,
            on_done=lambda mid, raw: self._log_only(row, mid, raw),
            body_dict=body
        )
        if not ok:
            self.tbl_tx.item(row, 2).setText("전송 실패")

    def _log_only(self, row: int, msg_id: str, raw: bytes | None):
        """
        주기 전송 시 상태는 그대로 두고 로그만 기록.
        RAW 데이터도 셀(UserRole)에 업데이트해 데이터 보기 팝업에서 활용.
        """
        if raw:
            self.tbl_tx.item(row, 0).setData(Qt.UserRole, raw)
        self._write_log(self.log_tx, "SEND", msg_id, raw)

    # ──────────── 내부 유틸 ───────────────────────
    def _update_state(self, tbl: QTableWidget, msg_id: str, state: str):
        for r in range(tbl.rowCount()):
            if tbl.item(r, 0).text() == msg_id:
                tbl.item(r, 2).setText(state)
                break

    def _write_log(self,
                log_w: QTextEdit,
                tag: str,
                msg_id: str,
                raw: bytes | None):
        """
        로그 출력:
        - 0102일 때만 BODY JSON을 'Timestamp, Status, SourceModuleName' 순서로 재정렬해서 보기 좋게 표시
        - 값은 변경하지 않음(재계산/추가 금지), 'sent' 같은 내부 키는 제거
        """
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {tag:<4} : {msg_id}"

        if raw:
            try:
                decoded = raw.decode(errors="ignore")
                if str(msg_id).strip() == "0102":
                    import json, re
                    from collections import OrderedDict
                    m = re.search(r"\{.*\}", decoded, flags=re.S)
                    if m:
                        try:
                            obj = json.loads(m.group(0))
                            # 필요한 값만 그대로 꺼내고, 순서만 보장
                            payload = OrderedDict()
                            if "Timestamp" in obj:
                                payload["Timestamp"] = obj["Timestamp"]
                            elif "timestamp" in obj:
                                payload["Timestamp"] = obj["timestamp"]
                            if "Status" in obj:
                                payload["Status"] = obj["Status"]
                            elif "status" in obj:
                                payload["Status"] = obj["status"]
                            if "SourceModuleName" in obj:
                                payload["SourceModuleName"] = obj["SourceModuleName"]
                            elif "source" in obj:
                                payload["SourceModuleName"] = obj["source"]
                            elif "requestModuleName" in obj:
                                payload["SourceModuleName"] = obj["requestModuleName"]
                            # 불필요한 내부 키 제거
                            for k in ("sent", ):
                                payload.pop(k, None)

                            new_json = json.dumps(payload, ensure_ascii=False)
                            decoded = decoded[:m.start()] + new_json + decoded[m.end():]
                        except Exception:
                            pass
                line += f"\n{decoded}"
            except Exception:
                line += " (binary)"

        log_w.append(line)