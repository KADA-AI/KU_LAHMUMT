# /mnt/data/csc_tab_base.py
from __future__ import annotations
from datetime import datetime
from typing import Optional, Sequence, Tuple, Dict

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QTextEdit, QLabel, QSizePolicy, QHeaderView, QPushButton, QDialog, QDialogButtonBox
)
import json
import re
import os
import socket

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

    [UDP 모니터링 전송 설정]
    - 우선순위 1: KU_MONITOR_UDP="HOST:PORT" (전체 모듈 공용)
    - 우선순위 2: 모듈별 호스트/포트
        KU_MON_HOST (기본 127.0.0.1)
        KU_MON_ASSIGNMENT_PORT (기본 46981)  # mission_planning / mission
        KU_MON_MONITORING_PORT (기본 46982)  # monitoring
        KU_MON_DECISION_PORT   (기본 46983)  # decision / decision_support
        KU_MON_INFO_PORT       (기본 46984)  # info / info_manage
    - KU_ROLE 별 role 코드:
        monitoring → MSM, mission → MMR, decision → MOB, info → INF
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
            '0501': 5,
            '0502': None,
            '0503': None,
            '0504': None,
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
        # 3) 수신 자동완료 타이머 사전: { msg_id: QTimer }
        self.receive_timers: Dict[str, QTimer] = {}
        # 4) 비주기 수신 횟수 카운트용 사전: { msg_id: int }
        self.receive_counts: Dict[str, int] = {}

        # ── UDP 모니터 연결(옵션) ─────────────────────────────
        self._udp_addr = self._resolve_udp_target()       # ('host', port) 또는 None
        self._udp_sock: Optional[socket.socket] = None
        if self._udp_addr:
            try:
                self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            except Exception:
                self._udp_sock = None
        # ───────────────────────────────────────────────────

        self._init_ui()
        self.tbl_tx.cellDoubleClicked.connect(self._on_tx_double_clicked)

        # 수신 메시지(tab) 등록
        for msg_id, _ in getattr(self, "RECEIVE_MESSAGES", []):
            register_listener(msg_id, self)

        # 자체점검 간이모드(2초 후 정상 보고)
        self._selfcheck_simple = True
        self._selfcheck_ready  = False
        QTimer.singleShot(2000, self._mark_selfcheck_ready)

    def _mark_selfcheck_ready(self):
        self._selfcheck_ready = True

    # ──────────────── UI 빌드 ───────────────────────
    def _init_ui(self):
        self.tbl_tx = self._make_tx_table()
        self.tbl_rx = self._make_rx_table()
        self._populate(self.tbl_tx, self.PUSH_MESSAGES, "발신 전")
        self._populate(self.tbl_rx, self.RECEIVE_MESSAGES, "수신 전")

        self.log_tx = self._make_log()
        self.log_rx = self._make_log()

        left = self._side("발신", self.tbl_tx, self.log_tx)
        right = self._side("수신", self.tbl_rx, self.log_rx)
        body = QHBoxLayout()
        body.addWidget(left)
        body.addWidget(right)

        root = QVBoxLayout(self)
        title = QLabel(self.TITLE)
        title.setStyleSheet("font-size:18px;font-weight:600;")
        self._title_label = title
        root.addWidget(title)
        root.addLayout(body)
        root.setContentsMargins(4, 4, 4, 4)
        
    def _make_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 3)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태"])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tbl.setStyleSheet("font-size:12px;")
        return tbl

    def _role_norm(self) -> str:
        """
        KU_ROLE 값을 표준화:
        mission_planning→mission, decision_support→decision, info_manage→info
        """
        role = (os.environ.get("KU_ROLE") or "").lower()
        return {
            "mission_planning": "mission",
            "decision_support": "decision",
            "info_manage": "info",
        }.get(role, role)

    def _module_human_name(self) -> str:
        """
        (기존) KU_ROLE 기준 모듈 표시명 반환.
        ※ 내부 UI 용도로 남겨둠. 모니터링 전송은 _sw_code() 사용.
        """
        role = self._role_norm()
        return {
            "monitoring": "Mission State Monitor",
            "mission":    "Multi-agent Mission Planner",
            "decision":   "Mission Option Builder",
            "info":       "Information Manager",
        }.get(role, "Multi-agent Mission Planner")

    def _sw_code(self) -> str:
        """
        KU_ROLE → 코드:
        monitoring: MSM, mission: MMR, decision: MOB, info: INF
        """
        role = self._role_norm()
        return {"mission": "MMR", "monitoring": "MSM", "decision": "MOB", "info": "INF"}.get(role, "MMR")

    def _self_diag_status(self) -> int:
        if not self._selfcheck_ready:
            return 0
        if self._selfcheck_simple:
            return 1
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

    def _make_rx_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 4)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태", "데이터"])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tbl.setStyleSheet("font-size:12px;")
        tbl.setColumnWidth(3, 60)
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
        item = self.tbl_tx.item(row, 0)
        mid  = item.text()
        raw  = item.data(Qt.UserRole)

        if not raw:
            self._show_data_dialog(mid, "(데이터 없음)")
            return

        txt = raw.decode(errors="ignore")
        m = re.search(r"\{.*\}", txt, flags=re.S)
        if not m:
            self._show_data_dialog(mid, txt.strip())
            return

        try:
            obj = json.loads(m.group(0))
            pretty = json.dumps(obj, indent=2, ensure_ascii=False)
            self._show_data_dialog(mid, pretty)
        except Exception:
            self._show_data_dialog(mid, m.group(0).strip())

    def _show_data_dialog(self, msg_id: str, text: str):
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
        self._on_tx_double_clicked(row, 0)

    def _make_tx_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 5)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태", "발신", "데이터"])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        tbl.setStyleSheet("font-size:12px;")
        tbl.setColumnWidth(3, 60)
        tbl.setColumnWidth(4, 60)
        return tbl

    # ──────────── Public API (메시지 완료) ───────────
    def mark_sent(self, msg_id: str, raw: bytes | None = None):
        self._update_state(self.tbl_tx, msg_id, "발신 완료")
        if raw:
            try:
                for r in range(self.tbl_tx.rowCount()):
                    item = self.tbl_tx.item(r, 0)
                    if item and item.text() == msg_id:
                        item.setData(Qt.UserRole, raw)
                        break
            except Exception:
                pass
        self._write_log(self.log_tx, "SEND", msg_id, raw)

    def mark_received(self, msg_id: str, raw: bytes | None = None):
        freq = self.periodic_config.get(msg_id, None)

        # 1) 테이블 상태 업데이트
        for r in range(self.tbl_rx.rowCount()):
            if self.tbl_rx.item(r, 0).text() == msg_id:
                item = self.tbl_rx.item(r, 2)

                if freq:
                    item.setText(f"수신 중({freq}Hz)")
                    item.setForeground(QColor("blue"))

                    if msg_id in self.receive_timers:
                        self.receive_timers[msg_id].stop()
                        self.receive_timers[msg_id].deleteLater()
                        del self.receive_timers[msg_id]

                    timer = QTimer(self)
                    timer.setSingleShot(True)
                    timeout_ms = int((2000.0 / freq))
                    timer.setInterval(timeout_ms)
                    timer.timeout.connect(lambda mid=msg_id: self._receive_timeout(mid))
                    timer.start()
                    self.receive_timers[msg_id] = timer
                else:
                    count = self.receive_counts.get(msg_id, 0) + 1
                    self.receive_counts[msg_id] = count
                    item.setText(f"수신 완료({count})")
                    item.setForeground(QColor("blue"))
                if raw:
                    self.tbl_rx.item(r, 0).setData(Qt.UserRole, raw)
                break

        # 2) 로그 기록
        self._write_log(self.log_rx, "RECV", msg_id, raw)

        # 3) 콘솔 요약 JSON (UDP 포맷과 동일)
        self._print_received_summary(msg_id, raw)

        # 4) UDP 모니터로 전송
        self._emit_rx_monitor(msg_id, raw)

    # ──────────── 더블클릭 → Push (주기/비주기) ────────────────
    def _on_tx_double_clicked(self, row: int, _col: int):
        msg_id = self.tbl_tx.item(row, 0).text()
        freq = self.periodic_config.get(msg_id, None)
        body = self._build_overridden_body(msg_id)

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
        for r in range(self.tbl_rx.rowCount()):
            if self.tbl_rx.item(r, 0).text() == msg_id:
                item = self.tbl_rx.item(r, 2)
                item.setText("수신 완료")
                item.setForeground(QColor("blue"))
                break
        if msg_id in self.receive_timers:
            self.receive_timers[msg_id].deleteLater()
            del self.receive_timers[msg_id]

    def _mark_single_sent(self, row: int, msg_id: str, raw: bytes | None):
        self.tbl_tx.item(row, 2).setText("발신 완료")
        if raw:
            self.tbl_tx.item(row, 0).setData(Qt.UserRole, raw)
        self._write_log(self.log_tx, "SEND", msg_id, raw)

    # ──────────── 주기 전송 관리 ─────────────────────
    def _start_periodic_send(self, msg_id: str, row: int, freq_hz: float):
        interval_ms = int(1000.0 / freq_hz)
        timer = QTimer(self)
        timer.setInterval(interval_ms)
        timer.timeout.connect(lambda: self._periodic_timeout(msg_id, row))
        timer.start()

        self.periodic_timers[msg_id] = timer
        self.tbl_tx.item(row, 2).setText(f"전송중 ({freq_hz}Hz)")

    def _stop_periodic_send(self, msg_id: str, row: int):
        timer = self.periodic_timers.get(msg_id)
        if timer:
            timer.stop()
            timer.deleteLater()
            del self.periodic_timers[msg_id]

        self.tbl_tx.item(row, 2).setText("전송 정지")

        for r in range(self.tbl_rx.rowCount()):
            if self.tbl_rx.item(r, 0).text() == msg_id:
                recv_item = self.tbl_rx.item(r, 2)
                recv_item.setText("수신 완료")
                recv_item.setForeground(QColor("blue"))
                break

    def _build_overridden_body(self, msg_id: str):
        if str(msg_id).strip() != "0102":
            return None
        return {
            "timestamp": _now_ms_since_2000(),
            "source": self._sw_code(),
            "status": self._self_diag_status(),
        }
    
    def _periodic_timeout(self, msg_id: str, row: int):
        body = self._build_overridden_body(msg_id)
        ok = push_message(
            msg_id, self.messenger,
            on_done=lambda mid, raw: self._log_only(row, mid, raw),
            body_dict=body
        )
        if not ok:
            self.tbl_tx.item(row, 2).setText("전송 실패")

    def _log_only(self, row: int, msg_id: str, raw: bytes | None):
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
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {tag:<4} : {msg_id}"

        if raw:
            try:
                decoded = raw.decode(errors="ignore")
                if str(msg_id).strip() == "0102":
                    from collections import OrderedDict
                    m = re.search(r"\{.*\}", decoded, flags=re.S)
                    if m:
                        try:
                            obj = json.loads(m.group(0))
                            payload = OrderedDict()
                            ts_val  = obj.get("timestamp", obj.get("Timestamp", None))
                            st_val  = obj.get("status",    obj.get("Status",    None))
                            src_val = (
                                obj.get("source",
                                    obj.get("Source",
                                        obj.get("SourceModuleName",
                                            obj.get("requestModuleName", None)))))
                            if ts_val is not None:
                                payload["timestamp"] = ts_val
                            if st_val is not None:
                                payload["status"] = st_val
                            if src_val is not None:
                                payload["source"] = src_val
                            payload.pop("sent", None)
                            new_json = json.dumps(payload, ensure_ascii=False)
                            decoded = decoded[:m.start()] + new_json + decoded[m.end():]
                        except Exception:
                            pass
                line += f"\n{decoded}"
            except Exception:
                line += " (binary)"

        log_w.append(line)

    # ────────────────────────────────────────────────────────────────
    # [신규] 수신 데이터 요약 JSON 프린트 + UDP 모니터 전송
    # ────────────────────────────────────────────────────────────────
    def _print_received_summary(self, msg_id: str, raw: bytes | None):
        """
        콘솔 프린트 비활성화.
        UDP 모니터 전송은 _emit_rx_monitor()가 계속 수행함.
        디버그로 콘솔 로그가 필요하면 KU_MON_RX_PRINT=1 로 일시 활성화.
        """
        import os, json  # 상단에 이미 있으면 무시됨
        if str(os.environ.get("KU_MON_RX_PRINT", "0")).lower() not in ("1", "true", "on", "yes"):
            return
        # ── 아래는 디버그용(환경변수로 켰을 때만 동작) ──
        try:
            s = str(msg_id)
            extracted = s.zfill(4) if s.isdigit() and len(s) < 4 else s
            payload = {"kind": "rx", "msg_id": extracted, "role": self._sw_code()}
            print(json.dumps(payload, ensure_ascii=False), flush=True)
        except Exception:
            pass

    def _extract_msg_id_from_raw(self, raw: bytes, allowed_ids: set) -> Optional[str]:
        """
        RAW 바이트 스트림에서 가능한 메시지 ID를 추출한다.
        1) 포함된 JSON이 있으면 파싱해서 id 후보 키에서 시도
        2) 실패 시 텍스트 전체에서 4자리 숫자 토큰을 스캔하여 allowed_ids에 존재하는 첫 매칭을 선택
        """
        try:
            txt = raw.decode(errors="ignore")
            m = re.search(r"\{.*\}", txt, flags=re.S)
            if m:
                try:
                    obj = json.loads(m.group(0))
                    candidates = [
                        obj.get("messageId"),
                        obj.get("message_id"),
                        obj.get("msg_id"),
                        obj.get("MessageID"),
                        obj.get("MessageId"),
                        obj.get("id"),
                    ]
                    for c in candidates:
                        if isinstance(c, (str, int)):
                            s = str(c)
                            s4 = s.zfill(4) if s.isdigit() and len(s) < 4 else s
                            if s4 in allowed_ids:
                                return s4
                except Exception:
                    pass
            for m2 in re.finditer(r"\b\d{4}\b", txt):
                s = m2.group(0)
                if s in allowed_ids:
                    return s
        except Exception:
            pass
        return None

    # ────────────────────────────────────────────────────────────────
    # [신규] UDP 모니터 전송 부분
    # ────────────────────────────────────────────────────────────────
    def _resolve_udp_target(self) -> Optional[tuple]:
        """
        1) KU_MONITOR_UDP="HOST:PORT" 우선 사용
        2) 없으면 KU_ROLE 표준화 후 모듈별 포트 환경변수/기본값 적용
        """
        # 우선 구버전 호환
        try:
            spec = (os.environ.get("KU_MONITOR_UDP") or "").strip()
            if spec:
                m = re.match(r"^\s*([^:]+)\s*:\s*(\d{2,5})\s*$", spec)
                if m:
                    host, port = m.group(1), int(m.group(2))
                    return (host, port)
        except Exception:
            pass

        # 모듈별 설정
        try:
            host = (os.environ.get("KU_MON_HOST") or "127.0.0.1").strip()
            role = self._role_norm()
            if role in ("mission", "mission_planning"):
                port = int(os.environ.get("KU_MON_ASSIGNMENT_PORT", "46981"))
            elif role == "monitoring":
                port = int(os.environ.get("KU_MON_MONITORING_PORT", "46982"))
            elif role in ("decision", "decision_support"):
                port = int(os.environ.get("KU_MON_DECISION_PORT", "46983"))
            elif role in ("info", "info_manage"):
                port = int(os.environ.get("KU_MON_INFO_PORT", "46984"))
            else:
                return None
            return (host, port)
        except Exception:
            return None

    def _send_udp_monitor(self, payload: dict):
        """
        UDP로 payload(JSON) 전송. 오류는 조용히 무시.
        """
        if not self._udp_sock or not self._udp_addr:
            return
        try:
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self._udp_sock.sendto(data, self._udp_addr)
        except Exception:
            pass

    def _emit_rx_monitor(self, msg_id: str, raw: bytes | None):
        """
        수신 이벤트를 UDP 모니터로 전송. 포맷:
        {"kind":"rx","msg_id":"0301","role":"MSM"}
        """
        try:
            allowed_ids = set(map(str, self.periodic_config.keys()))
            for mid, _ in getattr(self, "PUSH_MESSAGES", ()):
                allowed_ids.add(str(mid).zfill(4) if str(mid).isdigit() else str(mid))
            for mid, _ in getattr(self, "RECEIVE_MESSAGES", ()):
                allowed_ids.add(str(mid).zfill(4) if str(mid).isdigit() else str(mid))

            extracted = None
            if raw:
                extracted = self._extract_msg_id_from_raw(raw, allowed_ids)
            if not extracted:
                s = str(msg_id)
                extracted = s.zfill(4) if s.isdigit() and len(s) < 4 else s

            role_code = self._sw_code()
            payload = {
                "kind": "rx",
                "msg_id": extracted,
                "role": role_code
            }
            if payload.get("msg_id") == "0901" and role_code == "MOB":
                try:
                    host, port = self._udp_addr if self._udp_addr else ("?", "?")
                    print(f"[UDP SEND RX] addr={host}:{port} data={payload}", flush=True)
                except Exception:
                    pass
            self._send_udp_monitor(payload)
        except Exception:
            pass
