# /mnt/data/csc_tab_base.py
from __future__ import annotations
from datetime import datetime
from collections import deque
import time
from typing import Optional, Sequence, Tuple, Dict

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTableWidget, QTableWidgetItem,
    QTextEdit, QLabel, QSizePolicy, QHeaderView, QPushButton, QDialog, QDialogButtonBox
)
import json
import re
import os

from PyQt5.QtGui import QColor, QTextCursor
from PyQt5.QtCore import Qt, QTimer
try:
    from push_center import push_message
except ModuleNotFoundError:
    from modules.common.push_center import push_message
try:
    from modules.common.receive_center import register_listener
except ModuleNotFoundError:
    from receive_center import register_listener

_EPOCH2000_MS = 946684800000
def _now_ms_since_2000():
    import time
    return int(time.time() * 1000) - _EPOCH2000_MS

class CSCTabBase(QWidget):
    """Common CSC tab base."""

    # 서브클래스에서 오버라이드할 상수 -----------------
    HISTORY_LIMIT: int = 50
    HISTORY_SEPARATOR: str = "=" * 64
    LOG_ENTRY_LIMIT: int | None = None
    LOG_PAYLOAD_MAX_CHARS: int | None = None
    LOG_PAYLOAD_TRUNC_SUFFIX: str = " ...(truncated)"
    LOG_FLUSH_INTERVAL_MS: int | None = None

    TITLE: str = "CSC"
    PUSH_MESSAGES: Sequence[Tuple[str, str]] = ()
    RECEIVE_MESSAGES: Sequence[Tuple[str, str]] = ()
    # -------------------------------------------------

    def __init__(self, *, messenger, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.messenger = messenger

        self.periodic_config: Dict[str, Optional[float]] = {
            '0000': None,  
            '0001': None,
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


    # ───────── Payload History 유틸 ─────────
    def _coerce_payload_bytes(self, payload) -> bytes | None:
        if payload is None:
            return None
        if isinstance(payload, bytes):
            return payload
        if isinstance(payload, bytearray):
            return bytes(payload)
        if isinstance(payload, str):
            return payload.encode("utf-8", "ignore")
        try:
            return bytes(payload)
        except Exception:
            return None

    def _normalize_payload_history(self, payload) -> list[dict]:
        history: list[dict] = []
        if payload is None:
            return history

        if isinstance(payload, list):
            iterable = payload
        else:
            iterable = [payload]

        for entry in iterable:
            ts = None
            raw_obj = entry
            if isinstance(entry, dict):
                ts = entry.get("ts")
                raw_obj = entry.get("raw")
            elif isinstance(entry, (tuple, list)) and len(entry) >= 2:
                ts = entry[0]
                raw_obj = entry[1]
            raw_bytes = self._coerce_payload_bytes(raw_obj)
            if raw_bytes is None:
                continue
            history.append({"ts": ts, "raw": raw_bytes})
        return history

    def _append_payload_history(self, item: QTableWidgetItem | None, raw: bytes | None):
        if item is None:
            return []
        raw_bytes = self._coerce_payload_bytes(raw)
        if raw_bytes is None:
            return self._normalize_payload_history(item.data(Qt.UserRole))

        history = self._normalize_payload_history(item.data(Qt.UserRole))
        now = time.time()
        history.append({
            "ts": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "ms": int(now * 1000),
            "raw": raw_bytes
        })
        limit = getattr(self, "HISTORY_LIMIT", 50) or 50
        if len(history) > limit:
            history = history[-limit:]
        item.setData(Qt.UserRole, history)
        return history

    def _latest_payload_bytes(self, payload) -> bytes:
        history = self._normalize_payload_history(payload)
        return history[-1]["raw"] if history else b""

    def _format_payload_for_display(self, raw: bytes | None) -> str:
        if not raw:
            return "(데이터 없음)"
        txt = raw.decode(errors="ignore")
        m = re.search(r"\{.*\}", txt, flags=re.S)
        if not m:
            stripped = txt.strip()
            return stripped or "(데이터 없음)"
        try:
            obj = json.loads(m.group(0))
            return json.dumps(obj, indent=2, ensure_ascii=False)
        except Exception:
            return m.group(0).strip() or "(데이터 없음)"

    def _history_timestamps(self, history: list[dict]) -> list[float]:
        result: list[float] = []
        for entry in history:
            ms = entry.get("ms")
            if ms is None:
                ts = entry.get("ts")
                if isinstance(ts, str):
                    try:
                        dt = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S")
                        ms = int(dt.timestamp() * 1000)
                    except Exception:
                        continue
            if ms is None:
                continue
            try:
                result.append(float(ms) / 1000.0)
            except Exception:
                continue
        result.sort()
        return result

    def _calc_recent_frequency(self, history: list[dict], window_sec: float = 10.0) -> float | None:
        timestamps = self._history_timestamps(history)
        if len(timestamps) < 2:
            return None
        latest = timestamps[-1]
        cutoff = latest - window_sec
        recent = [t for t in timestamps if t >= cutoff]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        if span <= 0:
            return None
        return (len(recent) - 1) / span


    def _format_rate_state(self, label: str, planned: float | None, actual: float | None) -> str:
        rate_parts = []
        if planned is not None:
            rate_parts.append(f"{planned:g}Hz")
        if actual is not None:
            rate_parts.append(f"{actual:.2f}Hz")
        if rate_parts:
            joined = " / ".join(rate_parts)
            return f"{label}({joined})"
        return label

    def _format_history_for_dialog(self, history: list[dict]) -> str:
        if not history:
            return "(데이터 없음)"
        total = len(history)
        separator = getattr(self, "HISTORY_SEPARATOR", "=" * 64)
        parts: list[str] = []

        for idx, entry in enumerate(reversed(history), 1):
            ts = entry.get("ts")
            raw = entry.get("raw")
            body = self._format_payload_for_display(raw)
            meta_tokens = []
            if total > 1:
                meta_tokens.append(f"{idx}/{total}")
            if ts:
                meta_tokens.append(ts)
            header = " ".join(meta_tokens)
            if header:
                parts.append(f"{separator}\n[{header}]\n{body}")
            else:
                parts.append(f"{separator}\n{body}")

        return "\n".join(parts)


    def _on_rx_view_button_clicked(self, row: int):
        item = self.tbl_rx.item(row, 0)
        mid  = item.text()
        history = self._normalize_payload_history(item.data(Qt.UserRole))

        if not history:
            self._show_data_dialog(mid, "(데이터 없음)")
            return

        rendered = self._format_history_for_dialog(history)
        self._show_data_dialog(mid, rendered)

    def _on_tx_view_button_clicked(self, row: int):
        item = self.tbl_tx.item(row, 0)
        mid  = item.text()
        history = self._normalize_payload_history(item.data(Qt.UserRole))

        if not history:
            self._show_data_dialog(mid, "(데이터 없음)")
            return

        rendered = self._format_history_for_dialog(history)
        self._show_data_dialog(mid, rendered)

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
        row_index: int | None = None
        history: list[dict] = []
        try:
            for r in range(self.tbl_tx.rowCount()):
                item = self.tbl_tx.item(r, 0)
                if item and item.text() == msg_id:
                    row_index = r
                    if raw is not None:
                        history = self._append_payload_history(item, raw)
                    else:
                        history = self._normalize_payload_history(item.data(Qt.UserRole))
                    break
        except Exception:
            history = []

        freq = self.periodic_config.get(msg_id, None)
        if freq and row_index is not None:
            actual = self._calc_recent_frequency(history)
            state_item = self.tbl_tx.item(row_index, 2)
            if state_item:
                state_item.setText(self._format_rate_state("주기송신", freq, actual))
                state_item.setForeground(QColor("blue"))

        self._write_log(self.log_tx, "SEND", msg_id, raw)

    def mark_received(self, msg_id: str, raw: bytes | None = None):
        freq = self.periodic_config.get(msg_id, None)

        for r in range(self.tbl_rx.rowCount()):
            id_item = self.tbl_rx.item(r, 0)
            if not id_item or id_item.text() != msg_id:
                continue

            state_item = self.tbl_rx.item(r, 2)
            if state_item is None:
                break

            if raw is not None:
                history = self._append_payload_history(id_item, raw)
            else:
                history = self._normalize_payload_history(id_item.data(Qt.UserRole))

            if freq:
                actual = self._calc_recent_frequency(history)
                state_item.setText(self._format_rate_state("수신 중", freq, actual))
                state_item.setForeground(QColor("blue"))

                if msg_id in self.receive_timers:
                    self.receive_timers[msg_id].stop()
                    self.receive_timers[msg_id].deleteLater()
                    del self.receive_timers[msg_id]

                timer = QTimer(self)
                timer.setSingleShot(True)
                timeout_ms = int(2000.0 / freq)
                timer.setInterval(timeout_ms)
                timer.timeout.connect(lambda mid=msg_id: self._receive_timeout(mid))
                timer.start()
                self.receive_timers[msg_id] = timer
            else:
                count = self.receive_counts.get(msg_id, 0) + 1
                self.receive_counts[msg_id] = count
                state_item.setText(f"수신 완료({count})")
                state_item.setForeground(QColor("blue"))
            break

        self._write_log(self.log_rx, "RECV", msg_id, raw)

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
        state_item = self.tbl_tx.item(row, 2)
        history: list[dict] = []
        msg_item = self.tbl_tx.item(row, 0)
        if msg_item is not None:
            if raw is not None:
                history = self._append_payload_history(msg_item, raw)
            else:
                history = self._normalize_payload_history(msg_item.data(Qt.UserRole))

        freq = self.periodic_config.get(msg_id, None)
        if freq and state_item is not None:
            actual = self._calc_recent_frequency(history)
            state_item.setText(self._format_rate_state("주기송신", freq, actual))
            state_item.setForeground(QColor("blue"))
        elif state_item is not None:
            state_item.setText("발신 완료")
            state_item.setForeground(QColor("blue"))

        self._write_log(self.log_tx, "SEND", msg_id, raw)

    # ──────────── 주기 전송 관리 ─────────────────────
    def _start_periodic_send(self, msg_id: str, row: int, freq_hz: float):
        interval_ms = int(1000.0 / freq_hz)
        timer = QTimer(self)
        timer.setInterval(interval_ms)
        timer.timeout.connect(lambda: self._periodic_timeout(msg_id, row))
        timer.start()

        self.periodic_timers[msg_id] = timer
        state_item = self.tbl_tx.item(row, 2)
        if state_item is not None:
            state_item.setText(self._format_rate_state("주기송신", freq_hz, None))
            state_item.setForeground(QColor("blue"))

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
            self._append_payload_history(self.tbl_tx.item(row, 0), raw)
        self._write_log(self.log_tx, "SEND", msg_id, raw)

    # ──────────── 내부 유틸 ───────────────────────
    def _update_state(self, tbl: QTableWidget, msg_id: str, state: str):
        for r in range(tbl.rowCount()):
            if tbl.item(r, 0).text() == msg_id:
                item = tbl.item(r, 2)
                if item is None:
                    break
                item.setText(state)
                try:
                    if "완료" in state:
                        item.setForeground(QColor("blue"))
                except Exception:
                    pass
                break

    def _truncate_payload_text(self, text: str) -> str:
        max_chars = getattr(self, "LOG_PAYLOAD_MAX_CHARS", None)
        if max_chars is None:
            return text
        try:
            max_chars = int(max_chars)
        except Exception:
            return text
        if max_chars <= 0:
            return ""
        if len(text) <= max_chars:
            return text
        suffix = getattr(self, "LOG_PAYLOAD_TRUNC_SUFFIX", " ...(truncated)")
        return text[:max_chars] + suffix

    def _append_log_entries_batch(self, log_w: QTextEdit, entries: Sequence[str]) -> None:
        limit = getattr(self, "LOG_ENTRY_LIMIT", None)
        if limit is None:
            for line in entries:
                log_w.append(line)
            return
        try:
            limit = int(limit)
        except Exception:
            for line in entries:
                log_w.append(line)
            return
        if limit <= 0:
            return
        buf = getattr(log_w, "_entry_buffer", None)
        if buf is None or getattr(log_w, "_entry_buffer_limit", None) != limit:
            buf = deque(maxlen=limit)
            setattr(log_w, "_entry_buffer", buf)
            setattr(log_w, "_entry_buffer_limit", limit)
        for line in entries:
            buf.append(line)
        log_w.setPlainText("\n".join(buf))
        try:
            log_w.moveCursor(QTextCursor.End)
        except Exception:
            pass

    def _flush_log_queue(self, log_w: QTextEdit) -> None:
        pending = getattr(log_w, "_pending_entries", None)
        if not pending:
            return
        entries = list(pending)
        pending.clear()
        self._append_log_entries_batch(log_w, entries)

    def _append_log_entry(self, log_w: QTextEdit, line: str) -> None:
        flush_ms = getattr(self, "LOG_FLUSH_INTERVAL_MS", None)
        if flush_ms is None:
            self._append_log_entries_batch(log_w, [line])
            return
        try:
            flush_ms = int(flush_ms)
        except Exception:
            flush_ms = 0
        if flush_ms <= 0:
            self._append_log_entries_batch(log_w, [line])
            return
        pending = getattr(log_w, "_pending_entries", None)
        if pending is None:
            pending = []
            setattr(log_w, "_pending_entries", pending)
        pending.append(line)
        timer = getattr(log_w, "_flush_timer", None)
        if timer is None:
            timer = QTimer(log_w)
            timer.setSingleShot(True)
            timer.timeout.connect(lambda w=log_w: self._flush_log_queue(w))
            setattr(log_w, "_flush_timer", timer)
        if not timer.isActive():
            timer.start(flush_ms)

    def _decode_payload_for_log(self, raw: bytes, msg_id: str) -> str:
        max_chars = getattr(self, "LOG_PAYLOAD_MAX_CHARS", None)
        max_chars_int = None
        if max_chars is not None:
            try:
                max_chars_int = int(max_chars)
            except Exception:
                max_chars_int = None

        raw_for_decode = raw
        truncated_raw = False
        msg_id_norm = str(msg_id).strip()
        if max_chars_int is not None and max_chars_int > 0 and msg_id_norm != "0102":
            max_bytes = max_chars_int * 4
            if max_bytes > 0 and len(raw) > max_bytes:
                raw_for_decode = raw[:max_bytes]
                truncated_raw = True

        decoded = raw_for_decode.decode(errors="ignore")
        if msg_id_norm == "0102":
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

        decoded = self._truncate_payload_text(decoded)
        if truncated_raw:
            suffix = getattr(self, "LOG_PAYLOAD_TRUNC_SUFFIX", " ...(truncated)")
            if decoded and not decoded.endswith(suffix):
                decoded += suffix
            elif not decoded:
                decoded = suffix
        return decoded

    def _write_log(self,
                log_w: QTextEdit,
                tag: str,
                msg_id: str,
                raw: bytes | None):
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"[{ts}] {tag:<4} : {msg_id}"

        if raw:
            try:
                decoded = self._decode_payload_for_log(raw, str(msg_id))
                if decoded:
                    line += f"\n{decoded}"
            except Exception:
                line += " (binary)"

        self._append_log_entry(log_w, line)

    # ────────────────────────────────────────────────────────────────
    # ────────────────────────────────────────────────────────────────
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
    # ────────────────────────────────────────────────────────────────
