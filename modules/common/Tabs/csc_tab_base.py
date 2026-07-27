# /mnt/data/csc_tab_base.py
from __future__ import annotations
from datetime import datetime
from collections import deque
import copy
import importlib
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
from modules.common import db_paths
from modules.common.gui_style import polish_message_table
from modules.common.message_payload_dialog import MessagePayloadDialog, load_message_field_specs
from modules.common.option_codes import DEFAULT_OPTION_CODE_SEQUENCE
from modules.common.source_utils import override_source_fields
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


_COMMON_DIALOG_DB_RULES = {
    "0201": ("InputMissionPlan", "inputMissionPackageID", None),
    "0204": ("InputMissionPlan", "inputMissionPackageID", None),
    "0203": ("MissionReferenceInfo", "missionReferencePackageID", None),
    "0301": ("MissionPlan", "missionPlanID", None),
    "0302": ("IndividualMissionPlan", "individualMissionPackageID", None),
    "0303": ("FlightPath", "pathID", None),
    "0304": ("FlightPath", "pathID", "123"),
    "0903": ("MissionPlan", "missionPlanID", None),
}

class CSCTabBase(QWidget):
    """Common CSC tab base."""

    # 서브클래스에서 오버라이드할 상수 -----------------
    HISTORY_LIMIT: int = 50
    HISTORY_SEPARATOR: str = "=" * 64
    LOG_ENTRY_LIMIT: int | None = None
    LOG_PAYLOAD_MAX_CHARS: int | None = None
    LOG_PAYLOAD_TRUNC_SUFFIX: str = " ...(truncated)"
    LOG_FLUSH_INTERVAL_MS: int | None = None
    ENABLE_LOG_PANES: bool = False

    TITLE: str = "CSC"
    PUSH_MESSAGES: Sequence[Tuple[str, str]] = ()
    RECEIVE_MESSAGES: Sequence[Tuple[str, str]] = ()
    # -------------------------------------------------

    def __init__(self, *, messenger, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.messenger = messenger
        self._push_message_names = {
            str(msg_id).zfill(4): name for msg_id, name in getattr(self, "PUSH_MESSAGES", [])
        }

        self.periodic_config: Dict[str, Optional[float]] = {
            '0000': None,  
            '0001': None,
            '0101': None,
            '0102': 5,
            '0103': 5,
            '0104': None,
            '0201': None,
            '0202': None,
            '0203': None,
            '0204': None,
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
        self._periodic_payload_templates: Dict[str, dict] = {}
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

        left = self._side("발신 데이터 목록", self.tbl_tx, self.log_tx)
        right = self._side("수신 데이터 목록", self.tbl_rx, self.log_rx)
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(14)
        body.addWidget(left)
        body.addWidget(right)

        root = QVBoxLayout(self)
        title = QLabel(self.TITLE)
        title.setStyleSheet("font-size:18px; font-weight:700; color:#0f172a; padding:2px 0 6px 0;")
        self._title_label = title
        root.addWidget(title)
        root.addLayout(body)
        root.setContentsMargins(12, 10, 12, 12)
        root.setSpacing(12)

    def _make_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 3)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태"])
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        polish_message_table(tbl)
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
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        polish_message_table(tbl)
        return tbl

    def _make_log(self) -> QTextEdit | None:
        if not getattr(self, "ENABLE_LOG_PANES", False):
            return None
        log = QTextEdit()
        log.setReadOnly(True)
        log.setPlaceholderText("message log")
        return log

    def _side(self, caption: str, tbl: QTableWidget, log: QTextEdit | None) -> QWidget:
        w = QWidget()
        w.setObjectName("CscPanel")
        lay = QVBoxLayout(w)
        lbl = QLabel(caption)
        lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        lbl.setObjectName("TableTitle")
        lay.addWidget(lbl)
        lay.addWidget(tbl, 1)
        if log is not None:
            lay.addWidget(log, 0)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
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
            item_mid = QTableWidgetItem(mid)
            item_name = QTableWidgetItem(name)
            item_state = QTableWidgetItem(default_state)
            item_mid.setToolTip(mid)
            item_name.setToolTip(name)
            item_state.setToolTip(default_state)
            tbl.setItem(r, 0, item_mid)
            tbl.setItem(r, 1, item_name)
            tbl.setItem(r, 2, item_state)

            if is_tx_table:
                btn_send = QPushButton("발신")
                btn_send.setObjectName("SecondaryButton")
                btn_send.setMinimumHeight(32)
                btn_send.clicked.connect(lambda _, row=r: self._on_tx_button_clicked(row))
                tbl.setCellWidget(r, 3, btn_send)

                btn_view = QPushButton("보기")
                btn_view.setObjectName("SecondaryButton")
                btn_view.setMinimumHeight(32)
                btn_view.clicked.connect(lambda _, row=r: self._on_tx_view_button_clicked(row))
                tbl.setCellWidget(r, 4, btn_view)

            elif is_rx_table:
                btn_view = QPushButton("보기")
                btn_view.setObjectName("SecondaryButton")
                btn_view.setMinimumHeight(32)
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
        vbox.addWidget(txt)

        btns = QDialogButtonBox(QDialogButtonBox.Close)
        btns.rejected.connect(dlg.reject)
        vbox.addWidget(btns)

        dlg.resize(680, 420)
        dlg.exec_()

    def _on_tx_button_clicked(self, row: int):
        return self.send_tx_row(row, interactive=True)

    def _make_tx_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 5)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태", "발신", "데이터"])
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        polish_message_table(tbl)
        return tbl

    def append_log(self, text: str) -> None:
        if not getattr(self, "ENABLE_LOG_PANES", False):
            return
        self._write_log(self.log_rx, "INFO", "UI", str(text).encode("utf-8", "ignore"))

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

                timer = self.receive_timers.get(msg_id)
                if timer is None:
                    timer = QTimer(self)
                    timer.setSingleShot(True)
                    timer.timeout.connect(lambda mid=msg_id: self._receive_timeout(mid))
                    self.receive_timers[msg_id] = timer
                elif timer.isActive():
                    timer.stop()
                timeout_ms = int(2000.0 / freq)
                timer.setInterval(timeout_ms)
                timer.start()
            else:
                count = self.receive_counts.get(msg_id, 0) + 1
                self.receive_counts[msg_id] = count
                state_item.setText(f"수신 완료({count})")
                state_item.setForeground(QColor("blue"))
            break

        self._write_log(self.log_rx, "RECV", msg_id, raw)

    # ──────────── 더블클릭 → Push (주기/비주기) ────────────────
    def send_tx_row(self, row: int, *, interactive: bool = False) -> bool:
        return self._dispatch_tx(row, interactive=interactive)

    def _on_tx_double_clicked(self, row: int, _col: int):
        return self._on_tx_button_clicked(row)

    def _dispatch_tx(self, row: int, *, interactive: bool) -> bool:
        item = self.tbl_tx.item(row, 0)
        if item is None:
            return False

        msg_id = item.text()
        freq = self.periodic_config.get(msg_id, None)

        if freq is not None and msg_id in self.periodic_timers:
            self._stop_periodic_send(msg_id, row)
            return True

        body = self._clone_tx_body(self._build_overridden_body(msg_id))
        if self._should_confirm_tx(msg_id, interactive):
            body = self._confirm_tx_payload(msg_id, row, body, freq)
            if body is None:
                state_item = self.tbl_tx.item(row, 2)
                if state_item is not None:
                    state_item.setText("전송 취소")
                return False

        if freq is None:
            return self._push_tx_once(row, msg_id, body)

        self._start_periodic_send(msg_id, row, freq, body_override=body)
        return True

    def _push_tx_once(self, row: int, msg_id: str, body: dict | None) -> bool:
        ok = push_message(
            msg_id,
            self.messenger,
            on_done=lambda mid, raw: self._mark_single_sent(row, mid, raw),
            body_dict=body,
        )
        if not ok:
            state_item = self.tbl_tx.item(row, 2)
            if state_item is not None:
                state_item.setText("발신 실패")
            return False
        return True

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
    def _start_periodic_send(self, msg_id: str, row: int, freq_hz: float, body_override: dict | None = None):
        interval_ms = int(1000.0 / freq_hz)
        timer = QTimer(self)
        timer.setInterval(interval_ms)
        timer.timeout.connect(lambda: self._periodic_timeout(msg_id, row))
        timer.start()

        self.periodic_timers[msg_id] = timer
        if isinstance(body_override, dict):
            self._periodic_payload_templates[msg_id] = copy.deepcopy(body_override)
        else:
            self._periodic_payload_templates.pop(msg_id, None)
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
        self._periodic_payload_templates.pop(msg_id, None)

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

    def default_tx_payload(self, msg_id: str, body: dict | None = None) -> dict | None:
        mid = str(msg_id).strip().zfill(4)
        seed = self._clone_tx_body(body)
        if not isinstance(seed, dict) or not seed:
            seed = self._clone_tx_body(self._build_overridden_body(mid))
        if not isinstance(seed, dict) or not seed:
            seed = self._build_dialog_seed_payload(mid)
        if not isinstance(seed, dict):
            return seed
        return self._normalize_dialog_payload(mid, seed)

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
            self.default_tx_payload(mid, body) or {},
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

    def _build_dialog_seed_payload(self, msg_id: str) -> dict:
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
                "status": self._self_diag_status(),
            }

        if mid in _COMMON_DIALOG_DB_RULES:
            directory_name, field_name, prefix_chars = _COMMON_DIALOG_DB_RULES[mid]
            return {
                "timestamp": timestamp,
                "source": source,
                field_name: self._latest_numeric_id(directory_name, prefix_chars=prefix_chars),
            }

        if mid == "0305":
            return {
                "timestamp": timestamp,
                "source": source,
                "missionPlanningStatus": 1,
                "replanReason": "",
            }

        if mid == "0701":
            option_name = DEFAULT_OPTION_CODE_SEQUENCE[0] if DEFAULT_OPTION_CODE_SEQUENCE else "OPTION_A"
            return {
                "timestamp": timestamp,
                "source": source,
                "autoExecution": False,
                "optionList": [
                    {
                        "optionID": 1,
                        "optionName": option_name,
                        "missionPlanID": self._latest_numeric_id("MissionPlan"),
                        "survivalRate": 1,
                        "timeContraction": 1,
                        "recogEffectiveness": 1,
                        "distance": 15000,
                        "target": 0,
                    }
                ],
            }

        if mid == "0901":
            option_name = DEFAULT_OPTION_CODE_SEQUENCE[0] if DEFAULT_OPTION_CODE_SEQUENCE else "OPTION_A"
            return {
                "timestamp": timestamp,
                "source": source,
                "requestTime": timestamp,
                "pendingOptionList": [
                    {
                        "optionID": 1,
                        "optionName": option_name,
                        "missionPlanID": self._latest_numeric_id("MissionPlan"),
                    }
                ],
            }

        if mid == "0902":
            return {
                "timestamp": timestamp,
                "source": source,
                "replanRequestTime": {
                    "replanRequestTimestamp": timestamp,
                },
                "replanLevel": 1,
                "inputMissionIDList": [],
                "replanRequest": "",
                "optionList": [],
            }

        generated = self._generate_dialog_payload(mid)
        if isinstance(generated, dict) and generated:
            return generated

        return {
            "timestamp": timestamp,
            "source": source,
        }

    def _generate_dialog_payload(self, msg_id: str) -> dict | None:
        mid = str(msg_id).strip().zfill(4)
        try:
            module = importlib.import_module(f"modules.common.generator.message{mid}_generator")
            factory = getattr(module, f"make_msg{mid}_body", None)
            if not callable(factory):
                return None
        except Exception:
            return None

        try:
            payload = factory(source=self._sw_code())
        except TypeError:
            try:
                payload = factory(self._sw_code())
            except Exception:
                return None
        except Exception:
            return None

        if not isinstance(payload, dict):
            return None
        return override_source_fields(payload, self._sw_code())

    def _normalize_dialog_payload(self, msg_id: str, body: dict) -> dict:
        if not isinstance(body, dict):
            return body

        canonical_names = {
            spec.name.lower(): spec.name for spec in load_message_field_specs(str(msg_id).zfill(4))
        }
        normalized: dict = {}
        for key, value in body.items():
            name = str(key)
            normalized[canonical_names.get(name.lower(), name)] = value
        return normalized

    def _latest_numeric_id(self, directory_name: str, *, prefix_chars: str | None = None) -> int:
        try:
            directory = db_paths.ensure_db_payload(directory_name)
        except Exception:
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

    @staticmethod
    def _clone_tx_body(body: dict | None):
        if isinstance(body, dict):
            return copy.deepcopy(body)
        return body

    def _refresh_runtime_fields(self, body: dict | None) -> dict | None:
        if not isinstance(body, dict):
            return body
        refreshed = copy.deepcopy(body)
        for key in ("timestamp", "Timestamp"):
            if key in refreshed:
                refreshed[key] = _now_ms_since_2000()
        return refreshed

    def _periodic_timeout(self, msg_id: str, row: int):
        template = self._periodic_payload_templates.get(msg_id)
        body = self._refresh_runtime_fields(template)
        if body is None:
            body = self._refresh_runtime_fields(self._build_overridden_body(msg_id))
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

    def _append_log_entries_batch(self, log_w: QTextEdit | None, entries: Sequence[str]) -> None:
        if log_w is None:
            return
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

    def _flush_log_queue(self, log_w: QTextEdit | None) -> None:
        if log_w is None:
            return
        pending = getattr(log_w, "_pending_entries", None)
        if not pending:
            return
        entries = list(pending)
        pending.clear()
        self._append_log_entries_batch(log_w, entries)

    def _append_log_entry(self, log_w: QTextEdit | None, line: str) -> None:
        if log_w is None:
            return
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
                log_w: QTextEdit | None,
                tag: str,
                msg_id: str,
                raw: bytes | None):
        if log_w is None:
            return
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
