from __future__ import annotations

import dataclasses
import json
from collections import deque
from datetime import datetime, timezone, timedelta
from typing import Any, Deque, Dict, List, Optional
import time

from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QComboBox,
    QFormLayout,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QPushButton,
    QMessageBox,
)
from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt

from modules.monitoring_ver2.config import PUSH_MESSAGES, RECEIVE_MESSAGES, SYSTEM_MODE_OPTIONS
from push.push_center import push_message
from data.message_models import (
    ModuleStatusModelModel,
    MissionProgressBodyModel,
    MissionEndRequestBodyModel,
    CollaborativeMissionCompleteModel,
    ReplanRequestBodyModel,
    IndividualMissionIDModel,
    IndividualMissionProgressStatusModel,
    ReplanRequestTimeStampModel,
    InputMissionIDModel,
    IndividualMissionIDListModel,
    PriorMissionListModel,
    OptionListModel,
)


def to_dict(obj):
    """Recursively convert dataclass instances to dictionaries."""
    if dataclasses.is_dataclass(obj):
        return {f.name: to_dict(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
    if isinstance(obj, (list, tuple)):
        return [to_dict(v) for v in obj]
    if isinstance(obj, dict):
        return {k: to_dict(v) for k, v in obj.items()}
    return obj


class MonitoringCSCTab(QWidget):
    """Monitoring CSC view with transmit/receive tables and history."""

    MAX_HISTORY = 50

    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager
        self.receive_storage = manager.receive_store
        self.push_storage = manager.push_store
        self.mode_combo = None

        self.tx_row_map: Dict[str, int] = {}
        self.rx_row_map: Dict[str, int] = {}
        self.tx_history: Dict[str, Deque[dict]] = {msg_id: deque(maxlen=self.MAX_HISTORY) for msg_id, _ in PUSH_MESSAGES}
        self.rx_history: Dict[str, Deque[dict]] = {msg_id: deque(maxlen=self.MAX_HISTORY) for msg_id, _ in RECEIVE_MESSAGES}
        self._periodic_targets: Dict[str, float] = {"0102": 5.0, "0501": 1.0}
        self._prepared_replan_body: Any | None = None
        self._prepared_replan_context: Dict[str, Any] | None = None
        self._last_replan_error: Exception | None = None

        self._init_ui()
        self._update_mode_selection(self.manager.get_logic_result("SystemMode"))

    # ------------------------------------------------------------------ UI
    def _init_ui(self):
        self.tbl_tx = self._make_tx_table()
        self.tbl_rx = self._make_rx_table()

        self._populate_tx_table()
        self._populate_rx_table()

        self.log_tx = None
        self.log_rx = None

        self.mode_combo = QComboBox()
        for value, label in SYSTEM_MODE_OPTIONS:
            self.mode_combo.addItem(label, value)
        self.mode_combo.currentIndexChanged.connect(self._handle_mode_combo_changed)

        mode_group = QGroupBox("시스템 운용 모드")
        mode_form = QFormLayout()
        mode_form.addRow(QLabel("현재 모드:"), self.mode_combo)
        mode_group.setLayout(mode_form)

        top_layout = QHBoxLayout()
        top_layout.addWidget(self._wrap_in_group("송신 메시지 이력", self.tbl_tx))
        top_layout.addWidget(self._wrap_in_group("수신 메시지 이력", self.tbl_rx))

        root = QVBoxLayout(self)
        root.addWidget(mode_group, 0)
        root.addLayout(top_layout, 1)
        root.setContentsMargins(8, 6, 8, 6)
        self.setLayout(root)

    def _handle_mode_combo_changed(self, index: int) -> None:
        if self.mode_combo is None:
            return
        mode_value = self.mode_combo.itemData(index)
        if mode_value is None:
            return
        try:
            mode_int = int(mode_value)
        except (TypeError, ValueError):
            return
        self.manager.set_system_mode(mode_int)

    def _update_mode_selection(self, mode_value) -> None:
        if self.mode_combo is None:
            return
        try:
            mode_int = int(mode_value)
        except (TypeError, ValueError):
            return
        index = self.mode_combo.findData(mode_int)
        if index >= 0:
            self.mode_combo.blockSignals(True)
            self.mode_combo.setCurrentIndex(index)
            self.mode_combo.blockSignals(False)

    def _wrap_in_group(self, title: str, widget: QWidget) -> QGroupBox:
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        layout.addWidget(widget)
        layout.setContentsMargins(6, 12, 6, 6)
        return box

    def _make_tx_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 4)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태", "송신"])
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSelectionBehavior(QTableWidget.SelectRows)
        header = tbl.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        return tbl

    def _make_rx_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 3)
        tbl.setHorizontalHeaderLabels(["Message ID", "Message Name", "상태"])
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setSelectionBehavior(QTableWidget.SelectRows)
        header = tbl.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        return tbl

    def _populate_tx_table(self):
        for row, (msg_id, msg_name) in enumerate(PUSH_MESSAGES):
            self.tbl_tx.insertRow(row)
            id_item = QTableWidgetItem(msg_id)
            name_item = QTableWidgetItem(msg_name)
            status_item = QTableWidgetItem("발신 전")
            status_item.setTextAlignment(Qt.AlignCenter)

            self.tbl_tx.setItem(row, 0, id_item)
            self.tbl_tx.setItem(row, 1, name_item)
            self.tbl_tx.setItem(row, 2, status_item)

            btn_send = QPushButton("발신")
            btn_send.clicked.connect(lambda _, r=row: self._handle_send_clicked(r))
            self.tbl_tx.setCellWidget(row, 3, btn_send)


            self.tx_row_map[msg_id] = row
            self.tx_history[msg_id] = deque(maxlen=self.MAX_HISTORY)

    def _populate_rx_table(self):
        for row, (msg_id, msg_name) in enumerate(RECEIVE_MESSAGES):
            self.tbl_rx.insertRow(row)
            id_item = QTableWidgetItem(msg_id)
            name_item = QTableWidgetItem(msg_name)
            status_item = QTableWidgetItem("수신 전")
            status_item.setTextAlignment(Qt.AlignCenter)

            self.tbl_rx.setItem(row, 0, id_item)
            self.tbl_rx.setItem(row, 1, name_item)
            self.tbl_rx.setItem(row, 2, status_item)


            self.rx_row_map[msg_id] = row
            self.rx_history[msg_id] = deque(maxlen=self.MAX_HISTORY)

    # ------------------------------------------------------------------ Refresh
    def refresh_display(self, update_info=None, data_object=None):
        if update_info is None:
            self._update_mode_selection(self.manager.get_logic_result("SystemMode"))
        else:
            source, key = update_info
            if source == "send":
                raw_bytes = data_object if isinstance(data_object, (bytes, bytearray)) else None
                self.mark_sent(key, raw_bytes)
                return
            if (source == "logic" and key == "SystemMode") or (source == "receive" and key == "0101"):
                if source == "receive" and key == "0101" and data_object is not None and hasattr(data_object, "systemMode"):
                    self._update_mode_selection(getattr(data_object, "systemMode"))
                else:
                    self._update_mode_selection(self.manager.get_logic_result("SystemMode"))

        self._update_tx_table()
        self._update_rx_table()

    def _update_tx_table(self):
        for msg_id in self.tx_row_map.keys():
            history_objs = self.push_storage.get_history(msg_id) or []
            dq = self.tx_history.setdefault(msg_id, deque(maxlen=self.MAX_HISTORY))
            dq.clear()
            for obj in history_objs[: self.MAX_HISTORY]:
                dq.append(self._serialize_obj(obj))
            self._update_tx_row_state(msg_id)

    def _update_tx_row_state(self, msg_id: str) -> None:
        row = self.tx_row_map.get(msg_id)
        if row is None:
            return
        dq = self.tx_history.get(msg_id) or deque(maxlen=self.MAX_HISTORY)
        self.tx_history[msg_id] = dq
        if dq:
            latest = dq[0]
            self.tbl_tx.item(row, 0).setData(Qt.UserRole, list(dq))
            self._append_tx_log(msg_id, latest, replace=True)
            planned = self._periodic_targets.get(msg_id)
            actual = self._calc_recent_frequency(dq)
            if planned:
                status_text = self._format_rate_state("주기송신", planned, actual)
                status_item = self.tbl_tx.item(row, 2)
                if status_item:
                    status_item.setForeground(QColor("blue"))
                    status_item.setText(status_text)
            else:
                status_text = "발신 완료"
                status_item = self.tbl_tx.item(row, 2)
                if status_item:
                    status_item.setForeground(QColor("blue"))
                    status_item.setText(status_text)
        else:
            status_item = self.tbl_tx.item(row, 2)
            if status_item:
                status_item.setForeground(QColor("black"))
                status_item.setText("송신 대기")
            cell = self.tbl_tx.item(row, 0)
            if cell:
                cell.setData(Qt.UserRole, [])

    def _calc_recent_frequency(self, history: Deque[dict], window_sec: float = 10.0) -> float | None:
        timestamps: List[float] = []
        for entry in history:
            ts = entry.get("timestamp") or entry.get("ms")
            if ts is None:
                continue
            try:
                val = float(ts)
            except (TypeError, ValueError):
                continue
            if val > 1e10:
                val /= 1000.0
            timestamps.append(val)
        if len(timestamps) < 2:
            return None
        timestamps.sort()
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
        parts: List[str] = []
        if planned is not None:
            parts.append(f"{planned:g}Hz")
        if actual is not None:
            parts.append(f"{actual:.2f}Hz")
        if parts:
            inner = " / ".join(parts)
            return f"{label}({inner})"
        return label

    def mark_sent(self, msg_id: str, raw: bytes | None = None) -> None:
        if msg_id not in self.tx_row_map:
            return
        if raw:
            try:
                payload_dict = json.loads(raw.decode("utf-8"))
                self._append_tx_log(msg_id, payload_dict, replace=True)
            except Exception:
                pass
        self._update_tx_table()

    def _update_rx_table(self):
        all_data = self.receive_storage.get_all_data()
        for msg_id, row in self.rx_row_map.items():
            data_obj = all_data.get(msg_id)
            dq = self.rx_history[msg_id]
            item = self.tbl_rx.item(row, 2)
            if data_obj:
                entry = self._serialize_obj(data_obj)
                if not dq or dq[0] != entry:
                    dq.appendleft(entry)
                    self._append_rx_log(msg_id, entry)
                status = self._format_status("수신 완료", entry.get("timestamp"))
                if item:
                    item.setText(status)
                    item.setForeground(QColor("blue"))
            else:
                if item:
                    item.setText("수신 전")
                    item.setForeground(QColor("black"))

    def set_replan_context(self, context: Dict[str, Any] | None, body: Any | None = None) -> None:
        """Store the prepared 0902 context so it can be dispatched on demand."""
        if context is not None:
            self._prepared_replan_context = dict(context)
        else:
            self._prepared_replan_context = None
        self._prepared_replan_body = body

        row = self.tx_row_map.get("0902")
        if row is None:
            return
        status_item = self.tbl_tx.item(row, 2)
        if status_item is None:
            return
        if context:
            status_item.setText("재계획 요청 준비됨")
            status_item.setForeground(QColor("darkgreen"))
        else:
            status_item.setText("송신 대기")
            status_item.setForeground(QColor("black"))

    def dispatch_replan_request(self, body: Any | None = None, context: Dict[str, Any] | None = None) -> bool:
        """Send a prepared 0902 request through the push pipeline."""
        payload = body if body is not None else self._prepared_replan_body
        if payload is None:
            self._last_replan_error = ValueError("No 0902 payload is prepared")
            return False
        if context is None:
            context = self._prepared_replan_context
        if context is not None:
            try:
                self.set_replan_context(context, body=payload)
            except Exception:
                pass

        if dataclasses.is_dataclass(payload):
            body_dict = to_dict(payload)
            store_obj = payload
        elif isinstance(payload, dict):
            body_dict = dict(payload)
            store_obj = body_dict
        else:
            body_dict = to_dict(payload)
            store_obj = body_dict

        try:
            push_message("0902", self.manager.node_messenger, body_dict=body_dict)
        except Exception as exc:
            self._last_replan_error = exc
            try:
                self.manager._log("MON_CSC", "ERROR", f"0902 push failed: {exc}")
            except Exception:
                pass
            return False

        try:
            raw_bytes = json.dumps(body_dict, ensure_ascii=False).encode("utf-8")
        except Exception:
            raw_bytes = None

        try:
            if raw_bytes and self.manager.gui_update_callback:
                self.manager.gui_update_callback("send", "0902", raw_bytes)
        except Exception:
            pass

        try:
            self.push_storage.add_data("0902", store_obj)
        except Exception:
            pass

        self._append_tx_log("0902", body_dict)
        self.set_replan_context(None)
        self.refresh_display()
        self._last_replan_error = None
        return True


    # ------------------------------------------------------------------ Button handlers
    def _handle_send_clicked(self, row: int):
        item = self.tbl_tx.item(row, 0)
        if not item:
            return
        msg_id = item.text()
        if msg_id == "0902" and self._prepared_replan_body is not None:
            payload = self._prepared_replan_body
            context = self._prepared_replan_context
            if not self.dispatch_replan_request(payload, context=context):
                err = self._last_replan_error or Exception("0902 dispatch failed")
                QMessageBox.critical(
                    self,
                    "0902 발신 실패",
                    f"0902 재계획 요청을 발신하지 못했습니다.\n{err}",
                )
            return
        try:
            body_obj = self._create_dummy_body(msg_id)
        except NotImplementedError:
            QMessageBox.warning(self, "지원하지 않는 메시지", f"{msg_id} 더미 본문이 정의되어 있지 않습니다.")
            return
        except Exception as exc:
            QMessageBox.critical(self, "본문 생성 오류", f"{msg_id} 본문 생성 중 오류가 발생했습니다.\n{exc}")
            return

        if dataclasses.is_dataclass(body_obj):
            body_dict = to_dict(body_obj)
        elif isinstance(body_obj, dict):
            body_dict = body_obj
        else:
            QMessageBox.critical(self, "본문 오류", f"{msg_id} 본문을 dict로 변환할 수 없습니다.")
            return

        try:
            push_message(msg_id, self.manager.node_messenger, body_dict=body_dict)
        except Exception as exc:
            QMessageBox.critical(self, "발신 오류", f"{msg_id} 발신 중 오류가 발생했습니다.\n{exc}")
            return

        # push_storage에 수동 발신도 기록
        try:
            self.push_storage.add_data(msg_id, body_obj)
        except Exception:
            pass

        self._append_tx_log(msg_id, body_dict)
        self.refresh_display()

    def _append_tx_log(self, msg_id: str, body: dict, replace: bool = False):
        if self.log_tx is None:
            return
        payload = json.dumps(body, ensure_ascii=False, indent=2)
        text = f"[{msg_id}] {payload}"
        if replace:
            self.log_tx.setPlainText(text)
        else:
            self.log_tx.append(text)

    def _append_rx_log(self, msg_id: str, body: dict, replace: bool = False):
        if self.log_rx is None:
            return
        payload = json.dumps(body, ensure_ascii=False, indent=2)
        text = f"[{msg_id}] {payload}"
        if replace:
            self.log_rx.setPlainText(text)
        else:
            self.log_rx.append(text)

    def _format_status(self, prefix: str, timestamp) -> str:
        ts = self._format_time(timestamp)
        return f"{prefix} ({ts})" if ts else prefix

    def _format_time(self, timestamp) -> str | None:
        if timestamp in (None, "", 0):
            return None
        try:
            ts = int(timestamp)
        except Exception:
            return None
        try:
            if ts < 946_684_800_000:  # 값이 2000년 기준(ms)이라면
                base = datetime(2000, 1, 1, tzinfo=timezone.utc)
                dt = base + timedelta(milliseconds=ts)
            else:
                dt = datetime.fromtimestamp(ts / 1000, tz=timezone.utc)
            return dt.astimezone().strftime("%Y-%m-%d %H:%M:%S")
        except Exception:
            return None

    def _serialize_obj(self, obj):
        if dataclasses.is_dataclass(obj):
            return to_dict(obj)
        if isinstance(obj, dict):
            return {k: to_dict(v) for k, v in obj.items()}
        return {"value": str(obj)}
    def _create_dummy_body(self, msg_id: str):
        timestamp = int(datetime.now(timezone.utc).timestamp() * 1000)
        source_module = "MSM"
        import random

        if msg_id == "0102":
            return ModuleStatusModelModel(
                timestamp=timestamp,
                source=source_module,
                status=random.randint(0, 2),
            )
        if msg_id == "0501":
            return MissionProgressBodyModel(
                timestamp=timestamp,
                source=source_module,
                currentMissionPlanID=random.randint(100, 999),
                currentInputMissionID=random.randint(1000, 9999),
                individualMissionProgressStatusList=[
                    IndividualMissionProgressStatusModel(
                        aircraftID=1,
                        currentIndividualMission=IndividualMissionIDModel(individualMissionID=101),
                        currentIndividualMissionProgress=random.randint(0, 100),
                    ),
                    IndividualMissionProgressStatusModel(
                        aircraftID=2,
                        currentIndividualMission=IndividualMissionIDModel(individualMissionID=102),
                        currentIndividualMissionProgress=random.randint(0, 100),
                    ),
                    IndividualMissionProgressStatusModel(
                        aircraftID=3,
                        currentIndividualMission=IndividualMissionIDModel(individualMissionID=103),
                        currentIndividualMissionProgress=random.randint(0, 100),
                    ),
                ],
            )
        if msg_id == "0502":
            return MissionEndRequestBodyModel(
                timestamp=timestamp,
                source=source_module,
                reason=random.randint(0, 3),
            )
        if msg_id == "0503":
            return CollaborativeMissionCompleteModel(
                timestamp=timestamp,
                source=source_module,
                systemRecommend=random.randint(0, 1),
            )
        if msg_id == "0902":
            return ReplanRequestBodyModel(
                timestamp=timestamp,
                source=source_module,
                replanRequestTime=ReplanRequestTimeStampModel(replanRequestTimestamp=timestamp),
                replanLevel=random.randint(1, 3),
                inputMissionIDList=[
                    InputMissionIDModel(inputMissionID=random.randint(1, 10))
                ],
                IndividualMissionIDList=[
                    IndividualMissionIDListModel(individualMissionID=random.randint(101, 110))
                ],
                priorMissionList=[
                    PriorMissionListModel(priorMissionID=random.randint(201, 210))
                ],
                replanRequest=f"Dummy replan request {random.randint(1, 100)}",
                optionList=[
                    OptionListModel(
                        optionID=random.randint(1, 5),
                        optionName=f"Option{random.randint(1, 5)}",
                        missionPlanID=random.randint(1, 10),
                    )
                ],
            )
        raise NotImplementedError(f"지원하지 않는 메시지: {msg_id}")
