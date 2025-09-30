from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QTableWidgetItem,
    QTableWidget,
    QHBoxLayout,
    QHeaderView,
    QTextEdit,
    QMessageBox,
)
from PyQt5.QtCore import Qt
from datetime import datetime, timezone
import json, dataclasses


from config import PUSH_MESSAGES, RECEIVE_MESSAGES
from modules.common.push_center import push_message
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

# data storages are accessed via the manager


def to_dict(obj):
    """Recursively convert a dataclass object to a dictionary."""
    if dataclasses.is_dataclass(obj):
        result = []
        for f in dataclasses.fields(obj):
            value = to_dict(getattr(obj, f.name))
            result.append((f.name, value))
        return dict(result)
    elif isinstance(obj, tuple) and hasattr(obj, "_fields"):
        return type(obj)._make(to_dict(o) for o in obj)
    elif isinstance(obj, (list, tuple)):
        return type(obj)(to_dict(v) for v in obj)
    elif isinstance(obj, dict):
        return type(obj)((to_dict(k), to_dict(v)) for k, v in obj.items())
    else:
        return obj


class DummyTab(QWidget):
    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager
        self.receive_storage = self.manager.receive_store
        self.push_storage = self.manager.push_store

        self.rx_row_map = {}
        self.tx_row_map = {}

        self._init_ui()

    def _init_ui(self):
        # Tables
        self.tbl_tx = self._make_tx_table()
        self.tbl_rx = self._make_rx_table()

        self._populate_table(self.tbl_tx, PUSH_MESSAGES, self.tx_row_map)
        self._populate_table(self.tbl_rx, RECEIVE_MESSAGES, self.rx_row_map)

        # Connect signals
        self.tbl_tx.cellDoubleClicked.connect(self._on_tx_table_double_clicked)
        self.tbl_rx.cellClicked.connect(self._on_rx_table_clicked)

        # Log viewers
        self.log_tx = self._make_log_viewer(
            "Double-click a message to send dummy data."
        )
        self.log_rx = self._make_log_viewer("Click a message to view its data.")

        # Layout
        left_panel = self._create_panel("발신 (Sent)", self.tbl_tx, self.log_tx)
        right_panel = self._create_panel("수신 (Received)", self.tbl_rx, self.log_rx)

        main_layout = QHBoxLayout(self)
        main_layout.addWidget(left_panel)
        main_layout.addWidget(right_panel)
        self.setLayout(main_layout)

    def _populate_table(self, table, messages, row_map):
        for msg_id, msg_name in messages:
            row_position = table.rowCount()
            table.insertRow(row_position)

            id_item = QTableWidgetItem(msg_id)
            id_item.setData(Qt.UserRole, None)  # Initially no data

            table.setItem(row_position, 0, id_item)
            table.setItem(row_position, 1, QTableWidgetItem(msg_name))
            table.setItem(row_position, 2, QTableWidgetItem("대기"))  # Status

            row_map[msg_id] = row_position

    def _make_tx_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 4)
        tbl.setHorizontalHeaderLabels(["Message ID", "Name", "Status", "Last Sent"])
        self._configure_table(tbl)
        return tbl

    def _make_rx_table(self) -> QTableWidget:
        tbl = QTableWidget(0, 3)
        tbl.setHorizontalHeaderLabels(["Message ID", "Name", "Last Received"])
        self._configure_table(tbl)
        return tbl

    def _configure_table(self, tbl: QTableWidget):
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.horizontalHeader().setSectionResizeMode(1, QHeaderView.Interactive)
        tbl.verticalHeader().setVisible(False)
        tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        tbl.setStyleSheet("font-size:12px;")

    def _make_log_viewer(self, placeholder: str) -> QTextEdit:
        log = QTextEdit()
        log.setReadOnly(True)
        log.setStyleSheet(
            "border:1px solid #345; font-family:Consolas; font-size:11px;"
        )
        log.setPlaceholderText(placeholder)
        return log

    def _create_panel(self, title: str, table: QTableWidget, log: QTextEdit) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        label = QLabel(f"<b>{title}</b>")
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("font-size:14px;")
        layout.addWidget(label)
        layout.addWidget(table, 1)
        layout.addWidget(log, 1)
        layout.setContentsMargins(0, 0, 0, 0)
        return widget

    def refresh_display(self, update_info=None, data_object=None):
        self._update_rx_table()
        self._update_tx_table()

    def _update_rx_table(self):
        all_data = self.receive_storage.get_all_data()
        for msg_id, data_obj in all_data.items():
            if msg_id in self.rx_row_map:
                row = self.rx_row_map[msg_id]

                timestamp = getattr(data_obj, "timestamp", None)
                time_str = (
                    datetime.fromtimestamp(timestamp / 1000).strftime("%H:%M:%S")
                    if timestamp
                    else "N/A"
                )

                self.tbl_rx.item(row, 0).setData(Qt.UserRole, data_obj)
                self.tbl_rx.setItem(row, 2, QTableWidgetItem(time_str))

    def _update_tx_table(self):
        all_data = self.push_storage.get_all_data()
        for msg_id, history in all_data.items():
            if not history:
                continue

            if msg_id in self.tx_row_map:
                row = self.tx_row_map[msg_id]
                latest_obj = history[0]
                timestamp = getattr(latest_obj, "timestamp", None)
                time_str = (
                    datetime.fromtimestamp(timestamp / 1000).strftime("%H:%M:%S")
                    if timestamp
                    else "N/A"
                )
                count_str = f"{len(history)} 회"

                self.tbl_tx.item(row, 0).setData(Qt.UserRole, history)
                self.tbl_tx.setItem(row, 2, QTableWidgetItem(count_str))
                self.tbl_tx.setItem(row, 3, QTableWidgetItem(time_str))

    def _on_rx_table_clicked(self, row, column):
        item = self.tbl_rx.item(row, 0)
        if not item:
            return
        data_obj = item.data(Qt.UserRole)
        self._display_data(data_obj, self.log_rx)

    def _on_tx_table_double_clicked(self, row, column):
        item = self.tbl_tx.item(row, 0)
        if not item:
            return

        msg_id = item.text()
        msg_name = self.tbl_tx.item(row, 1).text()

        reply = QMessageBox.question(
            self,
            "Confirm Send",
            f"""더미 데이터를 발신 하겠습니까?\n\nID: {msg_id} ({msg_name})""",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply == QMessageBox.Yes:
            self._send_dummy_message(msg_id)

    def _create_dummy_body(self, msg_id: str):
        """Creates a dummy data model object for a given message ID."""
        timestamp = int(datetime.now(timezone.utc).timestamp() * 1000)
        source_module = "MonitoringDummyTab"
        import random

        if msg_id == "0102":  # 모듈 상태 정보
            return ModuleStatusModelModel(
                timestamp=timestamp,
                source=source_module,
                status=random.randint(0, 2),  # 0: Normal, 1: Warning, 2: Error
            )
        elif msg_id == "0501":  # 임무수행상태정보
            return MissionProgressBodyModel(
                timestamp=timestamp,
                source=source_module,
                currentMissionPlanID=random.randint(100, 999),
                currentInputMissionID=random.randint(1000, 9999),
                individualMissionProgressStatusList=[
                    IndividualMissionProgressStatusModel(
                        aircraftID=1,
                        currentIndividualMission=IndividualMissionIDModel(
                            individualMissionID=101
                        ),
                        currentIndividualMissionProgress=random.randint(0, 100),
                    ),
                    IndividualMissionProgressStatusModel(
                        aircraftID=2,
                        currentIndividualMission=IndividualMissionIDModel(
                            individualMissionID=102
                        ),
                        currentIndividualMissionProgress=random.randint(0, 100),
                    ),
                    IndividualMissionProgressStatusModel(
                        aircraftID=3,
                        currentIndividualMission=IndividualMissionIDModel(
                            individualMissionID=103
                        ),
                        currentIndividualMissionProgress=random.randint(0, 100),
                    ),
                ],
            )
        elif msg_id == "0502":  # 임무종료 요청
            return MissionEndRequestBodyModel(
                timestamp=timestamp,
                source=source_module,
                reason=random.randint(
                    0, 3
                ),  # 0: Operator, 1: System, 2: Emergency, 3: Complete
            )
        elif msg_id == "0503":  # 협업기저임무 완료 알림
            return CollaborativeMissionCompleteModel(
                timestamp=timestamp,
                source=source_module,
                systemRecommend=random.randint(0, 1),  # 0: No, 1: Yes
            )
        elif msg_id == "0902":  # 재계획 요청
            return ReplanRequestBodyModel(
                timestamp=timestamp,
                source=source_module,
                replanRequestTime=ReplanRequestTimeStampModel(
                    replanRequestTimestamp=timestamp
                ),
                replanLevel=random.randint(0, 2),
                inputMissionIDList=[
                    InputMissionIDModel(inputMissionID=random.randint(1, 10))
                ],
                IndividualMissionIDList=[
                    IndividualMissionIDListModel(
                        individualMissionID=random.randint(101, 110)
                    )
                ],
                priorMissionList=[
                    PriorMissionListModel(priorMissionID=random.randint(201, 210))
                ],
                replanRequest="Dummy Replan Request " + str(random.randint(1, 100)),
                optionList=[
                    OptionListModel(
                        optionID=random.randint(1, 5),
                        optionName="Option" + str(random.randint(1, 5)),
                        missionPlanID=random.randint(1, 10),
                    )
                ],
            )

        raise NotImplementedError(f"Dummy body creation not implemented for {msg_id}")

    def _send_dummy_message(self, msg_id: str):
        try:
            body_obj = self._create_dummy_body(msg_id)

            # Display the sent data in the log viewer
            self._display_data(body_obj, self.log_tx)

            if dataclasses.is_dataclass(body_obj):
                body_dict = to_dict(body_obj)
                push_message(msg_id, self.manager.node_messenger, body_dict=body_dict)
            else:  # In case it's a regular dict
                push_message(msg_id, self.manager.node_messenger, body_dict=body_obj)

        except Exception as e:
            error_dialog = QMessageBox()
            error_dialog.setIcon(QMessageBox.Critical)
            error_dialog.setText(f"Failed to send message {msg_id}")
            error_dialog.setInformativeText(str(e))
            error_dialog.setWindowTitle("Send Error")
            error_dialog.exec_()

    def _display_data(self, data, text_widget):
        if not data:
            text_widget.clear()
            return
        try:
            data_dict = to_dict(data)
            pretty_json = json.dumps(
                data_dict, indent=4, ensure_ascii=False, default=str
            )
            text_widget.setText(pretty_json)
        except Exception as e:
            text_widget.setText(
                f"""Error converting data to text: {e}

Data: {data}"""
            )
