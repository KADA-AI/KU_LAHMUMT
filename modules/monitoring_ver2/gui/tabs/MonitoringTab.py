# gui/tabs/MonitoringTab.py: 메인 GUI의 '모니터링' 탭에 해당하는 UI와 데이터 표시 기능을 정의합니다.

import json
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QTextEdit,
    QComboBox,
    QGroupBox,
    QFormLayout,
    QHBoxLayout,
)
from PyQt5.QtGui import QColor  # Import QColor
from ..widgets.CircularProgressBar import CircularProgressBar


class MonitoringTab(QWidget):
    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager
        self.uav_aircraft_ids = [4, 5, 6]
        self.init_ui()
        # 초기값 설정
        self.refresh_display(("logic", "SystemMode"))

    def init_ui(self):
        layout = QVBoxLayout(self)

        # 시스템 운용 모드 그룹
        mode_groupbox = QGroupBox("시스템 운용 모드")
        mode_layout = QFormLayout()

        self.system_mode_combo = QComboBox()
        self.system_mode_combo.addItems(
            [
                "0: 초기화 모드",
                "1: 대기 모드",
                "2: 초기임무재계획 모드",
                "3: 임무수행 모드",
            ]
        )
        self.system_mode_combo.currentIndexChanged.connect(self.on_system_mode_changed)
        mode_layout.addRow(QLabel("현재 모드:"), self.system_mode_combo)
        mode_groupbox.setLayout(mode_layout)
        layout.addWidget(mode_groupbox)

        # 원형 진행률 표시기 그룹
        progress_groupbox = QGroupBox("임무 진행률")
        progress_layout = QHBoxLayout()

        self.progress_bars = []
        for i, aircraft_id in enumerate(self.uav_aircraft_ids):
            progress_bar = CircularProgressBar()
            progress_bar.setText(f"UAV {i+1} (ID {aircraft_id})")
            self.progress_bars.append(progress_bar)
            progress_layout.addWidget(progress_bar)

        progress_groupbox.setLayout(progress_layout)
        layout.addWidget(progress_groupbox)

        self.label = QLabel("모니터링 탭: 수신된 데이터가 아래에 표시됩니다.")
        self.display = QTextEdit()
        self.display.setReadOnly(True)
        layout.addWidget(self.label)
        layout.addWidget(self.display)

    def on_system_mode_changed(self, index):
        """QComboBox의 값이 사용자에 의해 변경되었을 때 호출되는 슬롯"""
        self.manager.set_system_mode(index)

    def refresh_display(self, update_info: tuple, data_object: object = None):
        """Manager로부터 데이터 변경 알림을 받아 화면을 갱신합니다."""
        source, key = update_info

        # 시스템 모드 변경 처리
        if (source == "logic" and key == "SystemMode") or (
            source == "receive" and key == "0101"
        ):
            if source == "receive" and key == "0101":
                new_mode = (
                    data_object.systemMode
                    if data_object
                    else self.manager.get_logic_result("SystemMode")
                )
            else:
                new_mode = self.manager.get_logic_result("SystemMode")

            if new_mode is not None:
                self.system_mode_combo.blockSignals(True)
                self.system_mode_combo.setCurrentIndex(new_mode)
                self.system_mode_combo.blockSignals(False)

        # 0501 메시지 수신 시 진행률 업데이트
        if key == "0501":
            mission_progress_data = self.manager.get_logic_result("0501_data")
            if mission_progress_data and "individualMissionProgressStatusList" in mission_progress_data:
                progress_by_aircraft = {}
                for entry in mission_progress_data["individualMissionProgressStatusList"]:
                    if isinstance(entry, dict):
                        progress_by_aircraft[entry.get("aircraftID")] = entry

                for idx, aircraft_id in enumerate(self.uav_aircraft_ids):
                    if idx >= len(self.progress_bars):
                        break
                    bar = self.progress_bars[idx]
                    entry = progress_by_aircraft.get(aircraft_id, {})
                    progress_value = entry.get("currentIndividualMissionProgress")
                    if progress_value is None:
                        progress_value = 0
                    print(f"UAV {idx+1} (ID {aircraft_id}) Progress: {progress_value}%")
                    try:
                        bar.setValue(int(progress_value))
                    except Exception:
                        bar.setValue(0)
                    bar.setText(f"UAV {idx+1} (ID {aircraft_id}): {progress_value}%")

        # fuel_data 업데이트 및 색상 변경
        if source == "logic" and key == "fuel_data":
            fuel_data = self.manager.logic_store.get_data("fuel_data")
            if fuel_data and isinstance(fuel_data, list):
                fuel_by_aircraft = {}
                for fuel_item in fuel_data:
                    if isinstance(fuel_item, dict):
                        fuel_by_aircraft[fuel_item.get("id")] = fuel_item

                for idx, aircraft_id in enumerate(self.uav_aircraft_ids):
                    if idx >= len(self.progress_bars):
                        break
                    bar = self.progress_bars[idx]
                    fuel_item = fuel_by_aircraft.get(aircraft_id)
                    if not fuel_item:
                        bar.setText(f"UAV {idx+1} (ID {aircraft_id}) Fuel: N/A")
                        bar.setColor(QColor(0, 255, 0))
                        continue

                    warning_text = fuel_item.get("warning", "green") or "green"
                    display_text = warning_text

                    if warning_text == "red":
                        color = QColor(255, 0, 0)  # Red
                    elif warning_text == "yellow":
                        color = QColor(255, 255, 0)  # Yellow
                    elif warning_text == "unknown":
                        color = QColor(128, 128, 128)  # Grey for unknown
                    else:
                        color = QColor(0, 255, 0)  # Green
                        if not warning_text:
                            display_text = "green"

                    bar.setText(f"UAV {idx+1} (ID {aircraft_id}) Fuel: {display_text}")
                    bar.setColor(color)
        # 기존의 데이터 로깅 로직
        if not (source == "receive" and key):
            return

        data_to_display = (
            data_object if data_object else self.manager.get_received_data(key)
        )

        if data_to_display is None:
            return

        try:
            data_str = json.dumps(
                data_to_display.__dict__, indent=2, ensure_ascii=False, default=str
            )
        except Exception:
            data_str = str(data_to_display)

        log_message = f"""--- 수신 (ID: {key}) ---
{data_str}
"""
        current_text = self.display.toPlainText()
        self.display.setText(log_message + "\n" + current_text)
