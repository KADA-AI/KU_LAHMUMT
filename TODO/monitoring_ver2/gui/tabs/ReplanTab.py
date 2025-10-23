# gui/tabs/ReplanTab.py: 메인 GUI의 '재계획 판단' 탭에 해당하는 UI와 기능을 정의합니다.

from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QVBoxLayout,
    QComboBox,
    QGroupBox,
    QFormLayout,
    QGridLayout,
)
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtCore import Qt


class ReplanTab(QWidget):
    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager

        # 전체 레이아웃
        main_layout = QVBoxLayout(self)

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

        # 시그널 연결 (사용자 변경시)
        self.system_mode_combo.currentIndexChanged.connect(self.on_system_mode_changed)

        mode_layout.addRow(QLabel("현재 모드:"), self.system_mode_combo)
        mode_groupbox.setLayout(mode_layout)

        main_layout.addWidget(mode_groupbox)

        # --- 재계획 요약 그룹 --- #
        replan_summary_groupbox = QGroupBox("재계획 요약")
        summary_form_layout = QFormLayout()

        self.replan_status_label = QLabel("N/A")
        self.new_plan_status_label = QLabel("N/A")
        self.final_replan_type_label = QLabel("N/A")

        summary_form_layout.addRow(QLabel("재계획 상태:"), self.replan_status_label)
        summary_form_layout.addRow(QLabel("새 계획 상태:"), self.new_plan_status_label)
        summary_form_layout.addRow(QLabel("최종 재계획 유형:"), self.final_replan_type_label)

        replan_summary_groupbox.setLayout(summary_form_layout)
        main_layout.addWidget(replan_summary_groupbox)

        # --- 규칙 기반 재계획 판단 결과 그룹 --- #
        replan_rules_groupbox = QGroupBox("규칙 기반 재계획 판단 결과")
        rules_grid_layout = QGridLayout()

        # 규칙 레이블 초기화 및 스타일 설정
        self.rule_labels = {}
        self._init_rule_label(rules_grid_layout, "UAV 고장", "uav_health", 0, 0, priority=2)
        self._init_rule_label(rules_grid_layout, "UAV 페이로드 고장", "uav_payload_health", 0, 1, priority=2)
        self._init_rule_label(rules_grid_layout, "UAV 연료 부족", "uav_fuel_warning", 0, 2, priority=2)
        self._init_rule_label(rules_grid_layout, "UAV 추적 중", "uav_tracking", 0, 3, priority=3)
        self._init_rule_label(rules_grid_layout, "강제 귀환 명령", "forced_return", 1, 0, priority=1)
        self._init_rule_label(rules_grid_layout, "임무 일시 정지 명령", "mission_pause", 1, 1, priority=1)

        replan_rules_groupbox.setLayout(rules_grid_layout)
        main_layout.addWidget(replan_rules_groupbox)

        # 나머지 공간을 위한 스트레치
        main_layout.addStretch(1)

        self.setLayout(main_layout)

        # 초기값 설정
        self.refresh_display(("logic", "SystemMode"))

    def _init_rule_label(self, layout, text, name, row, col, priority=None):
        display_text = f"P{priority}: {text}" if priority is not None else text
        label = QLabel(display_text)
        label.setObjectName(name)  # 객체 이름 설정
        label.setAlignment(Qt.AlignCenter)
        label.setFixedSize(120, 50)  # 고정 크기
        label.setStyleSheet(
            "QLabel { border: 1px solid gray; background-color: lightgray; }"
        )
        self.rule_labels[name] = label
        layout.addWidget(label, row, col)

    def on_system_mode_changed(self, index):
        """QComboBox의 값이 사용자에 의해 변경되었을 때 호출되는 슬롯"""
        # manager를 통해 시스템 모드 변경 요청
        self.manager.set_system_mode(index)

    def refresh_display(self, update_info, data_object=None):
        """manager로부터 데이터 변경 알림을 받았을 때 호출됩니다."""
        source, key = update_info

        if source == "logic" and key == "SystemMode":
            new_mode = self.manager.get_logic_result("SystemMode")
            if new_mode is not None:
                # 무한 루프를 방지하기 위해 시그널을 잠시 끊음
                self.system_mode_combo.blockSignals(True)
                self.system_mode_combo.setCurrentIndex(new_mode)
                self.system_mode_combo.blockSignals(False)
        elif source == "receive" and key == "0101":
            if data_object and hasattr(data_object, "systemMode"):
                new_mode = data_object.systemMode
                # 무한 루프를 방지하기 위해 시그널을 잠시 끊음
                self.system_mode_combo.blockSignals(True)
                self.system_mode_combo.setCurrentIndex(new_mode)
                self.system_mode_combo.blockSignals(False)

        # --- 재계획 요약 업데이트 ---
        final_replan_output = self.manager.logic_store.get_data("final_replan_output")
        if final_replan_output:
            self.replan_status_label.setText(final_replan_output.replan_status)
            self.new_plan_status_label.setText(final_replan_output.new_plan.get("status", "N/A"))
            self.final_replan_type_label.setText(final_replan_output.final_replan_type if final_replan_output.final_replan_type else "N/A")

        # --- 규칙 기반 재계획 판단 결과 업데이트 ---
        # 모든 규칙 레이블의 배경색을 기본값으로 재설정
        for name, label in self.rule_labels.items():
            label.setStyleSheet(
                "QLabel { border: 1px solid gray; background-color: lightgray; }"
            )

        # logic_storage에서 재계획 트리거 결과 가져오기
        replan_triggers = self.manager.logic_store.get_data("replan_triggers")

        if replan_triggers:
            for trigger in replan_triggers:
                replan_reason = trigger.get("ReplanReason")
                # 각 ReplanReason에 따라 해당 레이블의 배경색 변경
                if "고장" in replan_reason and "페이로드" not in replan_reason:
                    self.rule_labels["uav_health"].setStyleSheet(
                        "QLabel { border: 1px solid gray; background-color: lightcoral; }"
                    )
                elif "페이로드 고장" in replan_reason:
                    self.rule_labels["uav_payload_health"].setStyleSheet(
                        "QLabel { border: 1px solid gray; background-color: lightcoral; }"
                    )
                elif "연료 부족" in replan_reason:
                    self.rule_labels["uav_fuel_warning"].setStyleSheet(
                        "QLabel { border: 1px solid gray; background-color: lightcoral; }"
                    )
                elif "추적 중" in replan_reason:
                    self.rule_labels["uav_tracking"].setStyleSheet(
                        "QLabel { border: 1px solid gray; background-color: lightyellow; }"
                    )
                elif "강제 귀환 명령" in replan_reason:
                    self.rule_labels["forced_return"].setStyleSheet(
                        "QLabel { border: 1px solid gray; background-color: lightblue; }"
                    )
                elif "임무 일시 정지 명령" in replan_reason:
                    self.rule_labels["mission_pause"].setStyleSheet(
                        "QLabel { border: 1px solid gray; background-color: lightsalmon; }"
                    )
