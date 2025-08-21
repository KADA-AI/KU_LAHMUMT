# mission_monitoring_tab.py
from PyQt5.QtWidgets import QPushButton, QWidget, QVBoxLayout, QGroupBox
from PyQt5.QtCore import Qt  # Qt 임포트 추가

from Tabs.csc_tab_base import CSCTabBase

# MissionMonitoringReplanCSCManager를 임포트합니다.
# 실제 환경에서는 NodeMessenger를 main_window.py 등에서 주입받아 Manager에 전달합니다.
from mission_monitoring_replan.mission_monitoring_replan_csc_manager import (
    MissionMonitoringReplanCSCManager,
    MockNodeMessenger,
)  # MockNodeMessenger는 테스트용


class MissionMonitoringTab(CSCTabBase):
    TITLE = "임무 모니터링·판단 CSC"
    # PUSH_MESSAGES와 RECEIVE_MESSAGES는 CSCManager로부터 가져옵니다.

    def __init__(self, *, messenger, parent: QWidget | None = None):
        # CSCTabBase의 PUSH/RECEIVE_MESSAGES를 Manager의 것으로 설정하기 전에
        # 임시 Manager 인스턴스를 만들어 메시지 목록을 가져옵니다.
        # 실제 Manager 인스턴스는 super().__init__() 이후에 생성합니다.
        temp_mock_messenger = MockNodeMessenger("TEMP_MMR_CSC_NM_FOR_CONFIG")
        temp_manager = MissionMonitoringReplanCSCManager(
            node_messenger=temp_mock_messenger
        )
        MissionMonitoringTab.PUSH_MESSAGES = temp_manager.get_push_messages()
        MissionMonitoringTab.RECEIVE_MESSAGES = temp_manager.get_receive_messages()

        # CSCManager를 생성합니다. 실제로는 messenger를 주입해야 합니다.
        # 테스트를 위해 MockNodeMessenger를 사용하고, 실제 환경에서는
        # main_window.py에서 생성된 NodeMessenger 인스턴스를 전달받아야 합니다.
        # self.csc_manager = MissionMonitoringReplanCSCManager(node_messenger=messenger, log_callback=self._manager_log_handler)

        # 임시: messenger 대신 MockNodeMessenger 사용
        mock_messenger_instance = MockNodeMessenger("MMR_CSC_NM")

        # CSCTabBase의 UI 요소들이 먼저 초기화되도록 super().__init__()를 먼저 호출합니다.
        super().__init__(messenger=messenger, parent=parent)  # CSCTabBase 초기화
        # self.messenger는 CSCTabBase에서 사용되므로 그대로 둡니다. (주로 push_center 호출 시)

        # super().__init__() 호출 후 CSCManager를 생성하고 콜백을 설정합니다.
        self.csc_manager = MissionMonitoringReplanCSCManager(
            node_messenger=mock_messenger_instance,
            log_callback=self._manager_log_handler,
        )
        self._add_custom_csc_ui()

        # Manager의 메시지 수신 콜백을 GUI의 mark_received와 유사하게 연결 (선택적)
        # 또는 Manager가 직접 로그를 남기고, GUI는 주기적으로 Manager 상태를 폴링하여 UI 업데이트
        # 여기서는 Manager가 _log_callback을 통해 로그를 전달하면 GUI가 처리하도록 함

    def _manager_log_handler(
        self, tag: str, log_type: str, message: str, raw_data: bytes | None
    ):
        """CSCManager로부터 로그를 받아 GUI 로그창에 기록합니다."""
        log_target = self.log_tx  # 기본적으로 송신 로그에 기록 (상황에 따라 변경 가능)
        if log_type == "RECV" or log_type == "STORE":
            log_target = self.log_rx

        self._write_log(log_target, f"{tag}-{log_type}", message, raw_data)

        # 특정 로그 타입에 따라 UI 테이블 상태 업데이트 (예시)
        if log_type == "SEND_OK":
            # msg_id를 message에서 파싱하거나, _on_send_done 콜백에서 msg_id를 받아야 함
            # 여기서는 간단히 message에 msg_id가 포함되어 있다고 가정
            for p_msg_id, _ in self.PUSH_MESSAGES:
                if p_msg_id in message:
                    self._update_state(self.tbl_tx, p_msg_id, "발신 완료 (매니저)")
                    break
        elif log_type == "RECV":
            for r_msg_id, _ in self.RECEIVE_MESSAGES:
                if r_msg_id in message:  # message에 msg_id 포함 가정
                    self._update_state(self.tbl_rx, r_msg_id, "수신 완료 (매니저)")
                    break

    def _add_custom_csc_ui(self):
        monitoring_csu_group = QGroupBox("모니터링 CSU (매니저 제어)")
        monitoring_csu_layout = QVBoxLayout()
        self.btn_trigger_monitoring_csu = QPushButton("모니터링 로직 실행 (매니저)")
        self.btn_trigger_monitoring_csu.clicked.connect(
            self.csc_manager.trigger_monitoring_csu_logic
        )
        monitoring_csu_layout.addWidget(self.btn_trigger_monitoring_csu)
        monitoring_csu_group.setLayout(monitoring_csu_layout)

        replan_csu_group = QGroupBox("재계획 CSU (매니저 제어)")
        replan_csu_layout = QVBoxLayout()
        self.btn_trigger_replan_csu = QPushButton("재계획 로직 실행 (매니저)")
        self.btn_trigger_replan_csu.clicked.connect(
            self.csc_manager.trigger_replan_csu_logic
        )
        replan_csu_layout.addWidget(self.btn_trigger_replan_csu)
        replan_csu_group.setLayout(replan_csu_layout)

        if self.layout() and self.layout().count() > 1:
            body_layout = self.layout().itemAt(1).layout()
            if body_layout and body_layout.count() > 0:
                left_side_widget = body_layout.itemAt(0).widget()
                if left_side_widget and left_side_widget.layout():
                    left_v_layout = left_side_widget.layout()
                    left_v_layout.addWidget(monitoring_csu_group)
                    left_v_layout.addWidget(replan_csu_group)

    # CSCTabBase의 mark_received는 NodeMessenger Consumer에 의해 직접 호출될 수 있습니다.
    # CSCManager가 통신을 전담한다면, 이 메서드는 GUI가 직접 사용할 일은 줄어들 수 있습니다.
    # 대신 Manager가 수신 처리를 하고, GUI는 Manager의 상태 변화를 구독하거나 콜백을 통해 UI를 업데이트합니다.
    def mark_received(self, msg_id: str, raw: bytes | None = None):
        # 이 메서드가 여전히 nFusion Consumer에 의해 호출된다면,
        # 수신된 정보를 CSCManager에게 전달할 수 있습니다.
        # self.csc_manager._handle_message_reception(msg_id, raw)
        # 또는, Manager가 nFusion Consumer를 직접 등록하고 처리하도록 합니다.
        # 현재 Manager 구조에서는 Manager가 직접 Consumer 역할을 하거나,
        # 외부(예: main_window)에서 메시지를 받아 Manager의 _handle_message_reception을 호출해야 합니다.

        # 여기서는 CSCTabBase의 기본 동작을 유지하되, Manager 로그를 통해 중복 로깅될 수 있음을 인지합니다.
        super().mark_received(msg_id, raw)
        self._write_log(
            self.log_rx, "GUI_RECV", f"GUI가 직접 {msg_id} 수신 (CSCTabBase)", raw
        )

    # _on_tx_double_clicked는 CSCTabBase의 push_center를 사용합니다.
    # CSCManager가 송신도 전담한다면, 이 메서드도 Manager의 send_message를 호출하도록 변경할 수 있습니다.
    # def _on_tx_double_clicked(self, row: int, _col: int):
    #     msg_id = self.tbl_tx.item(row, 0).text()
    #     # payload 구성 필요
    #     payload = {"gui_triggered": True, "content": f"Data for {msg_id} from GUI"}
    #     self.csc_manager.send_message(msg_id, payload)
    #     # UI 상태 업데이트는 Manager의 _on_send_done 콜백과 _manager_log_handler를 통해 이루어짐
