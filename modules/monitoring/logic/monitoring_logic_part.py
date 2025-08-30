# logic/monitoring_logic_part.py
from datetime import datetime
from typing import Union

# --- 데이터 모델 import ---
from data.message_models import (
    ModuleStatusModelModel,
    MissionPerformanceStatusBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
)

# --- 반환 가능한 모든 Push 메시지 본문 타입을 정의 ---
PushBodyType = Union[
    ModuleStatusModelModel,
    MissionPerformanceStatusBodyModel,
    MissionEndRequestBodyModel,
    ReplanRequestBodyModel,
]

class MonitoringLogic:
    def __init__(self, manager):
        self.manager = manager

    def execute(self):
        """시스템 모드를 확인하고, 'monitoring'일 경우에만 로직을 실행합니다."""
        system_mode = self.manager.logic_store.get_data("system_mode")
        
        if system_mode == "monitoring":
            # 향후 실제 모니터링 로직 추가
            self.manager._log("MON_LOGIC", "EXEC", "모니터링 로직 실행됨.")

    def generate_body_for(self, msg_id: str) -> PushBodyType:
        """메시지 ID에 따라 데이터 클래스 인스턴스를 생성하여 반환합니다."""
        timestamp = int(
            (datetime.utcnow() - datetime(2000, 1, 1)).total_seconds() * 1000
        )
        source_module = "MonitoringModule"

        if msg_id == "0102":
            return ModuleStatusModelModel(
                timestamp=timestamp, source=source_module, status=1
            )
        elif msg_id == "0501":
            return MissionPerformanceStatusBodyModel(
                timestamp=timestamp, sourceModuleName=source_module, missionStatus=1
            )
        elif msg_id == "0502":
            return MissionEndRequestBodyModel(
                timestamp=timestamp, sourceModuleName=source_module, reason=0
            )
        elif msg_id == "0902":
            return ReplanRequestBodyModel(
                timestamp=timestamp,
                sourceModuleName=source_module,
                replanRequest="ManualTrigger",
            )

        raise ValueError(f"Body generation not implemented for msg_id: {msg_id}")
