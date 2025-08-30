# data/receive_storage.py
from typing import Dict, Union, Optional

# --- 데이터 모델 import ---
# 앞으로 여기에 생성하는 모든 메시지 모델을 추가합니다.
from .message_models import (
    SystemOperationModeModel,
    ModuleStatusModelModel,
    InputMissionPlanModel,
    PriorMissionInfoModel,
    FlightReferenceInfoModel,
    MissionPlanModel,
    IndividualMissionPlanModel,
    UAVFlightPlanModel,
    LAHFlightPlanModel,
    AgentStatusModel,
    BattlefieldSituationAwarenessInfoModel,
    BaseBehaviorModel,
    DecisionResultModel,
    OperatorMissionReplanCommandModel,
    ForcedCommandModel,
    NextCollaborativeBaseMissionCommandModel,
    OperationEventModel,
    SystemBootCommandModel,
    PerformanceMissionUpdateCommandModel,
    RequestDataModel,
)

# --- 저장 가능한 모든 메시지 타입을 정의 ---
# Union을 사용하여 타입 힌트에서 여러 타입 허용
MessageData = Union[
    SystemOperationModeModel,
    ModuleStatusModelModel,
    InputMissionPlanModel,
    PriorMissionInfoModel,
    FlightReferenceInfoModel,
    MissionPlanModel,
    IndividualMissionPlanModel,
    UAVFlightPlanModel,
    LAHFlightPlanModel,
    AgentStatusModel,
    BattlefieldSituationAwarenessInfoModel,
    BaseBehaviorModel,
    DecisionResultModel,
    OperatorMissionReplanCommandModel,
    ForcedCommandModel,
    NextCollaborativeBaseMissionCommandModel,
    OperationEventModel,
    SystemBootCommandModel,
    PerformanceMissionUpdateCommandModel,
    RequestDataModel,
]


class ReceiveStorage:
    """nFusion으로부터 수신된 메시지를 파싱된 데이터 클래스 객체로 저장합니다."""

    _instance = None  # 싱글톤 인스턴스를 저장할 변수

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        # __init__은 __new__가 호출될 때마다 호출될 수 있으므로, 초기화는 한 번만 수행되도록 방지
        if not hasattr(self, "_initialized"):
            self._data: Dict[str, MessageData] = {}
            self._initialized = True

    def set_data(self, key: str, value: MessageData):
        """메시지 ID를 키로, 해당 메시지의 데이터 클래스 인스턴스를 저장합니다."""
        self._data[key] = value

    def get_data(self, key: str) -> Optional[MessageData]:
        """특정 메시지 ID의 데이터 클래스 인스턴스를 반환합니다."""
        return self._data.get(key)

    def get_all_data(self) -> Dict[str, MessageData]:
        """저장된 모든 데이터의 복사본을 반환합니다."""
        return self._data.copy()
