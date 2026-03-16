from typing import Dict, Optional, Union

from .message_models import (
    AgentStatusModel,
    BaseBehaviorModel,
    BattlefieldSituationAwarenessInfoModel,
    DecisionResultModel,
    FlightReferenceInfoModel,
    ForcedCommandModel,
    IndividualMissionPlanModel,
    InputMissionPlanModel,
    LAHFlightPlanModel,
    MissionPlanModel,
    ModuleStatusModelModel,
    NextCollaborativeBaseMissionCommandModel,
    OperationEventModel,
    OperatorMissionReplanCommandModel,
    PerformanceMissionUpdateCommandModel,
    PriorMissionInfoModel,
    RequestDataModel,
    SystemBootCommandModel,
    SystemOperationModeModel,
    UAVFlightPlanModel,
)

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
    """Singleton store for received message models."""

    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__(self):
        if not hasattr(self, "_initialized"):
            self._data: Dict[str, MessageData] = {}
            self._initialized = True

    def set_data(self, key: str, value: MessageData) -> None:
        self._data[key] = value

    def get_data(self, key: str) -> Optional[MessageData]:
        return self._data.get(key)

    def get_all_data(self) -> Dict[str, MessageData]:
        return self._data.copy()
