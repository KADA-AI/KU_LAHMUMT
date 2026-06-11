from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from .message_models import (
    InputMissionPlanModel,
    OperatorMissionReplanCommandModel,
    ReplanRequestBodyModel,
    BattlefieldSituationAwarenessInfoModel,
    AgentStatusModel,
    ForcedCommandModel,
)


@dataclass
class ReplanInputData:
    """
    Aggregated input required for the re-plan logic.
    """

    timestamp: int
    input_mission_plan: Optional[InputMissionPlanModel] = None
    operator_replan_command: Optional[OperatorMissionReplanCommandModel] = None
    replan_request: Optional[ReplanRequestBodyModel] = None
    battlefield_situation: Optional[BattlefieldSituationAwarenessInfoModel] = None
    agent_status: Optional[AgentStatusModel] = None
    forced_command: Optional[ForcedCommandModel] = None


@dataclass
class IntermediateReplanResult:
    """
    Intermediate step result for the re-plan process.
    """

    step_name: str
    data: Any
    timestamp: int


@dataclass
class FinalReplanOutput:
    """
    Final outcome structure for the re-plan process.
    """

    new_plan: Dict[str, Any]
    replan_status: str
    final_replan_type: Optional[str] = None
