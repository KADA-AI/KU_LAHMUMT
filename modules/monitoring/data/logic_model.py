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
    Represents the input data required for the re-planning logic.
    This will be fetched from receive_storage.py.
    """
    timestamp: int
    input_mission_plan: Optional[InputMissionPlanModel] = None
    operator_replan_command: Optional[OperatorMissionReplanCommandModel] = None
    replan_request: Optional[ReplanRequestBodyModel] = None
    battlefield_situation: Optional[BattlefieldSituationAwarenessInfoModel] = None
    agent_status: Optional[AgentStatusModel] = None
    forced_command: Optional[ForcedCommandModel] = None
    # Add other relevant fields as identified from receive_storage.py

@dataclass
class IntermediateReplanResult:
    """
    Represents an intermediate result from one step of the re-planning process.
    """
    step_name: str
    data: Any
    timestamp: int # Changed to int to match message_models

@dataclass
class FinalReplanOutput:
    """
    Represents the final output of the re-planning process.
    """
    new_plan: Dict[str, Any]
    replan_status: str
    final_replan_type: Optional[str] = None
    # Add other relevant fields