# data/message_models.py: 외부와 주고받는 메시지의 데이터 구조를 정의하는 데이터클래스(dataclass)들을 포함합니다.

# data/message_models.py
# 이 파일은 nFusion 메시지의 내용을 담는 데이터 클래스를 정의합니다.

from dataclasses import dataclass, field
from typing import List, Optional

# -----------------------------------------------------------------------------
# 참고: 이 클래스들은 receiver 파일들의 _to_dict... 함수들을 기반으로 생성되었습니다.
# -----------------------------------------------------------------------------

# ----------------- Common Sub-structures ----------------- #


@dataclass
class CoordinateModel:
    latitude: float
    longitude: float
    altitude: int


@dataclass
class LineModel:
    width: float
    coordinateList: List[CoordinateModel]


@dataclass
class AreaModel:
    isHole: bool
    coordinateList: List[CoordinateModel]


@dataclass
class VelocityModel:
    speed: float
    heading: float


# ----------------- Message 0000 (RequestData) ----------------- #
@dataclass
class RequestDataModel:
    """0000 RequestData 메시지"""

    timestamp: int
    source: str
    messageID: int


# ----------------- Message 0101 ----------------- #
@dataclass
class SystemOperationModeModel:
    """0101 SystemOperationMode 메시지"""

    timestamp: int
    systemMode: int


# ----------------- Message 0102 ----------------- #
@dataclass
class ModuleStatusModelModel:
    """0102 ModuleStatus 메시지"""

    timestamp: int
    source: str
    status: int


# ----------------- Message 0103 ----------------- #
@dataclass
class SWStatusModel:
    """0103 SWStatus 메시지"""

    timestamp: int
    source: str
    status: int
    mode: int


# ----------------- Message 0201 (InputMissionPlan) ----------------- #


@dataclass
class AvailableAircraftModel:
    aircraftID: int


@dataclass
class MissionDetailModel:
    coordinateList: List[CoordinateModel]
    lineList: List[LineModel]
    areaList: List[AreaModel]


@dataclass
class InputMissionModel:
    inputMissionID: int
    inputMissionType: int
    isDone: bool
    missionDetail: MissionDetailModel


@dataclass
class InputMissionPlanModel:
    """0201 InputMissionPlan 메시지"""

    timestamp: int
    inputMissionPackageID: int
    inputMissionPackageType: int
    mainSensor: int
    availableAircraftList: List[AvailableAircraftModel]
    inputMissionList: List[InputMissionModel]


# ----------------- Message 0202 (PriorMissionInfo) ----------------- #


@dataclass
class TargetOrientationModel:
    targetID: int


@dataclass
class CoordinateOrientationModel:
    coordinate: CoordinateModel


@dataclass
class PriorMissionModel:
    priorMissionID: int
    missionType: int
    coordinateOrientation: CoordinateOrientationModel
    targetOrientation: TargetOrientationModel


@dataclass
class PriorMissionInfoModel:
    """0202 PriorMissionInfo 메시지"""

    timestamp: int
    source: str
    priorMissionList: List[PriorMissionModel]


# ----------------- Message 0203 (FlightReferenceInfo) ----------------- #
@dataclass
class FlightReferenceInfoModel:
    """0203 FlightReferenceInfo 메시지"""

    timestamp: int
    missionReferencePackageID: int
    inputTimestamp: int
    takeOverInfoList: List["TakeOverInfoModel"]
    handOverInfoList: List["HandOverInfoModel"]
    rtbCoordinateList: List["RTBCoordinateModel"]
    flightAreaList: List["FlightAreaModel"]
    prohibitedAreaList: List["ProhibitedAreaModel"]


@dataclass
class TakeOverInfoModel:
    aircraftID: int
    coordinate: CoordinateModel


@dataclass
class HandOverInfoModel:
    aircraftID: int
    coordinate: CoordinateModel


@dataclass
class RTBCoordinateModel:
    latitude: float
    longitude: float
    altitude: int


@dataclass
class AreaLatLonModel:
    latitude: float
    longitude: float


@dataclass
class AltitudeLimitsModel:
    lowerLimit: float
    upperLimit: float


@dataclass
class FlightAreaModel:
    flightAreaID: int
    areaLatLonList: List[AreaLatLonModel]
    altitudeLimits: AltitudeLimitsModel


@dataclass
class ProhibitedAreaModel:
    prohibitedAreaID: int
    areaLatLonList: List[AreaLatLonModel]
    altitudeLimits: AltitudeLimitsModel


# ----------------- Message 0301 (MissionPlan) ----------------- #
@dataclass
class MissionPlanModel:
    """0301 MissionPlan 메시지"""

    timestamp: int
    missionPlanID: int
    missionPlanTimestamp: int
    planningTime: float
    plannerID: int
    inputMissionPackageID: int
    missionReferencePackageID: int
    aircraftList: List["AircraftModel"]


@dataclass
class AircraftModel:
    aircraftID: int
    individualMissionPackageID: int


# ----------------- Message 0302 (IndividualMissionPlan) ----------------- #
@dataclass
class IndividualMissionPlanModel:
    """0302 IndividualMissionPlan 메시지"""

    timestamp: int
    individualMissionPackageID: int
    aircraftID: int
    individualMissionList: List["IndividualMissionModel"]


@dataclass
class IndividualMissionModel:
    individualMissionID: int
    isDone: bool
    relatedMission: "RelatedMissionModel"
    individualMissionInfo: "IndividualMissionInfoModel"
    pathID: int


@dataclass
class RelatedMissionModel:
    relatedMissionType: int
    inputMissionID: int
    priorMissionID: int


@dataclass
class IndividualMissionInfoModel:
    individualMissionType: int
    patternType: int
    autoZoomIn: bool
    coordinateList: List[CoordinateModel]
    lineList: List[LineModel]
    areaList: List[AreaModel]
    targetID: int


# ----------------- Message 0303 (UAVFlightPlan) ----------------- #
@dataclass
class UAVFlightPlanModel:
    """0303 UAVFlightPlan 메시지"""

    timestamp: int
    pathID: int
    aircraftID: int
    isFormationFlight: bool
    formationInfo: "FormationInfoModel"
    waypointList: List["WaypointModel"]


@dataclass
class WaypointModel:
    waypointID: int
    coordinate: CoordinateModel
    speed: float
    eta: int
    ecf: float
    nextWaypointID: int
    waypointPassType: int
    loiterProperty: "LoiterPropertyModel"
    filmingProperty: "FilmingPropertyModel"


@dataclass
class FilmingPropertyModel:
    fieldOfView: float
    sensorType: int
    operationMode: int
    coordinateOrientation: CoordinateOrientationModel
    lineSearch: "LineSearchModel"
    autoTracking: "AutoTrackingModel"
    aircraftFixed: "AircraftFixedModel"
    autoScan: "AutoScanModel"


@dataclass
class AutoScanModel:
    gimbalPitch: float
    gimbalYawLimits: "GimbalYawLimitsModel"
    gimbalYawAngularSpeed: float


@dataclass
class GimbalYawLimitsModel:
    leftLimit: float
    rightLimit: float


@dataclass
class AircraftFixedModel:
    gimbalPitch: float
    gimbalYaw: float


@dataclass
class AutoTrackingModel:
    targetID: int


@dataclass
class LineSearchModel:
    coordinateList: List[CoordinateModel]
    searchSpeed: float


@dataclass
class LoiterPropertyModel:
    radius: int
    direction: int
    time: int
    speed: float


@dataclass
class FormationInfoModel:
    leaderAircraftID: int
    formation: "FormationModel"


@dataclass
class FormationModel:
    dX: int
    dY: int
    dZ: int


# ----------------- Message 0304 (LAHFlightPlan) ----------------- #
@dataclass
class LAHFlightPlanModel:
    """0304 LAHFlightPlan 메시지"""

    timestamp: int
    pathID: int
    aircraftID: int
    lahWaypointList: List["LAHWaypointModel"]


@dataclass
class LAHWaypointModel:
    waypointID: int
    coordinate: CoordinateModel
    speed: float
    eta: int
    ecf: float
    nextWaypointID: int
    hovering: "HoveringModel"
    loiter: "LoiterModel"
    attack: "AttackModel"


@dataclass
class HoveringModel:
    time: int


@dataclass
class LoiterModel:
    radius: int
    direction: int
    time: int
    speed: float


@dataclass
class AttackModel:
    targetID: int
    weaponType: int


# ----------------- Message 0401 (AgentStatus) ----------------- #


@dataclass
class WeaponsModel:
    type1: int
    type2: int
    type3: int


@dataclass
class DatalinkStatusModel:
    isConnectedToUAV1: bool
    isConnectedToUAV2: bool
    isConnectedToUAV3: bool


@dataclass
class MannedInfoModel:
    weapons: WeaponsModel
    datalinkStatus: DatalinkStatusModel


@dataclass
class CurrentWaypointIDModel:
    waypointID: int


@dataclass
class LoiterCoordinateModel:
    latitude: float
    longitude: float
    altitude: int


@dataclass
class TargetFollowingModel:
    targetID: int


@dataclass
class LeaderAircraftIDModel:
    aircraftID: int


@dataclass
class CenterCoordinateModel:
    latitude: float
    longitude: float
    altitude: int


@dataclass
class SensorInfoModel:
    operationalMode: int
    sensorType: int
    fov: float
    centerCoordinate: CenterCoordinateModel


@dataclass
class UnmannedInfoModel:
    currentWaypointID: CurrentWaypointIDModel
    flightMode: int
    loiterCoordinate: LoiterCoordinateModel
    targetFollowing: TargetFollowingModel
    leaderAircraftID: LeaderAircraftIDModel
    sensorInfo: SensorInfoModel
    payloadHealth: int
    fuelWarning: int


@dataclass
class AgentStateModel:
    aircraftID: int
    isUnmanned: bool
    coordinate: CoordinateModel
    velocity: VelocityModel
    fuel: float
    health: int
    mannedInfo: Optional[MannedInfoModel]
    unmannedInfo: Optional[UnmannedInfoModel]


@dataclass
class AgentStatusModel:
    """0401 AgentStatus 메시지"""

    timestamp: int
    source: str
    agentStateList: List[AgentStateModel]


# ----------------- Message 0402 (BattlefieldSituationAwarenessInfo) ----------------- #
@dataclass
class BattlefieldSituationAwarenessInfoModel:
    """0402 BattlefieldSituationAwarenessInfo 메시지"""

    timestamp: int
    source: str
    roiInfoList: List["ROIInfoModel"] = field(default_factory=list)
    situationAwarenessInfoList: List["SituationAwarenessInfoModel"] = field(default_factory=list)
    roiInfo: Optional["ROIInfoModel"] = None
    targetList: List["TargetInfoModel"] = field(default_factory=list)


@dataclass
class ROIInfoModel:
    aircraftID: int
    coordinate: CoordinateModel
    fov: float


@dataclass
class SituationAwarenessInfoModel:
    aircraftID: int
    coordinate: CoordinateModel
    fov: float


@dataclass
class TargetWatcherModel:
    aircraftID: Optional[int] = None


@dataclass
class TargetInfoModel:
    targetID: Optional[int] = None
    targetType: Optional[int] = None
    coordinate: Optional[CoordinateModel] = None
    watcher: Optional[TargetWatcherModel] = None
    targetInFrame: Optional[bool] = None
    isDestroyed: Optional[bool] = None
    threat: Optional[float] = None


# ----------------- Message 0601 (BaseBehavior) ----------------- #
@dataclass
class BaseBehaviorModel:
    """0601 BaseBehavior 메시지"""

    timestamp: int
    source: str
    aircraft: int
    flightMode: int
    filmingMode: int


# ----------------- Message 0702 (DecisionResult) ----------------- #
@dataclass
class DecisionResultModel:
    """0702 DecisionResult 메시지"""

    timestamp: int
    source: str
    ignore: int
    missionPlanID: int


# ----------------- Message 0801 (OperatorMissionReplanCommand) ----------------- #
@dataclass
class OperatorMissionReplanCommandModel:
    """0801 OperatorMissionReplanCommand 메시지"""

    timestamp: int
    source: str
    operatorReplanRequestTime: int
    inputMissionPackageID: int
    missionReferencePackageID: int


# ----------------- Message 0802 (ForcedCommand) ----------------- #
@dataclass
class ForcedCommandModel:
    """0802 ForcedCommand 메시지"""

    timestamp: int
    source: str
    aircraftID: int
    mandatoryType: int


# ----------------- Message 0803 (NextCollaborativeBaseMissionCommand) ----------------- #
@dataclass
class NextCollaborativeBaseMissionCommandModel:
    """0803 NextCollaborativeBaseMissionCommand 메시지"""

    timestamp: int
    source: str
    execute: int


# ----------------- Message 0804 (MissionRestartCommand) ----------------- #
@dataclass
class MissionRestartCommandModel:
    """0804 MissionRestartCommand 메시지"""

    timestamp: int
    inputMissionID: int


# ----------------- Message 0805 (OperationEvent) ----------------- #
@dataclass
class OperationEventModel:
    """0805 OperationEvent 메시지"""

    timestamp: int
    source: str
    eventType: int


# ----------------- Message 0806 (SystemBootCommand) ----------------- #
@dataclass
class SystemBootCommandModel:
    """0806 SystemBootCommand 메시지"""

    timestamp: int
    source: str
    # TODO: Add specific fields for SystemBootCommand


# ----------------- Message 0901 (RequestOptionInfo) ----------------- #
@dataclass
class PendingOptionModel:
    optionID: int
    optionName: str
    missionPlanID: int


@dataclass
class RequestOptionInfoModel:
    """0901 RequestOptionInfo 메시지"""

    timestamp: int
    source: str
    requestTime: int
    pendingOptionList: List[PendingOptionModel]


# ----------------- Message 0903 (PerformanceMissionUpdateCommand) ----------------- #
@dataclass
class PerformanceMissionUpdateCommandModel:
    """0903 PerformanceMissionUpdateCommand 메시지"""

    timestamp: int
    source: str
    missionPlanID: int


# ----------------- Push Message Bodies ----------------- #


@dataclass
class IndividualMissionIDModel:
    individualMissionID: int


@dataclass
class IndividualMissionProgressStatusModel:
    aircraftID: int
    currentIndividualMission: IndividualMissionIDModel
    currentIndividualMissionProgress: int


@dataclass
class MissionProgressBodyModel:
    """0501 MissionPerformanceStatus 메시지 본문"""

    source: str
    timestamp: int
    currentMissionPlanID: int
    currentInputMissionID: int
    individualMissionProgressStatusList: List[IndividualMissionProgressStatusModel]


@dataclass
class MissionEndRequestBodyModel:
    """0502 MissionEndRequest 메시지 본문"""

    timestamp: int
    source: str
    reason: int


# ----------------- Message 0902 (ReplanRequest) ----------------- #


@dataclass
class ReplanRequestTimeStampModel:
    replanRequestTimestamp: int


@dataclass
class InputMissionIDModel:
    inputMissionID: int


@dataclass
class OptionListModel:
    optionID: int
    optionName: str
    missionPlanID: int


@dataclass
class IndividualMissionIDListModel:
    individualMissionID: int


@dataclass
class PriorMissionListModel:
    priorMissionID: int
    missionType: int = 0


@dataclass
class ReplanRequestBodyModel:
    """0902 ReplanRequest 메시지 본문"""

    source: str
    timestamp: int
    replanRequestTime: ReplanRequestTimeStampModel
    replanLevel: int
    inputMissionIDList: List[InputMissionIDModel]
    IndividualMissionIDList: List[IndividualMissionIDListModel]
    priorMissionList: List[PriorMissionListModel]
    replanRequest: str
    optionList: List[OptionListModel]


# ----------------- Message 0503 (CollaborativeMissionComplete) ----------------- #


@dataclass
class CollaborativeMissionCompleteModel:
    """0503 CollaborativeMissionComplete 메시지"""

    timestamp: int
    source: str
    systemRecommend: int


# ----------------- Message 0904 (RequestBehaviorTree) ----------------- #
@dataclass
class RequestBehaviorTreeModel:
    """0904 RequestBehaviorTree 메시지"""

    timestamp: int
    source: str
    BehaviorTreeFileID: int
