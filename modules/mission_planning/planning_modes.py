from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Any, Dict


DEFAULT_UNMANNED_LOGIC = "enhanced_default_unmanned_v1"
DEFAULT_MANNED_LOGIC = "enhanced_default_manned_v1"
DEFAULT_NEXT_COLLAB_LOGIC = "next_collab_default_v1"
DEFAULT_SPLIT_LOGIC = "enhanced_split_default_v1"
DEFAULT_TYPE_DECIDER_LOGIC = "enhanced_type_decider_default_v1"


@dataclass(frozen=True)
class MissionPlanningMode:
    package_type: int
    code: str
    name: str
    unmanned_logic: str = DEFAULT_UNMANNED_LOGIC
    manned_logic: str = DEFAULT_MANNED_LOGIC
    next_collab_logic: str = DEFAULT_NEXT_COLLAB_LOGIC
    split_logic: str = DEFAULT_SPLIT_LOGIC
    type_decider_logic: str = DEFAULT_TYPE_DECIDER_LOGIC

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)


MISSION_PLANNING_MODES: Dict[int, MissionPlanningMode] = {
    1: MissionPlanningMode(
        package_type=1,
        code="anti_armor_air_strike",
        name="Type 1 Anti-armor air strike",
    ),
    2: MissionPlanningMode(
        package_type=2,
        code="ground_maneuver_support",
        name="Type 2 Ground maneuver support",
    ),
    3: MissionPlanningMode(
        package_type=3,
        code="air_assault_cover",
        name="Type 3 Air assault cover",
    ),
    4: MissionPlanningMode(
        package_type=4,
        code="critical_facility_protection",
        name="Type 4 Critical facility protection",
    ),
    5: MissionPlanningMode(
        package_type=5,
        code="urban_operation",
        name="Type 5 Urban operation",
    ),
}

UNKNOWN_MISSION_PLANNING_MODE = MissionPlanningMode(
    package_type=0,
    code="default",
    name="Default mission planning",
)


def _to_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def _extract_package_type(payload: Any) -> int:
    if isinstance(payload, MissionPlanningMode):
        return int(payload.package_type)
    if isinstance(payload, Mapping):
        for key in (
            "inputMissionPackageType",
            "InputMissionPackageType",
            "inputMissionPackageTypeCode",
            "packageType",
        ):
            if key in payload:
                return _to_int(payload.get(key), 0)
    return 0


def resolve_mission_planning_mode(
    payload: Any = None,
    *,
    package_type: Any = None,
) -> MissionPlanningMode:
    if isinstance(payload, MissionPlanningMode):
        return payload
    resolved_package_type = _to_int(package_type, 0)
    if resolved_package_type <= 0:
        resolved_package_type = _extract_package_type(payload)
    mode = MISSION_PLANNING_MODES.get(int(resolved_package_type))
    if mode is not None:
        return mode
    if resolved_package_type > 0:
        return MissionPlanningMode(
            package_type=int(resolved_package_type),
            code=f"type_{int(resolved_package_type)}",
            name=f"Type {int(resolved_package_type)} mission planning",
        )
    return UNKNOWN_MISSION_PLANNING_MODE


def mission_mode_context(
    payload: Any = None,
    *,
    package_type: Any = None,
    mode: MissionPlanningMode | Mapping[str, Any] | None = None,
) -> Dict[str, Any]:
    if isinstance(mode, MissionPlanningMode):
        return mode.as_dict()
    if isinstance(mode, Mapping):
        return dict(mode)
    return resolve_mission_planning_mode(payload, package_type=package_type).as_dict()


def mode_log_label(mode: MissionPlanningMode | Mapping[str, Any] | None) -> str:
    ctx = mission_mode_context(mode=mode)
    return (
        f"packageType={_to_int(ctx.get('package_type'), 0)} "
        f"mode={ctx.get('code') or 'default'} "
        f"uav={ctx.get('unmanned_logic') or DEFAULT_UNMANNED_LOGIC} "
        f"lah={ctx.get('manned_logic') or DEFAULT_MANNED_LOGIC} "
        f"nextCollab={ctx.get('next_collab_logic') or DEFAULT_NEXT_COLLAB_LOGIC}"
    )
