from __future__ import annotations

from copy import deepcopy
from typing import Any, Callable, Dict

from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
    prepare_next_collab_input_replacements,
)
from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float

REEXECUTE_FIRST_TRACKED_UAV_IDS = (4, 5, 6)


def _int_or_none(value: Any) -> int | None:
    try:
        if value is None or value == "":
            return None
        return int(value)
    except Exception:
        return None


def _mission_input_id(mission: Dict[str, Any] | None) -> int | None:
    if not isinstance(mission, dict):
        return None
    direct = _int_or_none(mission.get("inputMissionID"))
    if direct is not None:
        return int(direct)
    related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    return _int_or_none(related.get("inputMissionID"))


def _coord_key(coord: Dict[str, Any] | None) -> tuple[float, float, float | None] | None:
    if not isinstance(coord, dict):
        return None
    try:
        lat = round(float(coord.get("latitude")), 7)
        lon = round(float(coord.get("longitude")), 7)
    except Exception:
        return None
    alt_value = None
    try:
        alt_value = round(float(coord.get("altitude")), 2)
    except Exception:
        pass
    return lat, lon, alt_value


def _coord_label(coord: Dict[str, Any] | None) -> str:
    if not isinstance(coord, dict):
        return "-"
    try:
        lat = float(coord.get("latitude"))
        lon = float(coord.get("longitude"))
    except Exception:
        return "-"
    alt_text = ""
    try:
        alt_text = f",alt={float(coord.get('altitude')):.1f}"
    except Exception:
        pass
    return f"{lat:.7f},{lon:.7f}{alt_text}"


def reexecute_first_mission_generic_skip_policy(
    *,
    current_input_id: Any,
    source_template_input_id: Any = None,
) -> Dict[str, Any]:
    """Describe which generic 0303 rows may be replaced by the hybrid result."""

    current_id = _int_or_none(current_input_id)
    source_id = _int_or_none(source_template_input_id)
    if source_id is None:
        source_id = current_id
    return {
        "valid": current_id is not None and current_id > 0,
        "currentInputMissionID": current_id,
        "sourceTemplateInputID": source_id,
        "sourceCurrentDifferent": (
            current_id is not None
            and source_id is not None
            and int(current_id) != int(source_id)
        ),
        "genericSkipInputMissionID": current_id,
        "skipUsesCurrentInput": True,
        "skipPolicy": "current_input_only",
    }


def validate_reexecute_first_mission_inputs(
    *,
    current_input_mission: Dict[str, Any],
    entry_coord_map: Dict[int, Dict[str, Any]],
    source_template_input_id: Any = None,
) -> Dict[str, Any]:
    """Validate the execute=2 first-mission hybrid request contract."""

    current_id = _mission_input_id(current_input_mission)
    source_id = _int_or_none(source_template_input_id)
    if source_id is None:
        source_id = current_id
    entry_aircraft_ids = []
    invalid_aircraft_ids = []
    invalid_coordinate_ids = []
    coordinate_keys = []
    for raw_aid, coord in sorted((entry_coord_map or {}).items(), key=lambda item: _int_or_none(item[0]) or 0):
        aid = _int_or_none(raw_aid)
        if aid is None:
            continue
        entry_aircraft_ids.append(int(aid))
        if int(aid) not in REEXECUTE_FIRST_TRACKED_UAV_IDS:
            invalid_aircraft_ids.append(int(aid))
        coord_key = _coord_key(coord)
        if coord_key is None:
            invalid_coordinate_ids.append(int(aid))
        else:
            coordinate_keys.append(coord_key)
    skip_policy = reexecute_first_mission_generic_skip_policy(
        current_input_id=current_id,
        source_template_input_id=source_id,
    )
    three_uav_ids = set(REEXECUTE_FIRST_TRACKED_UAV_IDS)
    three_uav_entry_coordinate_set = (
        set(entry_aircraft_ids) == three_uav_ids
        and not invalid_aircraft_ids
        and not invalid_coordinate_ids
    )
    entry_coordinate_distinct = (
        bool(coordinate_keys)
        and len(set(coordinate_keys)) == len(coordinate_keys)
    )
    valid = (
        current_id is not None
        and current_id > 0
        and source_id is not None
        and bool(entry_aircraft_ids)
        and not invalid_aircraft_ids
        and not invalid_coordinate_ids
    )
    return {
        "valid": bool(valid),
        "currentInputMissionID": current_id,
        "sourceTemplateInputID": source_id,
        "sourceCurrentDifferent": bool(skip_policy.get("sourceCurrentDifferent")),
        "genericSkipInputMissionID": skip_policy.get("genericSkipInputMissionID"),
        "skipUsesCurrentInput": bool(skip_policy.get("skipUsesCurrentInput")),
        "entryAircraftIDs": entry_aircraft_ids,
        "entryAircraftUavOnly": not invalid_aircraft_ids,
        "invalidEntryAircraftIDs": invalid_aircraft_ids,
        "invalidEntryCoordinateAircraftIDs": invalid_coordinate_ids,
        "threeUavEntryCoordinateSet": bool(three_uav_entry_coordinate_set),
        "entryCoordinateDistinct": bool(entry_coordinate_distinct),
        "skipPolicy": skip_policy,
    }


def summarize_reexecute_first_mission_option_effect(
    *,
    option_code: Any,
    option_label: str | None = None,
    share_policy: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    option_int = _int_or_none(option_code)
    label_text = str(option_label or "").strip().lower()
    is_recon = option_int == 4 or "recon" in label_text
    policy = share_policy if isinstance(share_policy, dict) else {}
    reason = str(policy.get("reason") or "")
    return {
        "optionCode": option_int,
        "optionLabel": str(option_label or ""),
        "reconRuntimeOverride": bool(is_recon),
        "hybridShareAllowed": bool(policy.get("shareAllowed")),
        "sharePolicyReason": reason,
        "runtimeOverrideAffectsHybridSharing": bool(
            is_recon and reason == "recon_runtime_override"
        ),
    }


def _reexecute_first_search_speed_scale() -> float:
    try:
        value = float(get_runtime_float("replan_sweep_speed_scale", 1.3))
    except Exception:
        value = 1.3
    if value <= 0.0:
        return 1.0
    return max(float(value), 0.1)


def prepare_reexecute_first_mission_replacements(
    *,
    source_plan_id: int,
    current_input_mission: Dict[str, Any],
    entry_coord_map: Dict[int, Dict[str, Any]],
    heading_map: Dict[int, float] | None = None,
    entry_aircraft_context_map: Dict[int, Dict[str, Any]] | None = None,
    representative_entry: Dict[str, Any] | None = None,
    next_input_mission: Dict[str, Any] | None = None,
    turn_radius_scale: float | None = None,
    source_template_input_id: int | None = None,
    log: Callable[[str], None] | None = None,
):
    """Build the execute=2 first-mission replacement through a separate entry point.

    The path generation intentionally mirrors the next-collab algorithm, but this
    wrapper keeps execute=2 behavior isolated from the 0803 pipeline.
    """

    emit = log or (lambda _msg: None)
    input_validation = validate_reexecute_first_mission_inputs(
        current_input_mission=current_input_mission,
        entry_coord_map=entry_coord_map,
        source_template_input_id=source_template_input_id,
    )
    input_id = int(input_validation.get("currentInputMissionID") or 0)
    source_template_id = int(input_validation.get("sourceTemplateInputID") or input_id)
    aircraft_ids = sorted(int(aid) for aid in input_validation.get("entryAircraftIDs") or [])
    entry_summary = ", ".join(
        f"A{int(aid)}={_coord_label(entry_coord_map.get(int(aid)))}"
        for aid in aircraft_ids
    )
    emit(
        "[REEXEC-FIRST] current-position first mission builder armed: "
        f"sourcePlan={int(source_plan_id)}, inputMissionID={int(input_id)}, "
        f"sourceTemplateInputID={source_template_id}, "
        f"aircraft={aircraft_ids}, entries={entry_summary}"
    )
    emit(f"[REEXEC-FIRST] input validation: {input_validation}")
    emit(f"[REEXEC-FIRST] generic skip policy: {input_validation.get('skipPolicy')}")
    if not bool(input_validation.get("valid")):
        emit(f"[WARN] [REEXEC-FIRST] input validation failed: {input_validation}")

    search_speed_scale_multiplier = _reexecute_first_search_speed_scale()
    if abs(float(search_speed_scale_multiplier) - 1.0) > 1e-6:
        emit(
            "[REEXEC-FIRST] searchSpeed scale multiplier armed: "
            f"factor={float(search_speed_scale_multiplier):.2f}"
        )

    def _relay_helper_log(message: str) -> None:
        text = str(message)
        text = text.replace("[NEXTCOLLAB]", "[REEXEC-FIRST]")
        emit(text)

    return prepare_next_collab_input_replacements(
        source_plan_id=int(source_plan_id),
        target_input_mission=deepcopy(current_input_mission),
        entry_coord_map={int(aid): dict(coord) for aid, coord in entry_coord_map.items()},
        heading_map={int(aid): float(val) for aid, val in dict(heading_map or {}).items()},
        entry_aircraft_context_map={
            int(aid): dict(row)
            for aid, row in dict(entry_aircraft_context_map or {}).items()
            if isinstance(row, dict)
        },
        representative_entry=deepcopy(representative_entry)
        if isinstance(representative_entry, dict)
        else None,
        next_input_mission=deepcopy(next_input_mission)
        if isinstance(next_input_mission, dict)
        else None,
        turn_radius_scale=turn_radius_scale,
        search_speed_scale_multiplier=float(search_speed_scale_multiplier),
        source_template_input_id=source_template_input_id,
        log=_relay_helper_log,
    )
