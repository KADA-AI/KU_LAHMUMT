from __future__ import annotations

from copy import deepcopy
from typing import Any, Callable, Dict

from modules.mission_planning.pipelines.next_collab_replan_pipeline_impl import (
    prepare_next_collab_input_replacements,
)


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


def prepare_reexecute_first_mission_replacements(
    *,
    source_plan_id: int,
    current_input_mission: Dict[str, Any],
    entry_coord_map: Dict[int, Dict[str, Any]],
    heading_map: Dict[int, float] | None = None,
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
    try:
        input_id = int(current_input_mission.get("inputMissionID"))
    except Exception:
        input_id = 0
    aircraft_ids = sorted(int(aid) for aid in entry_coord_map.keys())
    entry_summary = ", ".join(
        f"A{int(aid)}={_coord_label(entry_coord_map.get(int(aid)))}"
        for aid in aircraft_ids
    )
    emit(
        "[REEXEC-FIRST] current-position first mission builder armed: "
        f"sourcePlan={int(source_plan_id)}, inputMissionID={int(input_id)}, "
        f"sourceTemplateInputID={int(source_template_input_id) if source_template_input_id else int(input_id)}, "
        f"aircraft={aircraft_ids}, entries={entry_summary}"
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
        representative_entry=deepcopy(representative_entry)
        if isinstance(representative_entry, dict)
        else None,
        next_input_mission=deepcopy(next_input_mission)
        if isinstance(next_input_mission, dict)
        else None,
        turn_radius_scale=turn_radius_scale,
        source_template_input_id=source_template_input_id,
        log=_relay_helper_log,
    )
