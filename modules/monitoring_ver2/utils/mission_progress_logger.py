from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from modules.common import db_paths


def _safe_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


class MissionProgressExporter:
    """Writes mission/waypoint monitoring snapshots to DSS_Internal for operators."""

    def __init__(
        self,
        log_callback=None,
        base_dir: Optional[Path] = None,
    ) -> None:
        self._log = log_callback
        self._base_dir_provider: Callable[[], Path]
        if base_dir:
            explicit_dir = Path(base_dir)

            def _provider() -> Path:
                explicit_dir.mkdir(parents=True, exist_ok=True)
                return explicit_dir

            self._base_dir_provider = _provider
        else:

            def _provider() -> Path:
                dss_dir = db_paths.ensure_db_payload("DSS_Internal")
                target = dss_dir / "mission_progress"
                target.mkdir(parents=True, exist_ok=True)
                return target

            self._base_dir_provider = _provider

        self._current_base_dir: Optional[Path] = None
        self._active_key: Optional[Tuple[Optional[int], Optional[int], Optional[int]]] = None
        self._active_path: Optional[Path] = None

    def reset(self) -> None:
        """Forces the next write to start a brand-new file (e.g., on plan change)."""
        self._active_key = None
        self._active_path = None

    def write_snapshot(
        self,
        plan_context: Optional[Dict[str, Any]],
        mission_snapshots: List[Dict[str, Any]],
        *,
        timestamp_ms: Optional[int],
        mission_plan_id: Optional[int],
    ) -> None:
        if not plan_context:
            return

        key = self._make_key(plan_context, mission_plan_id)
        if key is None:
            return

        if self._active_key != key or self._active_path is None:
            self._active_path = self._create_new_file(key)
            self._active_key = key

        snapshot = self._build_payload(plan_context, mission_snapshots, timestamp_ms, mission_plan_id)
        try:
            with self._active_path.open("w", encoding="utf-8") as fh:
                json.dump(snapshot, fh, ensure_ascii=False, indent=2)
        except Exception as exc:
            if self._log:
                self._log(
                    "MISSION_PROGRESS",
                    "WARN",
                    f"Failed to write mission snapshot {self._active_path.name}: {exc}",
                )

    # ------------------------------------------------------------------ internal helpers

    def _make_key(
        self,
        plan_context: Dict[str, Any],
        mission_plan_id: Optional[int],
    ) -> Optional[Tuple[Optional[int], Optional[int], Optional[int]]]:
        plan_id = _safe_int(mission_plan_id) or _safe_int(plan_context.get("missionPlanID")) or 0
        input_pkg = _safe_int(plan_context.get("inputMissionPackageID")) or 0
        active_input = _safe_int(plan_context.get("activeInputMissionID")) or 0
        return (plan_id, input_pkg, active_input)

    def _create_new_file(
        self,
        key: Tuple[Optional[int], Optional[int], Optional[int]],
    ) -> Path:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
        plan_id, input_pkg, active_input = key
        file_name = f"mission_progress_{plan_id}_{input_pkg}_{active_input}_{stamp}.json"
        base_dir = self._resolve_base_dir()
        return base_dir / file_name

    def _build_payload(
        self,
        plan_context: Dict[str, Any],
        mission_snapshots: List[Dict[str, Any]],
        timestamp_ms: Optional[int],
        mission_plan_id: Optional[int],
    ) -> Dict[str, Any]:
        if timestamp_ms is None:
            timestamp_ms = int(
                (datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)).total_seconds()
                * 1000
            )

        snapshot_map: Dict[Tuple[int, int], Dict[str, Any]] = {}
        for entry in mission_snapshots:
            try:
                aircraft_id = int(entry.get("aircraftID"))
            except (TypeError, ValueError):
                continue
            mission_index = entry.get("missionIndex")
            if mission_index is None:
                continue
            try:
                mission_index = int(mission_index)
            except (TypeError, ValueError):
                continue
            snapshot_map[(aircraft_id, mission_index)] = entry

        aircraft_entries: List[Dict[str, Any]] = []
        for raw_aid, payload in sorted((plan_context.get("aircraft") or {}).items(), key=self._sort_aircraft):
            try:
                aircraft_id = int(raw_aid)
            except (TypeError, ValueError):
                aircraft_id = raw_aid
            missions_payload: List[Dict[str, Any]] = []
            missions = payload.get("missions") or []
            for mission_index, mission in enumerate(missions):
                mission_entry = snapshot_map.get((aircraft_id, mission_index))
                progress = mission_entry.get("progress") if mission_entry else None
                if progress is None:
                    progress = 100 if mission.get("isDone") else 0
                try:
                    progress = max(0, min(100, int(progress)))
                except (TypeError, ValueError):
                    progress = 0
                waypoints = mission.get("waypoints") or []
                waypoint_status = self._build_waypoint_status(waypoints, progress)
                missions_payload.append(
                    {
                        "missionIndex": mission_index,
                        "individualMissionID": mission.get("individualMissionID"),
                        "inputMissionID": mission.get("inputMissionID"),
                        "pathID": mission.get("pathID"),
                        "isDone": bool(mission.get("isDone")),
                        "progress": progress,
                        "waypoints": waypoint_status,
                    }
                )
            aircraft_entries.append(
                {
                    "aircraftID": aircraft_id,
                    "individualMissionPackageID": payload.get("individualMissionPackageID"),
                    "missions": missions_payload,
                }
            )

        return {
            "timestamp": timestamp_ms,
            "capturedAt": datetime.now(timezone.utc).isoformat(),
            "missionPlanID": mission_plan_id
            if mission_plan_id is not None
            else plan_context.get("missionPlanID"),
            "activeInputMissionID": plan_context.get("activeInputMissionID"),
            "inputMissionPackageID": plan_context.get("inputMissionPackageID"),
            "aircraft": aircraft_entries,
        }

    @staticmethod
    def _sort_aircraft(item):
        key, _ = item
        try:
            return int(key)
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def _build_waypoint_status(waypoints: List[Any], progress: int) -> List[Dict[str, Any]]:
        total = len(waypoints)
        if total <= 0:
            return []
        # visited count inferred from percentage
        visited = int(round(progress * total / 100.0))
        visited = max(0, min(total, visited))
        status = []
        for idx, waypoint_id in enumerate(waypoints):
            status.append(
                {
                    "order": idx,
                    "waypointID": waypoint_id,
                    "visited": idx < visited,
                }
            )
        return status

    def _resolve_base_dir(self) -> Path:
        base_dir = self._base_dir_provider()
        if self._current_base_dir is None or base_dir.resolve() != self._current_base_dir.resolve():
            self._current_base_dir = base_dir.resolve()
            self._active_path = None
        return base_dir
