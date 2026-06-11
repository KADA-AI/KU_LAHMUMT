from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional

from modules.mission_planning.engine.mission_generation.id_allocation import allocator as id_allocator


@dataclass
class ReservedIdBlock:
    name: str
    ids: list[int]
    _index: int = 0

    def next(self) -> int:
        if self._index >= len(self.ids):
            raise RuntimeError(f"reserved ID block exhausted: {self.name}")
        value = int(self.ids[self._index])
        self._index += 1
        return value

    @property
    def used(self) -> list[int]:
        return [int(value) for value in self.ids[: self._index]]

    @property
    def unused(self) -> list[int]:
        return [int(value) for value in self.ids[self._index :]]

    def summary(self) -> dict[str, Any]:
        used_ids = self.used
        unused_ids = self.unused
        if not self.ids:
            return {
                "count": 0,
                "used": 0,
                "unused": 0,
                "reservedStart": None,
                "reservedEnd": None,
                "usedStart": None,
                "usedEnd": None,
                "unusedStart": None,
                "unusedEnd": None,
            }
        return {
            "start": int(self.ids[0]),
            "end": int(self.ids[-1]),
            "reservedStart": int(self.ids[0]),
            "reservedEnd": int(self.ids[-1]),
            "count": len(self.ids),
            "used": len(used_ids),
            "unused": len(unused_ids),
            "usedStart": int(used_ids[0]) if used_ids else None,
            "usedEnd": int(used_ids[-1]) if used_ids else None,
            "unusedStart": int(unused_ids[0]) if unused_ids else None,
            "unusedEnd": int(unused_ids[-1]) if unused_ids else None,
        }


def summarize_used_reserved_ids(name: str, ids: list[Any]) -> dict[str, Any]:
    normalized = [int(value) for value in ids if value is not None]
    return ReservedIdBlock(str(name), normalized, _index=len(normalized)).summary()


@dataclass
class ReplanIdReservation:
    """Per-replan local ID allocator backed by global reserve APIs."""

    plan_ids: ReservedIdBlock = field(default_factory=lambda: ReservedIdBlock("missionPlanID", []))
    imp_ids: ReservedIdBlock = field(default_factory=lambda: ReservedIdBlock("individualMissionPackage", []))
    individual_ids: ReservedIdBlock = field(default_factory=lambda: ReservedIdBlock("individualMission", []))
    waypoint_ids: ReservedIdBlock = field(default_factory=lambda: ReservedIdBlock("waypoint", []))
    path_ids_by_aircraft: dict[int, ReservedIdBlock] = field(default_factory=dict)

    @classmethod
    def reserve(
        cls,
        *,
        plan_count: int = 0,
        imp_count: int = 0,
        individual_count: int = 0,
        path_count_by_aircraft: Optional[Mapping[int, int]] = None,
        waypoint_count: int = 0,
    ) -> "ReplanIdReservation":
        path_blocks: dict[int, ReservedIdBlock] = {}
        normalized_path_counts: dict[int, int] = {}
        for raw_aircraft_id, raw_count in dict(path_count_by_aircraft or {}).items():
            aircraft_id = int(raw_aircraft_id)
            count = int(raw_count or 0)
            if count <= 0:
                continue
            normalized_path_counts[aircraft_id] = normalized_path_counts.get(aircraft_id, 0) + count
        imp_ids: list[int] = []
        individual_ids: list[int] = []
        can_bundle = (
            int(plan_count or 0) <= 0
            and (
                bool(normalized_path_counts)
                or int(imp_count or 0) > 0
                or int(individual_count or 0) > 0
            )
            and hasattr(id_allocator, "reserve_replan_id_bundle")
        )
        if can_bundle:
            bundle = id_allocator.reserve_replan_id_bundle(
                path_count_by_aircraft=normalized_path_counts,
                imp_count=int(imp_count or 0),
                individual_mission_count=int(individual_count or 0),
                waypoint_count=int(waypoint_count or 0),
            )
            bundled_paths = dict(bundle.get("pathID") or {}) if isinstance(bundle, dict) else {}
            for raw_aircraft_id, values in bundled_paths.items():
                aircraft_id = int(raw_aircraft_id)
                path_blocks[aircraft_id] = ReservedIdBlock(
                    f"pathID[{aircraft_id}]",
                    [int(value) for value in values],
                )
            imp_ids = [
                int(value)
                for value in (bundle.get("individualMissionPackage") or [])
            ] if isinstance(bundle, dict) else []
            individual_ids = [
                int(value)
                for value in (bundle.get("individualMission") or [])
            ] if isinstance(bundle, dict) else []
            waypoint_ids = [
                int(value)
                for value in (bundle.get("waypoint") or [])
            ] if isinstance(bundle, dict) else []
        else:
            for aircraft_id, count in sorted(normalized_path_counts.items()):
                path_blocks[aircraft_id] = ReservedIdBlock(
                    f"pathID[{aircraft_id}]",
                    [int(value) for value in id_allocator.reserve_path_ids(aircraft_id, count)],
                )
        if not can_bundle:
            waypoint_ids = []
        if int(waypoint_count or 0) > 0 and not waypoint_ids:
            start = int(id_allocator.reserve_waypoint_block(int(waypoint_count)))
            waypoint_ids = list(range(start, start + int(waypoint_count)))
        return cls(
            plan_ids=ReservedIdBlock(
                "missionPlanID",
                [int(value) for value in id_allocator.reserve_mission_plan_ids(int(plan_count))]
                if int(plan_count or 0) > 0
                else [],
            ),
            imp_ids=ReservedIdBlock(
                "individualMissionPackage",
                imp_ids
                if can_bundle
                else (
                    [int(value) for value in id_allocator.reserve_imp_ids(int(imp_count))]
                    if int(imp_count or 0) > 0
                    else []
                ),
            ),
            individual_ids=ReservedIdBlock(
                "individualMission",
                individual_ids
                if can_bundle
                else (
                    [int(value) for value in id_allocator.reserve_individual_mission_ids(int(individual_count))]
                    if int(individual_count or 0) > 0
                    else []
                ),
            ),
            waypoint_ids=ReservedIdBlock("waypoint", waypoint_ids),
            path_ids_by_aircraft=path_blocks,
        )

    def next_plan(self) -> int:
        return self.plan_ids.next()

    def next_imp(self) -> int:
        return self.imp_ids.next()

    def next_individual(self) -> int:
        return self.individual_ids.next()

    def next_path(self, aircraft_id: int) -> int:
        block = self.path_ids_by_aircraft.get(int(aircraft_id))
        if block is None:
            raise RuntimeError(f"no reserved pathID block for aircraftID={int(aircraft_id)}")
        return block.next()

    def next_waypoint(self) -> int:
        return self.waypoint_ids.next()

    def summary(self) -> dict[str, Any]:
        return {
            "missionPlanID": self.plan_ids.summary(),
            "individualMissionPackage": self.imp_ids.summary(),
            "individualMission": self.individual_ids.summary(),
            "waypoint": self.waypoint_ids.summary(),
            "pathID": {
                int(aircraft_id): block.summary()
                for aircraft_id, block in sorted(self.path_ids_by_aircraft.items())
            },
        }
