from __future__ import annotations

import json
import math
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt
from PyQt5.QtGui import (
    QColor,
    QBrush,
    QCursor,
    QFont,
    QFontMetrics,
    QPainter,
    QPainterPath,
    QPainterPathStroker,
    QPen,
    QTransform,
)
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDialog,
    QFrame,
    QGraphicsItem,
    QGraphicsObject,
    QGraphicsPathItem,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from modules.common import db_paths


def _as_int(value: Any) -> Optional[int]:
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    if value is None:
        return None
    try:
        text = str(value).strip()
        if not text:
            return None
        return int(text, 10)
    except (TypeError, ValueError):
        return None


def _as_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int_list(*values: Any) -> List[int]:
    items: List[int] = []
    seen: Set[int] = set()
    for value in values:
        if isinstance(value, (list, tuple, set)):
            for inner in value:
                parsed = _as_int(inner)
                if parsed is None or parsed in seen:
                    continue
                seen.add(parsed)
                items.append(parsed)
            continue
        parsed = _as_int(value)
        if parsed is None or parsed in seen:
            continue
        seen.add(parsed)
        items.append(parsed)
    return items


def _coalesce_waypoints(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    items = payload.get("waypointList") or payload.get("lahWaypointList") or []
    return [item for item in items if isinstance(item, dict)]


def _safe_json_text(payload: Any) -> str:
    try:
        return json.dumps(payload, ensure_ascii=False, indent=2)
    except Exception:
        return str(payload)


def _pretty_time(value: Any) -> str:
    parsed = _as_int(value)
    if parsed is None:
        return "-"
    try:
        return datetime.fromtimestamp((946_684_800_000 + parsed) / 1000.0).strftime("%Y-%m-%d %H:%M:%S")
    except Exception:
        return str(parsed)


def _node_sort_key(node_id: str) -> Tuple[int, str]:
    digits = "".join(ch for ch in node_id if ch.isdigit())
    return (int(digits) if digits else 0, node_id)


@dataclass
class ReplanLink:
    kind: str
    label: str
    path: Path
    payload: Dict[str, Any]
    source_plan_id: Optional[int]
    generated_plan_id: Optional[int]
    source_input_package_ids: List[int] = field(default_factory=list)
    generated_input_package_ids: List[int] = field(default_factory=list)
    source_individual_package_ids: List[int] = field(default_factory=list)
    generated_individual_package_ids: List[int] = field(default_factory=list)
    source_individual_ids: List[int] = field(default_factory=list)
    generated_individual_ids: List[int] = field(default_factory=list)
    source_path_ids: List[int] = field(default_factory=list)
    generated_path_ids: List[int] = field(default_factory=list)


@dataclass
class GraphNode:
    node_id: str
    entity_type: str
    display_role: str
    depth: float
    title: str
    subtitle: str
    detail_lines: List[str]
    payload: Any
    file_path: Optional[Path] = None
    expandable: bool = False
    expanded: bool = False


@dataclass
class GraphEdge:
    source_id: str
    target_id: str
    label: str
    kind: str
    color: str
    dashed: bool = False


class RelationshipCache:
    """Loads mission-planning artifacts and derives reusable ID relationships."""

    def __init__(self) -> None:
        self.db_root = Path(db_paths.bootstrap_db_root())
        self.packages: Dict[int, Dict[str, Any]] = {}
        self.mission_plans: Dict[int, Dict[str, Any]] = {}
        self.individual_packages: Dict[int, Dict[str, Any]] = {}
        self.flight_paths: Dict[int, Dict[str, Any]] = {}
        self.package_to_plans: Dict[int, List[int]] = defaultdict(list)
        self.plan_to_individual_packages: Dict[int, List[Dict[str, Optional[int]]]] = defaultdict(list)
        self.individual_package_to_plans: Dict[int, Set[int]] = defaultdict(set)
        self.individual_entries: Dict[str, Dict[str, Any]] = {}
        self.individual_entry_keys_by_id: Dict[int, List[str]] = defaultdict(list)
        self.path_entries: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
        self.plan_to_individuals: Dict[int, List[str]] = defaultdict(list)
        self.input_to_individuals: Dict[Tuple[int, int], List[Dict[str, Any]]] = defaultdict(list)
        self.replan_links: List[ReplanLink] = []
        self.plan_parent_links: Dict[int, List[ReplanLink]] = defaultdict(list)
        self.plan_child_links: Dict[int, List[ReplanLink]] = defaultdict(list)
        self.individual_parent_links: Dict[str, List[ReplanLink]] = defaultdict(list)
        self.individual_child_links: Dict[str, List[ReplanLink]] = defaultdict(list)
        self.path_parent_links: Dict[int, List[ReplanLink]] = defaultdict(list)
        self.path_child_links: Dict[int, List[ReplanLink]] = defaultdict(list)
        self.errors: List[str] = []

    def refresh(self) -> None:
        self.db_root = Path(db_paths.bootstrap_db_root())
        self.packages.clear()
        self.mission_plans.clear()
        self.individual_packages.clear()
        self.flight_paths.clear()
        self.package_to_plans.clear()
        self.plan_to_individual_packages.clear()
        self.individual_package_to_plans.clear()
        self.individual_entries.clear()
        self.individual_entry_keys_by_id.clear()
        self.path_entries.clear()
        self.plan_to_individuals.clear()
        self.input_to_individuals.clear()
        self.replan_links.clear()
        self.plan_parent_links.clear()
        self.plan_child_links.clear()
        self.individual_parent_links.clear()
        self.individual_child_links.clear()
        self.path_parent_links.clear()
        self.path_child_links.clear()
        self.errors.clear()

        self._load_packages()
        self._load_mission_plans()
        self._load_individual_packages()
        self._load_flight_paths()
        self._load_replan_logs()

    def get_plan_ids(self) -> List[int]:
        return sorted(self.mission_plans.keys())

    def get_individual_packages_for_plan(self, plan_id: int) -> List[Tuple[int, Optional[int]]]:
        items: List[Tuple[int, Optional[int]]] = []
        for entry in self.plan_to_individual_packages.get(plan_id, []):
            imp_id = entry.get("package_id")
            if imp_id is None:
                continue
            items.append((imp_id, entry.get("aircraft_id")))
        return sorted(items, key=lambda item: item[0])

    def get_individual_entries_for_plan(
        self,
        plan_id: int,
        *,
        individual_package_id: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        entries: List[Dict[str, Any]] = []
        for entry_key in self.plan_to_individuals.get(plan_id, []):
            entry = self.individual_entries.get(entry_key)
            if not entry:
                continue
            if individual_package_id is not None and entry.get("package_id") != individual_package_id:
                continue
            entries.append(entry)
        entries.sort(
            key=lambda entry: (
                entry.get("package_id") or 0,
                entry.get("input_mission_id") or 0,
                entry.get("individual_id") or 0,
            )
        )
        return entries

    def get_plan_summary(self, plan_id: int) -> str:
        plan = self.mission_plans.get(plan_id) or {}
        pkg_id = _as_int(plan.get("package_id"))
        ref_id = _as_int((plan.get("data") or {}).get("missionReferencePackageID"))
        aircraft_count = len(plan.get("aircraft_map") or [])
        parts = [f"InputPkg {pkg_id if pkg_id is not None else '-'}", f"Aircraft {aircraft_count}"]
        if ref_id:
            parts.append(f"RefPkg {ref_id}")
        return " | ".join(parts)

    def _load_packages(self) -> None:
        input_dir = self.db_root / "InputMissionPlan"
        if not input_dir.exists():
            return
        for path in sorted(input_dir.glob("*.json")):
            payloads = self._iter_payload_dicts(
                path,
                self._safe_load(path, "[InputMissionPlan]"),
                "[InputMissionPlan]",
            )
            for payload in payloads:
                package_id = _as_int(payload.get("inputMissionPackageID"))
                if package_id is None:
                    self.errors.append(f"[InputMissionPlan] {path.name}: missing inputMissionPackageID")
                    continue

                missions: Dict[int, Dict[str, Any]] = {}
                for mission in payload.get("inputMissionList") or []:
                    if not isinstance(mission, dict):
                        continue
                    mission_id = _as_int(mission.get("inputMissionID"))
                    if mission_id is None:
                        continue
                    missions[mission_id] = mission

                existing = self.packages.get(package_id)
                if existing and existing.get("missions") and not missions:
                    continue
                self.packages[package_id] = {
                    "path": path,
                    "data": payload,
                    "missions": missions,
                    "timestamp": _as_int(payload.get("timestamp")),
                }

    def _load_mission_plans(self) -> None:
        plan_dir = self.db_root / "MissionPlan"
        if not plan_dir.exists():
            return

        for path in sorted(plan_dir.glob("*.json")):
            payloads = self._iter_payload_dicts(
                path,
                self._safe_load(path, "[MissionPlan]"),
                "[MissionPlan]",
            )
            for payload in payloads:
                plan_id = _as_int(payload.get("missionPlanID"))
                if plan_id is None:
                    self.errors.append(f"[MissionPlan] {path.name}: missing missionPlanID")
                    continue

                package_id = _as_int(payload.get("inputMissionPackageID"))
                if package_id is not None:
                    self.package_to_plans[package_id].append(plan_id)

                aircraft_map: List[Dict[str, Optional[int]]] = []
                seen_packages: Set[int] = set()
                for aircraft in payload.get("aircraftList") or []:
                    if not isinstance(aircraft, dict):
                        continue
                    imp_id = _as_int(
                        aircraft.get("individualMissionPackageID")
                        or aircraft.get("individualMissionPlanPackageID")
                    )
                    aircraft_id = _as_int(aircraft.get("aircraftID"))
                    if imp_id is None or imp_id in seen_packages:
                        continue
                    seen_packages.add(imp_id)
                    aircraft_map.append({"package_id": imp_id, "aircraft_id": aircraft_id})
                    self.individual_package_to_plans[imp_id].add(plan_id)

                self.plan_to_individual_packages[plan_id] = aircraft_map
                self.mission_plans[plan_id] = {
                    "path": path,
                    "data": payload,
                    "package_id": package_id,
                    "aircraft_map": aircraft_map,
                    "mission_reference_package_id": _as_int(payload.get("missionReferencePackageID")),
                }

        for package_id, plan_ids in self.package_to_plans.items():
            self.package_to_plans[package_id] = sorted(set(plan_ids))

    def _load_individual_packages(self) -> None:
        imp_dir = self.db_root / "IndividualMissionPlan"
        if not imp_dir.exists():
            return

        for path in sorted(imp_dir.glob("*.json")):
            payloads = self._iter_payload_dicts(
                path,
                self._safe_load(path, "[IndividualMissionPlan]"),
                "[IndividualMissionPlan]",
            )
            for payload in payloads:
                imp_id = _as_int(payload.get("individualMissionPackageID"))
                if imp_id is None:
                    self.errors.append(f"[IndividualMissionPlan] {path.name}: missing individualMissionPackageID")
                    continue

                aircraft_id = _as_int(payload.get("aircraftID"))
                plan_ids = sorted(self.individual_package_to_plans.get(imp_id, set()))
                missions: Dict[int, Dict[str, Any]] = {}
                entries: List[Dict[str, Any]] = []

                for mission in payload.get("individualMissionList") or []:
                    if not isinstance(mission, dict):
                        continue
                    individual_id = _as_int(mission.get("individualMissionID"))
                    if individual_id is None:
                        continue
                    path_id = _as_int(mission.get("pathID"))
                    related = mission.get("relatedMission")
                    if not isinstance(related, dict):
                        related = {}
                    input_id = _as_int(related.get("inputMissionID"))
                    prior_id = _as_int(related.get("priorMissionID"))
                    related_type = _as_int(related.get("relatedMissionType"))

                    entry = {
                        "entry_key": self._entry_key(imp_id, individual_id),
                        "individual_id": individual_id,
                        "input_mission_id": input_id,
                        "related_mission_type": related_type,
                        "prior_mission_id": prior_id,
                        "package_id": imp_id,
                        "aircraft_id": aircraft_id,
                        "path_id": path_id,
                        "plan_ids": list(plan_ids),
                        "raw": mission,
                        "file_path": path,
                    }
                    entries.append(entry)
                    missions[individual_id] = mission
                    entry_key = str(entry["entry_key"])
                    self.individual_entries[entry_key] = entry
                    self.individual_entry_keys_by_id[individual_id].append(entry_key)
                    for plan_id in plan_ids:
                        self.plan_to_individuals[plan_id].append(entry_key)
                    if path_id is not None:
                        self.path_entries[path_id].append(entry)
                    if input_id is not None:
                        for plan_id in plan_ids:
                            package_id = _as_int((self.mission_plans.get(plan_id) or {}).get("package_id"))
                            if package_id is not None:
                                self.input_to_individuals[(package_id, input_id)].append(entry)

                self.individual_packages[imp_id] = {
                    "path": path,
                    "data": payload,
                    "aircraft_id": aircraft_id,
                    "plan_ids": plan_ids,
                    "missions": missions,
                    "entries": entries,
                }

        for plan_id in list(self.plan_to_individuals.keys()):
            deduped = sorted(set(self.plan_to_individuals[plan_id]))
            self.plan_to_individuals[plan_id] = deduped

    def _load_flight_paths(self) -> None:
        path_dir = self.db_root / "FlightPath"
        if not path_dir.exists():
            return
        for path in sorted(path_dir.glob("*.json")):
            payloads = self._iter_payload_dicts(
                path,
                self._safe_load(path, "[FlightPath]"),
                "[FlightPath]",
            )
            for payload in payloads:
                path_id = _as_int(payload.get("pathID"))
                if path_id is None:
                    self.errors.append(f"[FlightPath] {path.name}: missing pathID")
                    continue
                self.flight_paths[path_id] = {
                    "path": path,
                    "data": payload,
                    "aircraft_id": _as_int(payload.get("aircraftID")),
                    "waypoint_count": len(_coalesce_waypoints(payload)),
                }

    def _load_replan_logs(self) -> None:
        log_dir = self.db_root / "DSS_Internal"
        if not log_dir.exists():
            return
        for path in sorted(log_dir.glob("*.json")):
            payloads = self._iter_payload_dicts(
                path,
                self._safe_load(path, "[DSS_Internal]"),
                "[DSS_Internal]",
            )
            for payload in payloads:
                generated_plan_id = _as_int(payload.get("generatedMissionPlanID"))
                source_plan_id = _as_int(payload.get("sourceMissionPlanID"))
                if generated_plan_id is None and source_plan_id is None:
                    continue

                stem = path.stem.lower()
                kind, label = self._classify_replan_log(stem, payload)
                link = ReplanLink(
                    kind=kind,
                    label=label,
                    path=path,
                    payload=payload,
                    source_plan_id=source_plan_id,
                    generated_plan_id=generated_plan_id,
                    source_input_package_ids=_as_int_list(
                        payload.get("sourceInputMissionPackageID"),
                        payload.get("sourceInputMissionPackageIDs"),
                    ),
                    generated_input_package_ids=_as_int_list(
                        payload.get("generatedInputMissionPackageID"),
                        payload.get("generatedInputMissionPackageIDs"),
                    ),
                    source_individual_package_ids=_as_int_list(
                        payload.get("sourceIndividualMissionPackageID"),
                        payload.get("sourceIndividualMissionPackageIDs"),
                    ),
                    generated_individual_package_ids=_as_int_list(
                        payload.get("generatedIndividualMissionPackageID"),
                        payload.get("generatedIndividualMissionPackageIDs"),
                    ),
                    source_individual_ids=_as_int_list(
                        payload.get("sourceIndividualMissionID"),
                        payload.get("sourceIndividualMissionIDs"),
                    ),
                    generated_individual_ids=_as_int_list(
                        payload.get("generatedIndividualMissionID"),
                        payload.get("generatedPriorIndividualMissionID"),
                        payload.get("generatedResumeIndividualMissionID"),
                        payload.get("generatedIndividualMissionIDs"),
                    ),
                    source_path_ids=_as_int_list(
                        payload.get("sourcePathID"),
                        payload.get("sourcePathIDs"),
                    ),
                    generated_path_ids=_as_int_list(
                        payload.get("generatedPathID"),
                        payload.get("generatedDonePathID"),
                        payload.get("generatedPriorPathID"),
                        payload.get("generatedResumePathID"),
                        payload.get("generatedPathIDs"),
                    ),
                )
                self.replan_links.append(link)
                if generated_plan_id is not None and source_plan_id is not None:
                    self.plan_parent_links[generated_plan_id].append(link)
                    self.plan_child_links[source_plan_id].append(link)
                for generated_individual_id in link.generated_individual_ids:
                    for entry_key in self.individual_entry_keys_by_id.get(generated_individual_id, []):
                        self.individual_parent_links[entry_key].append(link)
                for source_individual_id in link.source_individual_ids:
                    for entry_key in self.individual_entry_keys_by_id.get(source_individual_id, []):
                        self.individual_child_links[entry_key].append(link)
                for generated_path_id in link.generated_path_ids:
                    self.path_parent_links[generated_path_id].append(link)
                for source_path_id in link.source_path_ids:
                    self.path_child_links[source_path_id].append(link)

    def _classify_replan_log(self, stem: str, payload: Dict[str, Any]) -> Tuple[str, str]:
        if stem.startswith("priormission"):
            return "prior", "선행"
        if stem.startswith("pathdeviation"):
            return "pathdev", "경로이탈"
        if stem.startswith("imagingschedule"):
            return "imaging", "촬영재계획"
        if stem.startswith("qualityspeed"):
            return "quality", "품질속도"
        if stem.startswith("nextcollab"):
            return "nextcollab", "다음영역"
        if "triggerType" in payload:
            return "replan", str(payload.get("triggerType") or "재계획")
        return "replan", "재계획"

    def _safe_load(self, path: Path, prefix: str) -> Optional[Any]:
        try:
            with path.open("r", encoding="utf-8") as handle:
                return json.load(handle)
        except Exception as exc:
            self.errors.append(f"{prefix} {path.name}: {exc}")
            return None

    def _iter_payload_dicts(self, path: Path, payload: Any, prefix: str) -> List[Dict[str, Any]]:
        if payload is None:
            return []
        if isinstance(payload, dict):
            return [payload]
        if isinstance(payload, list):
            rows = [item for item in payload if isinstance(item, dict)]
            skipped = len(payload) - len(rows)
            if skipped:
                self.errors.append(f"{prefix} {path.name}: skipped {skipped} non-object item(s)")
            if not rows:
                self.errors.append(f"{prefix} {path.name}: expected object or object list")
            return rows
        self.errors.append(
            f"{prefix} {path.name}: expected object or object list, got {type(payload).__name__}"
        )
        return []

    @staticmethod
    def _entry_key(individual_package_id: int, individual_id: int) -> str:
        return f"{int(individual_package_id)}:{int(individual_id)}"


class PlanGraphBuilder:
    """Builds a mission-plan-centric graph with lazy expansion."""

    def __init__(self, cache: RelationshipCache, plan_id: int, expanded_nodes: Set[str]) -> None:
        self.cache = cache
        self.plan_id = int(plan_id)
        self.expanded_nodes = set(expanded_nodes)
        self.nodes: Dict[str, GraphNode] = {}
        self.edges: List[GraphEdge] = []

    def build(self) -> Tuple[Dict[str, GraphNode], List[GraphEdge]]:
        if self.plan_id not in self.cache.mission_plans:
            return {}, []
        plan_node_id = self._plan_node_id(self.plan_id)
        self._add_plan_node(self.plan_id, display_role="mission_plan", depth=0.0)
        self._add_plan_context(self.plan_id, plan_node_id)
        return self.nodes, self.edges

    def _add_plan_context(self, plan_id: int, plan_node_id: str) -> None:
        plan = self.cache.mission_plans.get(plan_id) or {}
        package_id = _as_int(plan.get("package_id"))
        ref_package_id = _as_int(plan.get("mission_reference_package_id"))

        if package_id is not None:
            pkg_node_id = self._package_node_id(package_id, "input_package")
            self._add_package_node(package_id, display_role="input_package", depth=-1.0)
            self._add_edge(pkg_node_id, plan_node_id, "입력", "input", "#3566d6")
            if self._is_expanded(pkg_node_id):
                self._expand_input_package(package_id, pkg_node_id)

        if ref_package_id and ref_package_id != package_id:
            ref_node_id = self._package_node_id(ref_package_id, "reference_package")
            self._add_package_node(ref_package_id, display_role="reference_package", depth=-1.0)
            self._add_edge(ref_node_id, plan_node_id, "참조", "reference", "#8b5cf6", dashed=True)
            if self._is_expanded(ref_node_id):
                self._expand_input_package(ref_package_id, ref_node_id)

        for link in self.cache.plan_parent_links.get(plan_id, []):
            if link.source_plan_id is None:
                continue
            source_plan_node_id = self._plan_node_id(link.source_plan_id)
            self._add_plan_node(link.source_plan_id, display_role="source_plan", depth=-1.8)
            self._add_edge(source_plan_node_id, plan_node_id, link.label, link.kind, "#e67e22", dashed=True)

        for link in self.cache.plan_child_links.get(plan_id, []):
            if link.generated_plan_id is None:
                continue
            child_plan_node_id = self._plan_node_id(link.generated_plan_id)
            self._add_plan_node(link.generated_plan_id, display_role="child_plan", depth=0.9)
            self._add_edge(plan_node_id, child_plan_node_id, link.label, link.kind, "#d35454", dashed=True)

        for imp_id, aircraft_id in self.cache.get_individual_packages_for_plan(plan_id):
            imp_node_id = self._imp_node_id(imp_id)
            self._add_individual_package_node(imp_id, aircraft_id, depth=1.4)
            self._add_edge(plan_node_id, imp_node_id, f"A/C {aircraft_id or '-'}", "assignment", "#2f855a")
            if self._is_expanded(plan_node_id) or self._is_expanded(imp_node_id):
                self._expand_individual_package(plan_id, imp_id, imp_node_id)

    def _expand_input_package(self, package_id: int, package_node_id: str) -> None:
        package = self.cache.packages.get(package_id) or {}
        missions = package.get("missions") or {}
        for input_mission_id, mission in sorted(missions.items()):
            node_id = self._input_mission_node_id(package_id, input_mission_id)
            self._add_input_mission_node(package_id, input_mission_id, mission, depth=-2.2)
            self._add_edge(node_id, package_node_id, "소속", "contain", "#8aa2cc")

    def _expand_individual_package(self, plan_id: int, imp_id: int, imp_node_id: str) -> None:
        for entry in self.cache.get_individual_entries_for_plan(plan_id, individual_package_id=imp_id):
            entry_key = str(entry.get("entry_key") or "")
            if not entry_key:
                continue
            node_id = self._individual_node_id(entry_key)
            self._add_individual_mission_node(entry, display_role="individual_mission", depth=2.4)
            self._add_edge(imp_node_id, node_id, "분기", "individual", "#16a085")
            if self._is_expanded(node_id):
                self._expand_individual(entry, node_id)

    def _expand_individual(self, entry: Dict[str, Any], individual_node_id: str) -> None:
        input_mission_id = _as_int(entry.get("input_mission_id"))
        entry_key = str(entry.get("entry_key") or "")
        path_id = _as_int(entry.get("path_id"))
        prior_mission_id = _as_int(entry.get("prior_mission_id"))
        related_type = _as_int(entry.get("related_mission_type"))

        if input_mission_id is not None:
            for plan_id in entry.get("plan_ids") or []:
                package_id = _as_int((self.cache.mission_plans.get(plan_id) or {}).get("package_id"))
                if package_id is None:
                    continue
                mission = ((self.cache.packages.get(package_id) or {}).get("missions") or {}).get(input_mission_id)
                if mission is None:
                    continue
                input_node_id = self._input_mission_node_id(package_id, input_mission_id)
                self._add_package_node(package_id, display_role="input_package", depth=-1.0)
                self._add_input_mission_node(package_id, input_mission_id, mission, depth=-2.2)
                self._add_edge(input_node_id, individual_node_id, "입력 매핑", "mapping", "#4c6edb")
                break

        if prior_mission_id:
            prior_node_id = self._prior_node_id(prior_mission_id)
            detail_lines = [
                f"PriorMissionID: {prior_mission_id}",
                f"RelatedMissionType: {related_type if related_type is not None else '-'}",
            ]
            self._add_node(
                GraphNode(
                    node_id=prior_node_id,
                    entity_type="prior_reference",
                    display_role="prior_reference",
                    depth=1.6,
                    title=f"Prior {prior_mission_id}",
                    subtitle="선행 임무 참조",
                    detail_lines=detail_lines,
                    payload={"priorMissionID": prior_mission_id, "relatedMissionType": related_type},
                    expandable=False,
                    expanded=False,
                )
            )
            self._add_edge(prior_node_id, individual_node_id, "prior", "prior", "#c0392b", dashed=True)

        if path_id is not None:
            path_node_id = self._path_node_id(path_id)
            self._add_path_node(path_id, display_role="path", depth=3.3)
            self._add_edge(individual_node_id, path_node_id, "path", "path", "#b7791f")
            if self._is_expanded(path_node_id):
                self._expand_path(path_id, path_node_id)

        if entry_key:
            for link in self.cache.individual_parent_links.get(entry_key, []):
                for source_individual_id in link.source_individual_ids:
                    for source_entry_key in self.cache.individual_entry_keys_by_id.get(source_individual_id, []):
                        source_entry = self.cache.individual_entries.get(source_entry_key)
                        if not source_entry:
                            continue
                        source_node_id = self._individual_node_id(source_entry_key)
                        self._add_individual_mission_node(source_entry, display_role="source_individual", depth=1.9)
                        self._add_edge(source_node_id, individual_node_id, link.label, link.kind, "#f08c00", dashed=True)

            for link in self.cache.individual_child_links.get(entry_key, []):
                for generated_individual_id in link.generated_individual_ids:
                    for child_entry_key in self.cache.individual_entry_keys_by_id.get(generated_individual_id, []):
                        child_entry = self.cache.individual_entries.get(child_entry_key)
                        if not child_entry:
                            continue
                        child_node_id = self._individual_node_id(child_entry_key)
                        self._add_individual_mission_node(child_entry, display_role="child_individual", depth=2.9)
                        self._add_edge(individual_node_id, child_node_id, link.label, link.kind, "#dd6b20", dashed=True)

    def _expand_path(self, path_id: int, path_node_id: str) -> None:
        path_payload = (self.cache.flight_paths.get(path_id) or {}).get("data") or {}
        for index, waypoint in enumerate(_coalesce_waypoints(path_payload), start=1):
            waypoint_id = _as_int(waypoint.get("waypointID")) or index
            node_id = self._waypoint_node_id(path_id, waypoint_id, index)
            self._add_waypoint_node(path_id, index, waypoint, depth=4.4)
            self._add_edge(path_node_id, node_id, "WP", "waypoint", "#7b8794")

        for link in self.cache.path_parent_links.get(path_id, []):
            for source_path_id in link.source_path_ids:
                if source_path_id == path_id:
                    continue
                source_node_id = self._path_node_id(source_path_id)
                self._add_path_node(source_path_id, display_role="source_path", depth=2.8)
                self._add_edge(source_node_id, path_node_id, link.label, link.kind, "#d97706", dashed=True)

        for link in self.cache.path_child_links.get(path_id, []):
            for generated_path_id in link.generated_path_ids:
                if generated_path_id == path_id:
                    continue
                child_node_id = self._path_node_id(generated_path_id)
                self._add_path_node(generated_path_id, display_role="child_path", depth=3.8)
                self._add_edge(path_node_id, child_node_id, link.label, link.kind, "#ea580c", dashed=True)

    def _add_plan_node(self, plan_id: int, *, display_role: str, depth: float) -> None:
        plan = self.cache.mission_plans.get(plan_id)
        if not plan:
            return
        payload = plan.get("data") or {}
        package_id = _as_int(plan.get("package_id"))
        ref_package_id = _as_int(plan.get("mission_reference_package_id"))
        detail_lines = [
            f"MissionPlanID: {plan_id}",
            f"InputMissionPackageID: {package_id if package_id is not None else '-'}",
            f"MissionReferencePackageID: {ref_package_id if ref_package_id is not None else '-'}",
            f"AircraftCount: {len(plan.get('aircraft_map') or [])}",
            f"Timestamp: {_pretty_time(payload.get('timestamp'))}",
        ]
        node_id = self._plan_node_id(plan_id)
        self._add_node(
            GraphNode(
                node_id=node_id,
                entity_type="mission_plan",
                display_role=display_role,
                depth=depth,
                title=f"Plan {plan_id}",
                subtitle=self.cache.get_plan_summary(plan_id),
                detail_lines=detail_lines,
                payload=payload,
                file_path=plan.get("path"),
                expandable=(display_role == "mission_plan"),
                expanded=self._is_expanded(node_id),
            )
        )

    def _add_package_node(self, package_id: int, *, display_role: str, depth: float) -> None:
        package = self.cache.packages.get(package_id)
        if not package:
            return
        payload = package.get("data") or {}
        missions = package.get("missions") or {}
        label = "참조 패키지" if display_role == "reference_package" else "입력 패키지"
        node_id = self._package_node_id(package_id, display_role)
        self._add_node(
            GraphNode(
                node_id=node_id,
                entity_type="input_package",
                display_role=display_role,
                depth=depth,
                title=f"Pkg {package_id}",
                subtitle=f"{label} · missions {len(missions)}",
                detail_lines=[
                    f"InputMissionPackageID: {package_id}",
                    f"MissionCount: {len(missions)}",
                    f"Timestamp: {_pretty_time(payload.get('timestamp'))}",
                ],
                payload=payload,
                file_path=package.get("path"),
                expandable=bool(missions),
                expanded=self._is_expanded(node_id),
            )
        )

    def _add_input_mission_node(
        self,
        package_id: int,
        input_mission_id: int,
        mission: Dict[str, Any],
        *,
        depth: float,
    ) -> None:
        detail = mission.get("missionDetail") or {}
        line_count = len(detail.get("lineList") or [])
        area_count = len(detail.get("areaList") or [])
        subtitle = f"type {mission.get('inputMissionType') or '-'} · line {line_count} · area {area_count}"
        self._add_node(
            GraphNode(
                node_id=self._input_mission_node_id(package_id, input_mission_id),
                entity_type="input_mission",
                display_role="input_mission",
                depth=depth,
                title=f"Input {input_mission_id}",
                subtitle=subtitle,
                detail_lines=[
                    f"InputMissionID: {input_mission_id}",
                    f"InputMissionPackageID: {package_id}",
                    f"InputMissionType: {mission.get('inputMissionType')}",
                    f"LineCount: {line_count}",
                    f"AreaCount: {area_count}",
                ],
                payload=mission,
                file_path=(self.cache.packages.get(package_id) or {}).get("path"),
                expandable=False,
                expanded=False,
            )
        )

    def _add_individual_package_node(
        self,
        imp_id: int,
        aircraft_id: Optional[int],
        *,
        depth: float,
    ) -> None:
        imp = self.cache.individual_packages.get(imp_id)
        if not imp:
            return
        entries = imp.get("entries") or []
        node_id = self._imp_node_id(imp_id)
        self._add_node(
            GraphNode(
                node_id=node_id,
                entity_type="individual_package",
                display_role="individual_package",
                depth=depth,
                title=f"IndPkg {imp_id}",
                subtitle=f"A/C {aircraft_id if aircraft_id is not None else '-'} · missions {len(entries)}",
                detail_lines=[
                    f"IndividualMissionPackageID: {imp_id}",
                    f"AircraftID: {aircraft_id if aircraft_id is not None else '-'}",
                    f"MissionCount: {len(entries)}",
                    f"Plans: {', '.join(str(v) for v in imp.get('plan_ids') or []) or '-'}",
                ],
                payload=imp.get("data"),
                file_path=imp.get("path"),
                expandable=bool(entries),
                expanded=self._is_expanded(node_id),
            )
        )

    def _add_individual_mission_node(
        self,
        entry: Dict[str, Any],
        *,
        display_role: str,
        depth: float,
    ) -> None:
        individual_id = _as_int(entry.get("individual_id"))
        imp_id = _as_int(entry.get("package_id"))
        entry_key = str(entry.get("entry_key") or "")
        if individual_id is None or imp_id is None or not entry_key:
            return
        input_id = _as_int(entry.get("input_mission_id"))
        path_id = _as_int(entry.get("path_id"))
        prior_id = _as_int(entry.get("prior_mission_id"))
        related_type = _as_int(entry.get("related_mission_type"))
        detail_lines = [
            f"IndividualMissionKey: {entry_key}",
            f"IndividualMissionID: {individual_id}",
            f"IndividualMissionPackageID: {imp_id}",
            f"AircraftID: {entry.get('aircraft_id') or '-'}",
            f"PathID: {path_id if path_id is not None else '-'}",
            f"InputMissionID: {input_id if input_id is not None else '-'}",
            f"PriorMissionID: {prior_id if prior_id is not None else '-'}",
            f"RelatedMissionType: {related_type if related_type is not None else '-'}",
            f"Plans: {', '.join(str(v) for v in entry.get('plan_ids') or []) or '-'}",
        ]
        hidden_relations = bool(
            path_id is not None
            or input_id is not None
            or prior_id
            or self.cache.individual_parent_links.get(entry_key)
            or self.cache.individual_child_links.get(entry_key)
        )
        self._add_node(
            GraphNode(
                node_id=self._individual_node_id(entry_key),
                entity_type="individual_mission",
                display_role=display_role,
                depth=depth,
                title=f"IM {individual_id}",
                subtitle=f"IMP {imp_id} · input {input_id if input_id is not None else '-'} · path {path_id if path_id is not None else '-'}",
                detail_lines=detail_lines,
                payload=entry.get("raw"),
                file_path=entry.get("file_path"),
                expandable=hidden_relations,
                expanded=self._is_expanded(self._individual_node_id(entry_key)),
            )
        )

    def _add_path_node(self, path_id: int, *, display_role: str, depth: float) -> None:
        path_payload = self.cache.flight_paths.get(path_id)
        if not path_payload:
            return
        payload = path_payload.get("data") or {}
        waypoint_count = len(_coalesce_waypoints(payload))
        node_id = self._path_node_id(path_id)
        self._add_node(
            GraphNode(
                node_id=node_id,
                entity_type="path",
                display_role=display_role,
                depth=depth,
                title=f"PATH {path_id}",
                subtitle=f"WP {waypoint_count} · A/C {path_payload.get('aircraft_id') or '-'}",
                detail_lines=[
                    f"PathID: {path_id}",
                    f"AircraftID: {path_payload.get('aircraft_id') or '-'}",
                    f"WaypointCount: {waypoint_count}",
                    f"Timestamp: {_pretty_time(payload.get('timestamp'))}",
                ],
                payload=payload,
                file_path=path_payload.get("path"),
                expandable=bool(waypoint_count or self.cache.path_parent_links.get(path_id) or self.cache.path_child_links.get(path_id)),
                expanded=self._is_expanded(node_id),
            )
        )

    def _add_waypoint_node(self, path_id: int, index: int, waypoint: Dict[str, Any], *, depth: float) -> None:
        coord = waypoint.get("coordinate") or {}
        lat = _as_float(coord.get("latitude"))
        lon = _as_float(coord.get("longitude"))
        alt = _as_int(coord.get("altitude"))
        waypoint_id = _as_int(waypoint.get("waypointID")) or index
        self._add_node(
            GraphNode(
                node_id=self._waypoint_node_id(path_id, waypoint_id, index),
                entity_type="waypoint",
                display_role="waypoint",
                depth=depth,
                title=f"WP {waypoint_id}",
                subtitle=f"{lat if lat is not None else '-'}, {lon if lon is not None else '-'}",
                detail_lines=[
                    f"PathID: {path_id}",
                    f"WaypointID: {waypoint_id}",
                    f"NextWaypointID: {_as_int(waypoint.get('nextWaypointID')) or '-'}",
                    f"Speed: {waypoint.get('speed') if waypoint.get('speed') is not None else '-'}",
                    f"ETA: {waypoint.get('eta') if waypoint.get('eta') is not None else '-'}",
                    f"Altitude: {alt if alt is not None else '-'}",
                ],
                payload=waypoint,
                file_path=(self.cache.flight_paths.get(path_id) or {}).get("path"),
                expandable=False,
                expanded=False,
            )
        )

    def _add_node(self, node: GraphNode) -> None:
        current = self.nodes.get(node.node_id)
        if current is None:
            self.nodes[node.node_id] = node
            return
        if node.depth < current.depth:
            current.depth = node.depth
        if len(node.detail_lines) > len(current.detail_lines):
            current.detail_lines = list(node.detail_lines)
        if current.file_path is None and node.file_path is not None:
            current.file_path = node.file_path
        if not current.subtitle and node.subtitle:
            current.subtitle = node.subtitle
        if current.payload is None and node.payload is not None:
            current.payload = node.payload
        current.expandable = current.expandable or node.expandable
        current.expanded = current.expanded or node.expanded

    def _add_edge(
        self,
        source_id: str,
        target_id: str,
        label: str,
        kind: str,
        color: str,
        *,
        dashed: bool = False,
    ) -> None:
        for edge in self.edges:
            if edge.source_id == source_id and edge.target_id == target_id and edge.label == label:
                return
        self.edges.append(
            GraphEdge(
                source_id=source_id,
                target_id=target_id,
                label=label,
                kind=kind,
                color=color,
                dashed=dashed,
            )
        )

    def _is_expanded(self, node_id: str) -> bool:
        return node_id in self.expanded_nodes

    @staticmethod
    def _plan_node_id(plan_id: int) -> str:
        return f"plan:{int(plan_id)}"

    @staticmethod
    def _package_node_id(package_id: int, role: str) -> str:
        return f"{role}:{int(package_id)}"

    @staticmethod
    def _input_mission_node_id(package_id: int, input_mission_id: int) -> str:
        return f"input:{int(package_id)}:{int(input_mission_id)}"

    @staticmethod
    def _imp_node_id(imp_id: int) -> str:
        return f"imp:{int(imp_id)}"

    @staticmethod
    def _individual_node_id(entry_key: str) -> str:
        return f"individual:{entry_key}"

    @staticmethod
    def _path_node_id(path_id: int) -> str:
        return f"path:{int(path_id)}"

    @staticmethod
    def _waypoint_node_id(path_id: int, waypoint_id: int, index: int) -> str:
        return f"wp:{int(path_id)}:{int(waypoint_id)}:{int(index)}"

    @staticmethod
    def _prior_node_id(prior_mission_id: int) -> str:
        return f"prior:{int(prior_mission_id)}"


NODE_COLORS: Dict[str, Tuple[str, str]] = {
    "mission_plan": ("#216db6", "#123a61"),
    "source_plan": ("#7b8ba1", "#435267"),
    "child_plan": ("#5d83c6", "#2f4d7d"),
    "input_package": ("#2a9d8f", "#1d6f66"),
    "reference_package": ("#7c6ad4", "#55439c"),
    "input_mission": ("#49a6d8", "#1f6d93"),
    "individual_package": ("#2f855a", "#1e5a3c"),
    "individual_mission": ("#805ad5", "#5a3ca3"),
    "source_individual": ("#9c7ce4", "#6849ba"),
    "child_individual": ("#9c5b7a", "#6f3550"),
    "path": ("#d69e2e", "#8a6517"),
    "source_path": ("#c07c1f", "#865514"),
    "child_path": ("#de8a32", "#99541c"),
    "waypoint": ("#94a3b8", "#65758b"),
    "prior_reference": ("#d25743", "#8f3528"),
}


EDGE_LABEL_FONTS: Dict[str, QFont] = {}


class RelationshipNodeItem(QGraphicsObject):
    def __init__(self, node: GraphNode, click_cb, double_click_cb, move_cb=None) -> None:
        super().__init__()
        self.node = node
        self._click_cb = click_cb
        self._double_click_cb = double_click_cb
        self._move_cb = move_cb
        self._selected = False
        self._dragging = False
        self._press_pos = QPointF()
        self._width = 230.0 if node.entity_type != "waypoint" else 180.0
        self._height = 86.0 if node.entity_type != "waypoint" else 62.0
        self.setAcceptedMouseButtons(Qt.LeftButton)
        self.setAcceptHoverEvents(True)
        self.setCursor(QCursor(Qt.OpenHandCursor))
        self.setFlag(QGraphicsItem.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)

    def _badge_rect(self) -> QRectF:
        rect = self.boundingRect()
        return QRectF(rect.right() - 40.0, rect.top() + 10.0, 20.0, 20.0)

    def boundingRect(self) -> QRectF:  # type: ignore[override]
        return QRectF(0.0, 0.0, self._width, self._height)

    def shape(self) -> QPainterPath:  # type: ignore[override]
        path = QPainterPath()
        path.addRoundedRect(self.boundingRect(), 16.0, 16.0)
        return path

    def set_selected(self, selected: bool) -> None:
        if self._selected == selected:
            return
        self._selected = selected
        self.update()

    def paint(self, painter: QPainter, option, widget=None) -> None:  # type: ignore[override]
        painter.setRenderHint(QPainter.Antialiasing)
        fill_hex, border_hex = NODE_COLORS.get(
            self.node.display_role,
            NODE_COLORS.get(self.node.entity_type, ("#3b82f6", "#1e40af")),
        )
        rect = self.boundingRect()

        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 18))
        painter.drawRoundedRect(rect.adjusted(4.0, 6.0, -2.0, -2.0), 16.0, 16.0)

        painter.setBrush(QBrush(QColor(fill_hex)))
        painter.setPen(QPen(QColor("#ffffff" if self._selected else border_hex), 2.4 if self._selected else 1.6))
        painter.drawRoundedRect(rect.adjusted(0.0, 0.0, -6.0, -8.0), 16.0, 16.0)

        content = rect.adjusted(16.0, 12.0, -22.0, -18.0)
        title_font = QFont("Malgun Gothic", 10)
        title_font.setBold(True)
        subtitle_font = QFont("Malgun Gothic", 8)
        badge_font = QFont("Malgun Gothic", 8)
        badge_font.setBold(True)

        painter.setPen(QColor("#f8fbff"))
        painter.setFont(title_font)
        painter.drawText(QRectF(content.x(), content.y(), content.width(), 22.0), Qt.AlignLeft | Qt.AlignVCenter, self.node.title)

        painter.setFont(subtitle_font)
        painter.setPen(QColor("#dbeafe"))
        painter.drawText(QRectF(content.x(), content.y() + 24.0, content.width(), 30.0), Qt.TextWordWrap, self.node.subtitle)

        if self.node.expandable:
            badge_rect = self._badge_rect()
            painter.setBrush(QColor(255, 255, 255, 40))
            painter.setPen(QPen(QColor("#e8f2ff"), 1.0))
            painter.drawEllipse(badge_rect)
            painter.setFont(badge_font)
            painter.setPen(QColor("#ffffff"))
            painter.drawText(badge_rect, Qt.AlignCenter, "-" if self.node.expanded else "+")

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        self._dragging = False
        self._press_pos = QPointF(self.pos())
        self.setCursor(QCursor(Qt.ClosedHandCursor))
        super().mousePressEvent(event)
        event.accept()

    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
        if (self.pos() - self._press_pos).manhattanLength() > 2.0:
            self._dragging = True
        super().mouseMoveEvent(event)
        event.accept()

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        self.setCursor(QCursor(Qt.OpenHandCursor))
        super().mouseReleaseEvent(event)
        if not self._dragging and callable(self._click_cb):
            toggle_expand = bool(self.node.expandable and self._badge_rect().contains(event.pos()))
            self._click_cb(self.node.node_id, toggle_expand)
        event.accept()

    def mouseDoubleClickEvent(self, event) -> None:  # type: ignore[override]
        if callable(self._double_click_cb):
            self._double_click_cb(self.node.node_id)
        event.accept()

    def itemChange(self, change, value):  # type: ignore[override]
        if change == QGraphicsItem.ItemPositionHasChanged and callable(self._move_cb):
            self._move_cb(self.node.node_id, QPointF(value))
        return super().itemChange(change, value)


class RelationshipEdgeItem(QGraphicsPathItem):
    def __init__(self, edge: GraphEdge, source_item: RelationshipNodeItem, target_item: RelationshipNodeItem) -> None:
        super().__init__()
        self.edge = edge
        self.source_item = source_item
        self.target_item = target_item
        self._highlighted = False
        self._path_rect = QRectF()
        self._last_anchor_end = QPointF()
        self._last_label_point = QPointF()
        self.setZValue(-1.0)
        self.refresh_path()

    def set_highlighted(self, highlighted: bool) -> None:
        if self._highlighted == highlighted:
            return
        self._highlighted = highlighted
        self.update()

    def refresh_path(self) -> None:
        source_rect = self.source_item.sceneBoundingRect()
        target_rect = self.target_item.sceneBoundingRect()
        source_center = source_rect.center()
        target_center = target_rect.center()

        if source_center.x() <= target_center.x():
            start = QPointF(source_rect.right() - 6.0, source_center.y())
            end = QPointF(target_rect.left(), target_center.y())
        else:
            start = QPointF(source_rect.left() + 6.0, source_center.y())
            end = QPointF(target_rect.right() - 2.0, target_center.y())

        mid_x = (start.x() + end.x()) / 2.0
        path = QPainterPath(start)
        path.lineTo(mid_x, start.y())
        path.lineTo(mid_x, end.y())
        path.lineTo(end)
        self._last_anchor_end = end
        self._last_label_point = QPointF(mid_x, (start.y() + end.y()) / 2.0)
        self.setPath(path)
        stroker = QPainterPathStroker()
        stroker.setWidth(14.0)
        self._path_rect = stroker.createStroke(path).boundingRect().adjusted(-18.0, -18.0, 18.0, 18.0)

    def boundingRect(self) -> QRectF:  # type: ignore[override]
        return self._path_rect

    def paint(self, painter: QPainter, option, widget=None) -> None:  # type: ignore[override]
        if self.path().isEmpty():
            return
        painter.setRenderHint(QPainter.Antialiasing)
        color = QColor(self.edge.color)
        if self._highlighted:
            color = color.lighter(130)
        pen = QPen(color, 2.8 if self._highlighted else 2.0)
        if self.edge.dashed:
            pen.setStyle(Qt.DashLine)
            pen.setDashPattern([7.0, 5.0])
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(self.path())
        self._draw_arrow_head(painter, color)
        self._draw_label(painter, color)

    def _draw_arrow_head(self, painter: QPainter, color: QColor) -> None:
        path = self.path()
        if path.elementCount() < 2:
            return
        end = self._last_anchor_end
        prev = path.elementAt(path.elementCount() - 2)
        start = QPointF(prev.x, prev.y)
        angle = math.atan2(end.y() - start.y(), end.x() - start.x())
        size = 10.0
        left = QPointF(
            end.x() - math.cos(angle - math.pi / 6.0) * size,
            end.y() - math.sin(angle - math.pi / 6.0) * size,
        )
        right = QPointF(
            end.x() - math.cos(angle + math.pi / 6.0) * size,
            end.y() - math.sin(angle + math.pi / 6.0) * size,
        )
        painter.setBrush(QBrush(color))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(end, left, right)

    def _draw_label(self, painter: QPainter, color: QColor) -> None:
        text = str(self.edge.label or "").strip()
        if not text:
            return
        font = EDGE_LABEL_FONTS.get("edge")
        if font is None:
            font = QFont("Malgun Gothic", 8)
            font.setBold(True)
            EDGE_LABEL_FONTS["edge"] = font
        painter.setFont(font)
        metrics = QFontMetrics(font)
        text_width = metrics.horizontalAdvance(text) + 12
        text_height = metrics.height() + 4
        rect = QRectF(
            self._last_label_point.x() - text_width / 2.0,
            self._last_label_point.y() - text_height / 2.0,
            text_width,
            text_height,
        )
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(255, 255, 255, 228))
        painter.drawRoundedRect(rect, 8.0, 8.0)
        painter.setPen(QPen(color.darker(115), 1.0))
        painter.drawRoundedRect(rect, 8.0, 8.0)
        painter.setPen(QColor("#243447"))
        painter.drawText(rect, Qt.AlignCenter, text)


class RelationshipGraphView(QGraphicsView):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setBackgroundBrush(QColor("#eef3fa"))
        self.setFrameShape(QFrame.NoFrame)
        self._panning = False
        self._pan_start = QPoint()

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        delta = event.angleDelta().y()
        if delta == 0:
            event.ignore()
            return
        self.scale(1.15 if delta > 0 else 1.0 / 1.15, 1.15 if delta > 0 else 1.0 / 1.15)
        event.accept()

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.RightButton:
            self._panning = True
            self._pan_start = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
        if self._panning:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.RightButton and self._panning:
            self._panning = False
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)


class RawPayloadDialog(QDialog):
    def __init__(self, title: str, text: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(760, 620)
        layout = QVBoxLayout(self)
        editor = QPlainTextEdit()
        editor.setReadOnly(True)
        editor.setPlainText(text)
        layout.addWidget(editor, 1)
        button_row = QHBoxLayout()
        copy_btn = QPushButton("복사")
        copy_btn.clicked.connect(lambda: QApplication.clipboard().setText(text))
        close_btn = QPushButton("닫기")
        close_btn.clicked.connect(self.accept)
        button_row.addStretch(1)
        button_row.addWidget(copy_btn)
        button_row.addWidget(close_btn)
        layout.addLayout(button_row)


class MissionIdRelationshipTab(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("missionRelationshipTab")
        self.cache = RelationshipCache()
        self._session_scope: Optional[Dict[str, Set[int]]] = None
        self._input_status = {"0201": None, "0203": None, "plan": "임무계획 전"}
        self._expanded_nodes: Set[str] = set()
        self._selected_node_id: Optional[str] = None
        self._current_plan_id: Optional[int] = None
        self._nodes: Dict[str, GraphNode] = {}
        self._edges: List[GraphEdge] = []
        self._node_items: Dict[str, RelationshipNodeItem] = {}
        self._edge_items: List[RelationshipEdgeItem] = []
        self._node_positions: Dict[int, Dict[str, QPointF]] = {}
        self._drawing_graph = False
        self._layout_focus_node_id: Optional[str] = None
        self._subtree_shift_guard = False

        self.setStyleSheet(self._build_stylesheet())
        self._build_ui()
        self.refresh()

    def _build_stylesheet(self) -> str:
        return """
            QWidget#missionRelationshipTab { background: #f4f7fb; color: #1b2840; }
            QFrame#relCard { background: #ffffff; border: 1px solid #d9e4ef; border-radius: 18px; }
            QLabel#relTitle { color: #10203b; font-size: 20px; font-weight: 700; }
            QLabel#relSubTitle { color: #65758d; font-size: 11px; }
            QLabel#relHint { background: #eef4ff; color: #294a7e; border: 1px solid #d3def4; border-radius: 11px; padding: 8px 10px; }
            QLabel#relSectionTitle { color: #14345c; font-size: 13px; font-weight: 700; }
            QLabel#relMeta { color: #5f6f86; font-size: 11px; }
            QLabel#relBadge { background: #f6f9fc; border: 1px solid #dce7f3; border-radius: 10px; padding: 6px 10px; color: #27405c; }
            QComboBox, QPlainTextEdit {
                background: #fbfdff;
                border: 1px solid #c9d7e7;
                border-radius: 10px;
                padding: 6px 8px;
            }
            QPushButton, QToolButton {
                background: #e9f1fc;
                border: 1px solid #c8d6ee;
                border-radius: 10px;
                color: #17365d;
                padding: 7px 12px;
                font-weight: 600;
            }
            QPushButton:hover, QToolButton:hover { background: #dce9fb; }
            QCheckBox { color: #1e3352; spacing: 8px; }
        """

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        header = QFrame()
        header.setObjectName("relCard")
        header_layout = QVBoxLayout(header)
        header_layout.setContentsMargins(18, 16, 18, 16)
        header_layout.setSpacing(10)

        title_row = QHBoxLayout()
        title_box = QVBoxLayout()
        title = QLabel("임무 관계도")
        title.setObjectName("relTitle")
        subtitle = QLabel("현재 시나리오 DB에서 MissionPlan 기준 ID 연결 구조를 시각적으로 추적합니다.")
        subtitle.setObjectName("relSubTitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        title_row.addLayout(title_box, 1)

        self._summary_label = QLabel("")
        self._summary_label.setObjectName("relBadge")
        self._summary_label.setWordWrap(True)
        title_row.addWidget(self._summary_label, 1)
        header_layout.addLayout(title_row)

        control_row = QHBoxLayout()
        self._plan_combo = QComboBox()
        self._plan_combo.currentIndexChanged.connect(self._on_plan_changed)
        self._plan_combo.setMinimumWidth(260)
        control_row.addWidget(self._plan_combo, 0)

        self._scope_only_check = QCheckBox("세션 범위만 보기")
        self._scope_only_check.setChecked(True)
        self._scope_only_check.toggled.connect(self._reload_plan_list)
        control_row.addWidget(self._scope_only_check, 0)

        refresh_btn = QPushButton("새로고침")
        refresh_btn.clicked.connect(self.refresh)
        control_row.addWidget(refresh_btn, 0)

        fit_btn = QPushButton("화면 맞춤")
        fit_btn.clicked.connect(self._fit_graph)
        control_row.addWidget(fit_btn, 0)

        reset_btn = QPushButton("확장 초기화")
        reset_btn.clicked.connect(self._reset_expansion)
        control_row.addWidget(reset_btn, 0)

        self._focus_plan_btn = QPushButton("선택 Plan으로 보기")
        self._focus_plan_btn.clicked.connect(self._focus_selected_plan)
        self._focus_plan_btn.setEnabled(False)
        control_row.addWidget(self._focus_plan_btn, 0)

        control_row.addStretch(1)

        self._status_label = QLabel("")
        self._status_label.setObjectName("relHint")
        self._status_label.setWordWrap(True)
        self._status_label.setMinimumWidth(360)
        control_row.addWidget(self._status_label, 1)
        header_layout.addLayout(control_row)
        root.addWidget(header, 0)

        body_splitter = QSplitter(Qt.Horizontal)
        body_splitter.setChildrenCollapsible(False)

        graph_card = QFrame()
        graph_card.setObjectName("relCard")
        graph_layout = QVBoxLayout(graph_card)
        graph_layout.setContentsMargins(14, 14, 14, 14)
        graph_layout.setSpacing(10)
        graph_title = QLabel("그래프")
        graph_title.setObjectName("relSectionTitle")
        graph_layout.addWidget(graph_title, 0)

        self._scene = QGraphicsScene(self)
        self._graph_view = RelationshipGraphView()
        self._graph_view.setScene(self._scene)
        graph_layout.addWidget(self._graph_view, 1)
        body_splitter.addWidget(graph_card)

        side_card = QFrame()
        side_card.setObjectName("relCard")
        side_layout = QVBoxLayout(side_card)
        side_layout.setContentsMargins(14, 14, 14, 14)
        side_layout.setSpacing(10)

        self._selection_title = QLabel("선택 없음")
        self._selection_title.setObjectName("relSectionTitle")
        side_layout.addWidget(self._selection_title, 0)

        self._selection_meta = QLabel("노드를 클릭하면 상세 관계를 보여줍니다.")
        self._selection_meta.setObjectName("relMeta")
        self._selection_meta.setWordWrap(True)
        side_layout.addWidget(self._selection_meta, 0)

        raw_btn = QToolButton()
        raw_btn.setText("Raw 텍스트 보기")
        raw_btn.clicked.connect(self._show_selected_raw_dialog)
        side_layout.addWidget(raw_btn, 0, Qt.AlignLeft)

        legend = QLabel("Plan 파랑 | Input 청록 | Individual 초록/보라 | Path 황토 | 점선은 재계획/재사용 연결")
        legend.setObjectName("relHint")
        legend.setWordWrap(True)
        side_layout.addWidget(legend, 0)

        self._detail_text = QPlainTextEdit()
        self._detail_text.setReadOnly(True)
        self._detail_text.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        side_layout.addWidget(self._detail_text, 1)

        guide = QLabel("좌클릭 선택/확장, 더블클릭 raw 창, 우클릭 드래그 팬, 휠 확대·축소")
        guide.setObjectName("relMeta")
        guide.setWordWrap(True)
        side_layout.addWidget(guide, 0)

        body_splitter.addWidget(side_card)
        body_splitter.setStretchFactor(0, 3)
        body_splitter.setStretchFactor(1, 1)
        root.addWidget(body_splitter, 1)

    def refresh(self) -> None:
        self.cache.refresh()
        self._reload_plan_list()
        self._update_summary()
        self._update_status_label()

    def update_session_scope(self, scope: Optional[Dict[str, Set[int]]]) -> None:
        if scope is None:
            self._session_scope = None
        else:
            self._session_scope = {
                "packages": set(scope.get("packages", set())),
                "plans": set(scope.get("plans", set())),
                "individual_packages": set(scope.get("individual_packages", set())),
                "paths": set(scope.get("paths", set())),
            }
        self._reload_plan_list()
        self._update_summary()

    def update_input_status(
        self,
        *,
        cmpk_id: Optional[int] = None,
        mrpk_id: Optional[int] = None,
        plan_state: Optional[str] = None,
    ) -> None:
        if cmpk_id is not None:
            self._input_status["0201"] = cmpk_id
        if mrpk_id is not None:
            self._input_status["0203"] = mrpk_id
        if plan_state is not None:
            self._input_status["plan"] = plan_state
        self._update_status_label()

    def _reload_plan_list(self) -> None:
        current = self._current_plan_id
        plan_ids = self._available_plan_ids()
        self._plan_combo.blockSignals(True)
        self._plan_combo.clear()
        for plan_id in plan_ids:
            self._plan_combo.addItem(f"{plan_id}  |  {self.cache.get_plan_summary(plan_id)}", plan_id)
        self._plan_combo.blockSignals(False)

        if not plan_ids:
            self._current_plan_id = None
            self._render_empty_graph("MissionPlan이 없습니다.")
            return

        target = current if current in plan_ids else plan_ids[-1]
        index = max(0, self._plan_combo.findData(target))
        self._plan_combo.setCurrentIndex(index)
        self._current_plan_id = int(self._plan_combo.currentData())
        self._rebuild_graph(preserve_view=False, fit=True)

    def _available_plan_ids(self) -> List[int]:
        plan_ids = self.cache.get_plan_ids()
        if not self._scope_only_check.isChecked():
            return plan_ids
        if not self._session_scope:
            return plan_ids
        scoped = sorted(
            int(plan_id)
            for plan_id in self._session_scope.get("plans", set())
            if int(plan_id) in self.cache.mission_plans
        )
        return scoped or plan_ids

    def _on_plan_changed(self, index: int) -> None:
        plan_id = _as_int(self._plan_combo.itemData(index))
        if plan_id is None:
            return
        self._current_plan_id = plan_id
        self._expanded_nodes.clear()
        self._layout_focus_node_id = None
        self._selected_node_id = self._plan_node_id(plan_id)
        self._rebuild_graph(preserve_view=False, fit=True)

    def _rebuild_graph(self, *, preserve_view: bool = True, fit: bool = False) -> None:
        plan_id = self._current_plan_id
        if plan_id is None:
            self._render_empty_graph("MissionPlan이 없습니다.")
            return
        previous_node_ids = set(self._nodes.keys())
        previous_positions = self._capture_node_positions()
        view_state = self._capture_view_state() if preserve_view else None
        builder = PlanGraphBuilder(self.cache, plan_id, self._expanded_nodes)
        self._nodes, self._edges = builder.build()
        if not self._nodes:
            self._render_empty_graph(f"MissionPlan {plan_id} 데이터를 읽지 못했습니다.")
            return
        self._draw_graph(previous_positions=previous_positions, new_node_ids=set(self._nodes.keys()) - previous_node_ids)
        if self._selected_node_id not in self._nodes:
            self._selected_node_id = self._plan_node_id(plan_id)
        self._apply_selection()
        if fit:
            self._fit_graph()
        elif view_state is not None:
            self._restore_view_state(view_state)
        self._layout_focus_node_id = None

    def _draw_graph(
        self,
        *,
        previous_positions: Optional[Dict[str, QPointF]] = None,
        new_node_ids: Optional[Set[str]] = None,
    ) -> None:
        self._scene.clear()
        self._node_items.clear()
        self._edge_items.clear()
        self._drawing_graph = True

        positioned = self._layout_nodes(
            self._nodes,
            previous_positions=previous_positions or {},
            new_node_ids=new_node_ids or set(),
        )
        for node_id, (_, x, y) in positioned.items():
            item = RelationshipNodeItem(
                self._nodes[node_id],
                self._handle_node_click,
                self._handle_node_double_click,
                self._handle_node_moved,
            )
            self._scene.addItem(item)
            item.setPos(x, y)
            self._node_items[node_id] = item

        for edge in self._edges:
            source_item = self._node_items.get(edge.source_id)
            target_item = self._node_items.get(edge.target_id)
            if source_item is None or target_item is None:
                continue
            item = RelationshipEdgeItem(edge, source_item, target_item)
            self._scene.addItem(item)
            self._edge_items.append(item)

        rect = self._scene.itemsBoundingRect().adjusted(-80.0, -80.0, 80.0, 80.0)
        self._scene.setSceneRect(rect)
        self._drawing_graph = False

    def _capture_node_positions(self) -> Dict[str, QPointF]:
        if self._current_plan_id is None:
            return {}
        captured: Dict[str, QPointF] = {}
        plan_positions = self._node_positions.setdefault(self._current_plan_id, {})
        for node_id, item in self._node_items.items():
            pos = QPointF(item.pos())
            captured[node_id] = QPointF(pos)
            plan_positions[node_id] = QPointF(pos)
        return captured

    def _layout_nodes(
        self,
        nodes: Dict[str, GraphNode],
        *,
        previous_positions: Dict[str, QPointF],
        new_node_ids: Set[str],
    ) -> Dict[str, Tuple[GraphNode, float, float]]:
        depth_buckets: Dict[float, List[GraphNode]] = defaultdict(list)
        for node in nodes.values():
            depth_buckets[node.depth].append(node)
        positioned: Dict[str, Tuple[GraphNode, float, float]] = {}
        x_step = 280.0
        y_step = 112.0
        depth_x = {
            depth: 40.0 + depth_index * x_step
            for depth_index, depth in enumerate(sorted(depth_buckets.keys()))
        }
        plan_positions = self._node_positions.setdefault(self._current_plan_id or 0, {})
        for node_id, pos in previous_positions.items():
            if node_id in nodes:
                plan_positions[node_id] = QPointF(pos)
        for node_id in new_node_ids:
            plan_positions.pop(node_id, None)

        occupied: List[Tuple[float, float, float, float]] = []

        def _node_size(node_id: str) -> Tuple[float, float]:
            if nodes[node_id].entity_type == "waypoint":
                return 180.0, 62.0
            return 230.0, 86.0

        def _register(node_id: str, x: float, y: float) -> None:
            positioned[node_id] = (nodes[node_id], x, y)
            width, height = _node_size(node_id)
            occupied.append((x, y, width, height))
            plan_positions[node_id] = QPointF(x, y)

        def _collides(x: float, y: float, width: float, height: float) -> bool:
            for ox, oy, ow, oh in occupied:
                if abs(x - ox) < max(width, ow) * 0.82 and abs(y - oy) < max(height, oh) * 0.78:
                    return True
            return False

        def _resolve_y(x: float, y: float, width: float, height: float) -> float:
            attempt = y
            tries = 0
            while _collides(x, attempt, width, height) and tries < 200:
                attempt += y_step * 0.72
                tries += 1
            return attempt

        for node_id, stored in list(plan_positions.items()):
            if node_id in nodes:
                _register(node_id, stored.x(), stored.y())

        outgoing: Dict[str, List[str]] = defaultdict(list)
        incoming: Dict[str, List[str]] = defaultdict(list)
        for edge in self._edges:
            outgoing[edge.source_id].append(edge.target_id)
            incoming[edge.target_id].append(edge.source_id)

        def _choose_anchor(node_id: str) -> Optional[str]:
            focus_id = self._layout_focus_node_id
            if focus_id and focus_id in positioned:
                if node_id in outgoing.get(focus_id, []) or node_id in incoming.get(focus_id, []):
                    return focus_id
            for candidate in incoming.get(node_id, []):
                if candidate in positioned:
                    return candidate
            for candidate in outgoing.get(node_id, []):
                if candidate in positioned:
                    return candidate
            return None

        remaining = [node_id for node_id in nodes.keys() if node_id not in positioned]
        while remaining:
            groups: Dict[Tuple[str, float], List[str]] = defaultdict(list)
            unresolved: List[str] = []
            for node_id in remaining:
                anchor_id = _choose_anchor(node_id)
                if anchor_id is None:
                    unresolved.append(node_id)
                    continue
                delta_x = depth_x[nodes[node_id].depth] - depth_x[positioned[anchor_id][0].depth]
                groups[(anchor_id, delta_x)].append(node_id)

            if not groups:
                break

            for (anchor_id, delta_x), node_ids in groups.items():
                _, anchor_x, anchor_y = positioned[anchor_id]
                ordered = sorted(node_ids, key=_node_sort_key)
                start_y = anchor_y - ((len(ordered) - 1) * y_step * 0.5)
                for row, node_id in enumerate(ordered):
                    width, height = _node_size(node_id)
                    x = anchor_x + delta_x
                    y = _resolve_y(x, start_y + row * y_step, width, height)
                    _register(node_id, x, y)

            remaining = unresolved

        for depth in sorted(depth_buckets.keys()):
            bucket = sorted(depth_buckets[depth], key=lambda item: _node_sort_key(item.node_id))
            row = 0
            for node in bucket:
                if node.node_id in positioned:
                    continue
                width, height = _node_size(node.node_id)
                x = depth_x[depth]
                y = _resolve_y(x, 40.0 + row * y_step, width, height)
                _register(node.node_id, x, y)
                row += 1
        return positioned

    def _render_empty_graph(self, message: str) -> None:
        self._scene.clear()
        self._scene.setSceneRect(QRectF(0, 0, 800, 500))
        self._scene.addText(message, QFont("Malgun Gothic", 12))
        self._selection_title.setText("선택 없음")
        self._selection_meta.setText(message)
        self._detail_text.setPlainText(message)
        self._focus_plan_btn.setEnabled(False)

    def _handle_node_click(self, node_id: str, toggle_expand: bool = False) -> None:
        self._selected_node_id = node_id
        node = self._nodes.get(node_id)
        if node and node.expandable and toggle_expand:
            self._layout_focus_node_id = node_id
            if node_id in self._expanded_nodes:
                self._expanded_nodes.discard(node_id)
            else:
                self._expanded_nodes.add(node_id)
            self._rebuild_graph(preserve_view=True, fit=False)
            return
        self._apply_selection()

    def _handle_node_double_click(self, node_id: str) -> None:
        self._selected_node_id = node_id
        self._apply_selection()
        self._show_selected_raw_dialog()

    def _handle_node_moved(self, node_id: str, pos: QPointF) -> None:
        if self._drawing_graph or self._current_plan_id is None or self._subtree_shift_guard:
            return
        plan_positions = self._node_positions.setdefault(self._current_plan_id, {})
        prev = QPointF(plan_positions.get(node_id, QPointF(pos)))
        plan_positions[node_id] = QPointF(pos)
        delta = QPointF(pos.x() - prev.x(), pos.y() - prev.y())
        if abs(delta.x()) > 0.01 or abs(delta.y()) > 0.01:
            self._shift_visible_subtree(node_id, delta)
        for edge_item in self._edge_items:
            edge_item.refresh_path()
        rect = self._scene.itemsBoundingRect().adjusted(-80.0, -80.0, 80.0, 80.0)
        self._scene.setSceneRect(rect)

    def _shift_visible_subtree(self, root_node_id: str, delta: QPointF) -> None:
        if self._current_plan_id is None:
            return
        outgoing: Dict[str, List[str]] = defaultdict(list)
        for edge in self._edges:
            outgoing[edge.source_id].append(edge.target_id)
        descendants: List[str] = []
        queue: List[str] = list(outgoing.get(root_node_id, []))
        visited: Set[str] = set()
        while queue:
            node_id = queue.pop(0)
            if node_id in visited or node_id not in self._node_items:
                continue
            visited.add(node_id)
            descendants.append(node_id)
            queue.extend(outgoing.get(node_id, []))
        if not descendants:
            return

        plan_positions = self._node_positions.setdefault(self._current_plan_id, {})
        blocked: List[QRectF] = []
        moving_set = set(descendants) | {root_node_id}
        for node_id, item in self._node_items.items():
            if node_id in moving_set:
                continue
            rect = item.sceneBoundingRect().adjusted(-20.0, -14.0, 20.0, 14.0)
            blocked.append(rect)

        def _avoid_overlap(rect: QRectF) -> QPointF:
            shifted = QPointF(rect.topLeft())
            current = QRectF(rect)
            tries = 0
            while any(current.intersects(other) for other in blocked) and tries < 200:
                current.translate(0.0, 28.0)
                shifted = QPointF(current.topLeft())
                tries += 1
            blocked.append(current)
            return shifted

        self._subtree_shift_guard = True
        try:
            for node_id in descendants:
                item = self._node_items.get(node_id)
                if item is None:
                    continue
                target = item.pos() + delta
                rect = QRectF(target.x(), target.y(), item.boundingRect().width(), item.boundingRect().height())
                adjusted = _avoid_overlap(rect)
                item.setPos(adjusted)
                plan_positions[node_id] = QPointF(adjusted)
        finally:
            self._subtree_shift_guard = False

    def _apply_selection(self) -> None:
        selected = self._selected_node_id
        for node_id, item in self._node_items.items():
            item.set_selected(node_id == selected)
        for edge_item in self._edge_items:
            edge_item.set_highlighted(bool(selected) and (edge_item.edge.source_id == selected or edge_item.edge.target_id == selected))

        node = self._nodes.get(selected or "")
        if node is None:
            self._selection_title.setText("선택 없음")
            self._selection_meta.setText("노드를 클릭하면 상세 관계를 보여줍니다.")
            self._detail_text.clear()
            self._focus_plan_btn.setEnabled(False)
            return

        self._selection_title.setText(node.title)
        self._selection_meta.setText(node.subtitle)
        detail_lines = list(node.detail_lines)
        if node.file_path:
            detail_lines.append(f"File: {self._relative_path(node.file_path)}")
        if node.expandable:
            detail_lines.append(f"ExpandState: {'expanded' if node.node_id in self._expanded_nodes else 'collapsed'}")
        self._detail_text.setPlainText("\n".join(detail_lines))
        self._focus_plan_btn.setEnabled(node.entity_type == "mission_plan" and node.node_id != self._plan_node_id(self._current_plan_id or 0))

    def _show_selected_raw_dialog(self) -> None:
        node = self._nodes.get(self._selected_node_id or "")
        if node is None:
            return
        lines: List[str] = []
        if node.file_path:
            lines.append(f"File: {self._relative_path(node.file_path)}")
            lines.append("")
        lines.append(_safe_json_text(node.payload))
        RawPayloadDialog(node.title, "\n".join(lines), self).exec_()

    def _update_summary(self) -> None:
        summary = (
            f"MissionPlan {len(self.cache.mission_plans)}  |  "
            f"IndividualPackage {len(self.cache.individual_packages)}  |  "
            f"Path {len(self.cache.flight_paths)}  |  "
            f"ReplanLink {len(self.cache.replan_links)}"
        )
        if self._session_scope:
            summary += f"  |  SessionPlans {len(self._session_scope.get('plans', set()))}"
        if self.cache.errors:
            summary += f"  |  Errors {len(self.cache.errors)}"
            self._summary_label.setToolTip("\n".join(self.cache.errors[:30]))
        else:
            self._summary_label.setToolTip("")
        self._summary_label.setText(summary)

    def _update_status_label(self) -> None:
        cmpk = self._input_status.get("0201")
        mrpk = self._input_status.get("0203")
        plan = self._input_status.get("plan") or "임무계획 전"
        self._status_label.setText(
            f"0201: {cmpk if cmpk is not None else '미수신'}  |  "
            f"0203: {mrpk if mrpk is not None else '미수신'}  |  "
            f"Plan: {plan}"
        )

    def _reset_expansion(self) -> None:
        self._expanded_nodes.clear()
        self._layout_focus_node_id = None
        if self._current_plan_id is not None:
            self._node_positions.pop(self._current_plan_id, None)
        self._selected_node_id = self._plan_node_id(self._current_plan_id or 0)
        self._rebuild_graph(preserve_view=False, fit=True)

    def _fit_graph(self) -> None:
        if not self._scene.items():
            return
        rect = self._scene.itemsBoundingRect().adjusted(-40.0, -40.0, 40.0, 40.0)
        self._graph_view.fitInView(rect, Qt.KeepAspectRatio)

    def _focus_selected_plan(self) -> None:
        node = self._nodes.get(self._selected_node_id or "")
        if node is None or node.entity_type != "mission_plan":
            return
        plan_id = _as_int(node.node_id.split(":", 1)[1] if ":" in node.node_id else None)
        if plan_id is None:
            return
        index = self._plan_combo.findData(plan_id)
        if index >= 0:
            self._plan_combo.setCurrentIndex(index)

    def _relative_path(self, path: Path) -> str:
        try:
            return str(Path(path).relative_to(self.cache.db_root))
        except ValueError:
            return str(path)

    def _capture_view_state(self) -> Optional[Tuple[QTransform, QPointF]]:
        if not self._scene.items():
            return None
        center = self._graph_view.mapToScene(self._graph_view.viewport().rect().center())
        return QTransform(self._graph_view.transform()), center

    def _restore_view_state(self, state: Optional[Tuple[QTransform, QPointF]]) -> None:
        if state is None:
            return
        transform, center = state
        self._graph_view.setTransform(transform)
        self._graph_view.centerOn(center)

    @staticmethod
    def _plan_node_id(plan_id: int) -> str:
        return f"plan:{int(plan_id)}"


__all__ = ["MissionIdRelationshipTab", "RelationshipCache"]
