from __future__ import annotations

import json
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QSplitter,
    QTextEdit,
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
    except (ValueError, TypeError):
        return None


class RelationshipCache:
    """Builds indices that describe how mission related IDs connect together."""

    def __init__(self, db_root: Path) -> None:
        self.db_root = Path(db_root)
        self.packages: Dict[int, Dict[str, Any]] = {}
        self.mission_plans: Dict[int, Dict[str, Any]] = {}
        self.individual_packages: Dict[int, Dict[str, Any]] = {}
        self.package_to_plans: Dict[int, List[int]] = defaultdict(list)
        self.plan_to_individual_packages: Dict[int, List[Dict[str, Optional[int]]]] = defaultdict(list)
        self.individual_package_to_packages: Dict[int, Set[int]] = defaultdict(set)
        self.individual_package_to_plans: Dict[int, Set[int]] = defaultdict(set)
        self.input_to_individuals: Dict[Tuple[int, int], List[Dict[str, Any]]] = defaultdict(list)
        self.individual_entries: Dict[int, Dict[str, Any]] = {}
        self.path_entries: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
        self.errors: List[str] = []

    def refresh(self) -> None:
        self.packages.clear()
        self.mission_plans.clear()
        self.individual_packages.clear()
        self.package_to_plans.clear()
        self.plan_to_individual_packages.clear()
        self.individual_package_to_packages.clear()
        self.individual_package_to_plans.clear()
        self.input_to_individuals.clear()
        self.individual_entries.clear()
        self.path_entries.clear()
        self.errors.clear()

        self._load_packages()
        self._load_mission_plans()
        self._load_individual_packages()

    # Package loading -----------------------------------------------------
    def _load_packages(self) -> None:
        input_dir = self.db_root / "InputMissionPlan"
        if not input_dir.exists():
            return
        for path in sorted(input_dir.glob("*.json")):
            pkg = self._safe_load(path, "[InputMissionPlan]")
            if pkg is None:
                continue

            pkg_id = _as_int(pkg.get("inputMissionPackageID"))
            if pkg_id is None:
                self._note_error(f"[InputMissionPlan] {path.name}: missing inputMissionPackageID")
                continue

            missions = {}
            for mission in pkg.get("inputMissionList") or []:
                mid = _as_int(mission.get("inputMissionID"))
                if mid is None:
                    continue
                missions[mid] = mission

            self.packages[pkg_id] = {
                "path": path,
                "data": pkg,
                "missions": missions,
                "timestamp": _as_int(pkg.get("timestamp")),
            }

    # Mission plan loading ------------------------------------------------
    def _load_mission_plans(self) -> None:
        plan_dir = self.db_root / "MissionPlan"
        if not plan_dir.exists():
            return

        for path in sorted(plan_dir.glob("*.json")):
            mp = self._safe_load(path, "[MissionPlan]")
            if mp is None:
                continue

            plan_id = _as_int(mp.get("missionPlanID"))
            if plan_id is None:
                self._note_error(f"[MissionPlan] {path.name}: missing missionPlanID")
                continue

            pkg_id = _as_int(mp.get("inputMissionPackageID"))
            if pkg_id is not None:
                self.package_to_plans[pkg_id].append(plan_id)

            aircraft_list = mp.get("aircraftList") or []
            mapped_aircraft: List[Dict[str, Optional[int]]] = []
            seen_packages: set[int] = set()
            for aircraft in aircraft_list:
                imp_id = _as_int(aircraft.get("individualMissionPackageID"))
                ac_id = _as_int(aircraft.get("aircraftID"))
                if imp_id is None or imp_id in seen_packages:
                    continue
                seen_packages.add(imp_id)
                mapped_aircraft.append({"package_id": imp_id, "aircraft_id": ac_id})
                self.individual_package_to_plans[imp_id].add(plan_id)
                if pkg_id is not None:
                    self.individual_package_to_packages[imp_id].add(pkg_id)

            self.plan_to_individual_packages[plan_id] = mapped_aircraft
            self.mission_plans[plan_id] = {
                "path": path,
                "data": mp,
                "package_id": pkg_id,
                "aircraft_map": mapped_aircraft,
            }

        # ensure plan lists sorted for deterministic UI
        for pkg_id, plans in self.package_to_plans.items():
            self.package_to_plans[pkg_id] = sorted(set(plans))

    # Individual mission package loading ---------------------------------
    def _load_individual_packages(self) -> None:
        imp_dir = self.db_root / "IndividualMissionPlan"
        if not imp_dir.exists():
            return

        for path in sorted(imp_dir.glob("*.json")):
            imp = self._safe_load(path, "[IndividualMissionPlan]")
            if imp is None:
                continue

            imp_id = _as_int(imp.get("individualMissionPackageID"))
            if imp_id is None:
                self._note_error(f"[IndividualMissionPlan] {path.name}: missing individualMissionPackageID")
                continue

            aircraft_id = _as_int(imp.get("aircraftID"))
            plan_ids = sorted(self.individual_package_to_plans.get(imp_id, set()))
            package_ids = sorted(self.individual_package_to_packages.get(imp_id, set()))
            missions = {}
            entries: List[Dict[str, Any]] = []

            for mission in imp.get("individualMissionList") or []:
                individual_id = _as_int(mission.get("individualMissionID"))
                if individual_id is None:
                    continue
                path_id = _as_int(mission.get("pathID"))
                related = mission.get("relatedMission") or {}
                input_id = _as_int(related.get("inputMissionID"))

                candidate_packages = list(package_ids)
                if not candidate_packages and input_id is not None:
                    candidate_packages = self._infer_unique_package(input_id)

                entry = {
                    "individual_id": individual_id,
                    "input_mission_id": input_id,
                    "package_id": imp_id,
                    "aircraft_id": aircraft_id,
                    "path_id": path_id,
                    "plan_ids": plan_ids,
                    "input_packages": candidate_packages,
                    "raw": mission,
                    "file_path": path,
                }
                entries.append(entry)
                missions[individual_id] = mission
                self.individual_entries[individual_id] = entry
                if path_id is not None:
                    self.path_entries[path_id].append(entry)

                for pkg_id in candidate_packages:
                    if pkg_id is not None and input_id is not None:
                        key = (pkg_id, input_id)
                        self.input_to_individuals.setdefault(key, []).append(entry)

            self.individual_packages[imp_id] = {
                "path": path,
                "data": imp,
                "aircraft_id": aircraft_id,
                "plan_ids": plan_ids,
                "input_packages": package_ids,
                "missions": missions,
                "entries": entries,
            }

    # Helpers -------------------------------------------------------------
    def _safe_load(self, path: Path, prefix: str) -> Optional[Dict[str, Any]]:
        try:
            with path.open("r", encoding="utf-8") as fh:
                return json.load(fh)
        except Exception as exc:
            self._note_error(f"{prefix} {path.name}: {exc}")
            return None

    def _note_error(self, msg: str) -> None:
        self.errors.append(msg)

    def _infer_unique_package(self, input_id: int) -> List[int]:
        matches = [pkg_id for pkg_id, pkg in self.packages.items() if input_id in pkg["missions"]]
        return matches if len(matches) == 1 else []

    def filter_scope(self, scope: Optional[Dict[str, Set[int]]]) -> None:
        if not scope:
            return

        packages_scope = scope.get("packages")
        plans_scope = scope.get("plans")
        imps_scope = scope.get("individual_packages")
        path_scope = scope.get("paths")

        def _resolve_allowed(current_keys: Iterable[int], requested: Optional[Set[int]]) -> Set[int]:
            current_set = set(current_keys)
            if requested is None:
                return current_set
            return current_set & set(requested)

        allowed_packages = _resolve_allowed(self.packages.keys(), packages_scope)
        allowed_plans = _resolve_allowed(self.mission_plans.keys(), plans_scope)
        allowed_imps = _resolve_allowed(self.individual_packages.keys(), imps_scope)

        # Filter packages
        self.packages = {pkg_id: data for pkg_id, data in self.packages.items() if pkg_id in allowed_packages}

        # Filter package->plans mapping
        filtered_pkg_to_plans = defaultdict(list)
        for pkg_id, plans in self.package_to_plans.items():
            if pkg_id not in allowed_packages:
                continue
            filtered = [pid for pid in plans if pid in allowed_plans]
            if filtered:
                filtered_pkg_to_plans[pkg_id] = filtered
        self.package_to_plans = filtered_pkg_to_plans

        # Filter mission plans and ensure package relationship
        filtered_mission_plans: Dict[int, Dict[str, Any]] = {}
        for plan_id, mp in self.mission_plans.items():
            if plan_id not in allowed_plans:
                continue
            pkg_id = mp.get("package_id")
            if pkg_id is not None and allowed_packages and pkg_id not in allowed_packages:
                continue
            filtered_mission_plans[plan_id] = mp
        self.mission_plans = filtered_mission_plans
        allowed_plans = set(self.mission_plans.keys())
        for pkg_id in list(self.package_to_plans.keys()):
            plans = [pid for pid in self.package_to_plans[pkg_id] if pid in allowed_plans]
            self.package_to_plans[pkg_id] = plans

        # Filter plan -> individual mapping
        filtered_plan_to_imp: Dict[int, List[Dict[str, Optional[int]]]] = {}
        for plan_id, items in self.plan_to_individual_packages.items():
            if plan_id not in allowed_plans:
                continue
            filtered_items = []
            for item in items:
                imp_id = item.get("package_id")
                if imps_scope is not None and imp_id is not None and imp_id not in allowed_imps:
                    continue
                filtered_items.append(item)
            if filtered_items:
                filtered_plan_to_imp[plan_id] = filtered_items
        self.plan_to_individual_packages = filtered_plan_to_imp

        # Individual package associations
        filtered_imp_to_plans = defaultdict(set)
        for imp_id, plans in self.individual_package_to_plans.items():
            if imps_scope is not None and imp_id not in allowed_imps:
                continue
            filtered = {pid for pid in plans if pid in allowed_plans}
            if filtered:
                filtered_imp_to_plans[imp_id] = filtered
        self.individual_package_to_plans = filtered_imp_to_plans

        filtered_imp_to_packages = defaultdict(set)
        for imp_id, pkgs in self.individual_package_to_packages.items():
            if imps_scope is not None and imp_id not in allowed_imps:
                continue
            filtered = {pkg for pkg in pkgs if pkg in allowed_packages}
            filtered_imp_to_packages[imp_id] = filtered
        self.individual_package_to_packages = filtered_imp_to_packages

        # Determine allowed individual packages after relationship pruning
        if allowed_imps:
            allowed_imps = {imp_id for imp_id in allowed_imps if imp_id in self.individual_package_to_plans}
        else:
            allowed_imps = set(self.individual_package_to_plans.keys())

        self.individual_packages = {
            imp_id: data for imp_id, data in self.individual_packages.items() if imp_id in allowed_imps
        }

        # Filter input_to_individuals
        filtered_input_to_inds = defaultdict(list)
        for (pkg_id, input_id), entries in self.input_to_individuals.items():
            if packages_scope is not None and pkg_id not in allowed_packages:
                continue
            filtered_entries = [
                entry for entry in entries
                if (not allowed_imps or entry.get("package_id") in allowed_imps)
                and (not allowed_plans or not entry.get("plan_ids") or any(pid in allowed_plans for pid in entry.get("plan_ids", [])))
            ]
            if filtered_entries:
                filtered_input_to_inds[(pkg_id, input_id)] = filtered_entries
        self.input_to_individuals = filtered_input_to_inds

        # Filter individual entries
        filtered_individual_entries: Dict[int, Dict[str, Any]] = {}
        for ind_id, entry in self.individual_entries.items():
            imp_id = entry.get("package_id")
            pkg_ids = entry.get("input_packages") or []
            plan_ids = entry.get("plan_ids") or []
            if imps_scope is not None and imp_id not in allowed_imps:
                continue
            if packages_scope is not None and pkg_ids and not any(pkg in allowed_packages for pkg in pkg_ids):
                continue
            if plans_scope is not None and plan_ids and not any(pid in allowed_plans for pid in plan_ids):
                continue
            filtered_individual_entries[ind_id] = entry
        self.individual_entries = filtered_individual_entries

        # Filter path entries
        if path_scope is not None:
            allowed_paths = set(path_scope)
        else:
            allowed_paths = {
                path_id
                for path_id, entries in self.path_entries.items()
                for entry in entries
                if entry.get("package_id") in self.individual_packages
            }

        filtered_paths = defaultdict(list)
        for path_id, entries in self.path_entries.items():
            if path_id not in allowed_paths:
                continue
            filtered_entries = [
                entry for entry in entries
                if entry.get("package_id") in self.individual_packages
                and (
                    not allowed_plans
                    or not entry.get("plan_ids")
                    or any(pid in allowed_plans for pid in entry.get("plan_ids", []))
                )
            ]
            if filtered_entries:
                filtered_paths[path_id] = filtered_entries
        self.path_entries = filtered_paths

    # Query helpers -------------------------------------------------------
    def get_package_ids(self) -> List[int]:
        return sorted(self.packages.keys())

    def get_input_missions(self, package_id: int) -> List[Tuple[int, Dict[str, Any]]]:
        pkg = self.packages.get(package_id)
        if not pkg:
            return []
        return sorted(pkg["missions"].items())

    def get_plan_ids_for_package(self, package_id: int) -> List[int]:
        return sorted(self.package_to_plans.get(package_id, []))

    def get_individual_packages_for_package(self, package_id: int) -> List[Tuple[int, Optional[int]]]:
        result: Dict[int, Optional[int]] = {}
        for plan_id in self.package_to_plans.get(package_id, []):
            for item in self.plan_to_individual_packages.get(plan_id, []):
                imp_id = item.get("package_id")
                if imp_id is None:
                    continue
                if imp_id not in result:
                    result[imp_id] = item.get("aircraft_id")
        return sorted(result.items())

    def get_individual_packages_for_plan(self, plan_id: int) -> List[Tuple[int, Optional[int]]]:
        out = []
        for item in self.plan_to_individual_packages.get(plan_id, []):
            imp_id = item.get("package_id")
            if imp_id is None:
                continue
            out.append((imp_id, item.get("aircraft_id")))
        return sorted(out)

    def get_individual_entries(
        self,
        package_id: Optional[int] = None,
        input_mission_id: Optional[int] = None,
        plan_id: Optional[int] = None,
        individual_package_id: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        entries: List[Dict[str, Any]] = []

        if package_id is not None and input_mission_id is not None:
            for entry in self.input_to_individuals.get((package_id, input_mission_id), []):
                if plan_id is not None and plan_id not in entry["plan_ids"]:
                    continue
                if individual_package_id is not None and entry["package_id"] != individual_package_id:
                    continue
                entries.append(entry)
        elif individual_package_id is not None:
            imp = self.individual_packages.get(individual_package_id)
            if imp:
                for entry in imp["entries"]:
                    if package_id is not None and package_id not in entry["input_packages"]:
                        continue
                    if plan_id is not None and plan_id not in entry["plan_ids"]:
                        continue
                    entries.append(entry)
        elif plan_id is not None:
            for imp_id, _ in self.get_individual_packages_for_plan(plan_id):
                imp = self.individual_packages.get(imp_id)
                if not imp:
                    continue
                for entry in imp["entries"]:
                    if package_id is not None and package_id not in entry["input_packages"]:
                        continue
                    entries.append(entry)

        entries.sort(key=lambda item: (
            item.get("input_mission_id") or 0,
            item.get("individual_id") or 0,
        ))
        return entries


class _SignalBlocker:
    def __init__(self, widget: QWidget) -> None:
        self.widget = widget

    def __enter__(self) -> None:
        self.widget.blockSignals(True)

    def __exit__(self, exc_type, exc, tb) -> None:
        self.widget.blockSignals(False)


class IdRelationshipTab(QWidget):
    """Interactive explorer that links mission planning IDs."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.db_root = Path(db_paths.bootstrap_db_root())
        self.cache = RelationshipCache(self.db_root)
        self._session_scope: Optional[Dict[str, Set[int]]] = None
        self._selected_package_id: Optional[int] = None
        self._selected_input_id: Optional[int] = None
        self._selected_plan_id: Optional[int] = None
        self._selected_individual_package_id: Optional[int] = None
        self._selected_individual_id: Optional[int] = None
        self._selected_path_id: Optional[int] = None
        self._pending_input_id: Optional[int] = None
        self._pending_plan_id: Optional[int] = None
        self._pending_individual_package_id: Optional[int] = None
        self._pending_individual_id: Optional[int] = None
        self._pending_path_id: Optional[int] = None
        self._last_refresh: Optional[datetime] = None
        self._current_entries_base: List[Dict[str, Any]] = []
        self._input_status = {
            "0201": None,
            "0203": None,
            "plan": "임무계획 전",
        }

        self._build_ui()
        self.refresh()

    # UI construction -----------------------------------------------------
    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        header = QHBoxLayout()
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh)
        header.addWidget(self.refresh_btn, 0)

        self.summary_label = QLabel("")
        self.summary_label.setStyleSheet("font-weight:600;")
        header.addWidget(self.summary_label, 1)

        self.scope_label = QLabel("0201: 미수신 | 0203: 미수신 | Plan: 임무계획 전")
        self.scope_label.setStyleSheet("color:#246;")
        header.addWidget(self.scope_label, 1)

        self.selection_label = QLabel("No selection")
        self.selection_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.selection_label.setStyleSheet("color:#456;")
        header.addWidget(self.selection_label, 1)

        layout.addLayout(header)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        self.package_list = self._make_list("Input Mission Package ID")
        self.package_list.currentItemChanged.connect(self._on_package_changed)
        splitter.addWidget(self._wrap_list("Input Packages", self.package_list))

        self.input_list = self._make_list("Input Mission ID")
        self.input_list.currentItemChanged.connect(self._on_input_mission_changed)
        splitter.addWidget(self._wrap_list("Input Missions", self.input_list))

        self.plan_list = self._make_list("Mission Plan ID")
        self.plan_list.currentItemChanged.connect(self._on_plan_changed)
        splitter.addWidget(self._wrap_list("Mission Plans", self.plan_list))

        self.individual_package_list = self._make_list("Individual Mission Package ID")
        self.individual_package_list.currentItemChanged.connect(self._on_individual_package_changed)
        splitter.addWidget(self._wrap_list("Individual Packages", self.individual_package_list))

        self.individual_list = self._make_list("Individual Mission ID")
        self.individual_list.currentItemChanged.connect(self._on_individual_changed)
        splitter.addWidget(self._wrap_list("Individual Missions", self.individual_list))

        self.path_list = self._make_list("Path ID")
        self.path_list.currentItemChanged.connect(self._on_path_changed)
        splitter.addWidget(self._wrap_list("Path IDs", self.path_list))

        layout.addWidget(splitter, 1)

        detail_group = QGroupBox("Details")
        detail_layout = QVBoxLayout(detail_group)
        self.detail_title = QLabel("")
        self.detail_body = QTextEdit()
        self.detail_body.setReadOnly(True)
        self.detail_body.setLineWrapMode(QTextEdit.NoWrap)
        self.detail_body.setPlaceholderText("Select an item to inspect its raw payload.")
        detail_layout.addWidget(self.detail_title)
        detail_layout.addWidget(self.detail_body, 1)
        layout.addWidget(detail_group, 1)

    def _make_list(self, placeholder: str) -> QListWidget:
        widget = QListWidget()
        widget.setSelectionMode(QListWidget.SingleSelection)
        widget.setAlternatingRowColors(True)
        widget.setSortingEnabled(False)
        widget.setUniformItemSizes(False)
        widget.setWordWrap(False)
        widget.setObjectName(placeholder.replace(" ", "_").lower())
        # narrower minimum so the Mission Planning window can shrink comfortably
        widget.setMinimumWidth(50)
        return widget

    def _wrap_list(self, title: str, widget: QListWidget) -> QWidget:
        box = QGroupBox(title)
        layout = QVBoxLayout(box)
        layout.addWidget(widget, 1)
        return box

    # Public API ----------------------------------------------------------
    def refresh(self) -> None:
        prev_package = self._selected_package_id
        prev_input = self._selected_input_id
        prev_plan = self._selected_plan_id
        prev_imp = self._selected_individual_package_id
        prev_individual = self._selected_individual_id
        prev_path = self._selected_path_id

        self.cache.refresh()
        self.cache.filter_scope(self._session_scope)
        self._last_refresh = datetime.now()

        self._selected_package_id = None
        self._selected_input_id = None
        self._selected_plan_id = None
        self._selected_individual_package_id = None
        self._selected_individual_id = None
        self._selected_path_id = None

        self._pending_input_id = prev_input
        self._pending_plan_id = prev_plan
        self._pending_individual_package_id = prev_imp
        self._pending_individual_id = prev_individual
        self._pending_path_id = prev_path
        self._current_entries_base = []

        self._populate_package_list()
        self._populate_input_list(None)
        self._populate_plan_list(None)
        self._populate_individual_package_list(None)
        self._update_individual_views([])

        summary = [
            f"Packages: {len(self.cache.packages)}",
            f"MissionPlans: {len(self.cache.mission_plans)}",
            f"IndividualPackages: {len(self.cache.individual_packages)}",
            f"Paths: {len(self.cache.path_entries)}",
        ]
        if self.cache.errors:
            summary.append(f"Errors: {len(self.cache.errors)}")
            self.summary_label.setToolTip("\n".join(self.cache.errors[:20]))
        else:
            self.summary_label.setToolTip("")
        if self._last_refresh:
            summary.append(f"Updated: {self._last_refresh.strftime('%H:%M:%S')}")
        self.summary_label.setText(" | ".join(summary))
        self._update_status_label()

        self._update_selection_label()
        self._set_detail(None, None)

        if prev_package is not None:
            if not self._select_list_row(self.package_list, prev_package):
                self._auto_select_first(self.package_list)
        else:
            self._auto_select_first(self.package_list)

    # List population helpers --------------------------------------------
    def _populate_package_list(self) -> None:
        with _SignalBlocker(self.package_list):
            self.package_list.clear()
            for pkg_id in self.cache.get_package_ids():
                pkg = self.cache.packages.get(pkg_id, {})
                mission_count = len(pkg.get("missions", {}))
                item = QListWidgetItem(f"{pkg_id}  (missions {mission_count})")
                item.setData(Qt.UserRole, pkg_id)
                path = pkg.get("path")
                if isinstance(path, Path):
                    item.setToolTip(f"File: {self._relative_path(path)}")
                self.package_list.addItem(item)

    def _populate_input_list(self, package_id: Optional[int]) -> None:
        with _SignalBlocker(self.input_list):
            self.input_list.clear()
            if package_id is None:
                return
            for input_id, mission in self.cache.get_input_missions(package_id):
                detail = mission.get("missionDetail") or {}
                kind = next(iter(detail.keys()), "-") if isinstance(detail, dict) else "-"
                item = QListWidgetItem(f"{input_id}  (detail {kind})")
                item.setData(Qt.UserRole, (package_id, input_id))
                item.setToolTip(self._format_tooltip_lines({
                    "MissionType": mission.get("inputMissionType"),
                    "DetailKey": kind,
                }))
                self.input_list.addItem(item)
            if self._pending_input_id is not None:
                self._select_list_row(self.input_list, (package_id, self._pending_input_id))
                self._pending_input_id = None
            elif self.input_list.count():
                self._auto_select_first(self.input_list)

    def _populate_plan_list(self, package_id: Optional[int]) -> None:
        with _SignalBlocker(self.plan_list):
            self.plan_list.clear()
            if package_id is None:
                return
            for plan_id in self.cache.get_plan_ids_for_package(package_id):
                aircraft = self.cache.plan_to_individual_packages.get(plan_id, [])
                item = QListWidgetItem(f"{plan_id}  (aircraft {len(aircraft)})")
                item.setData(Qt.UserRole, plan_id)
                mp = self.cache.mission_plans.get(plan_id)
                if mp and mp.get("path"):
                    item.setToolTip(f"File: {self._relative_path(mp['path'])}")
                self.plan_list.addItem(item)
            if self._pending_plan_id is not None:
                self._select_list_row(self.plan_list, self._pending_plan_id)
                self._pending_plan_id = None
            elif self.plan_list.count():
                self._auto_select_first(self.plan_list)

    def _populate_individual_package_list(self, plan_id: Optional[int]) -> None:
        with _SignalBlocker(self.individual_package_list):
            self.individual_package_list.clear()
            candidates: Iterable[Tuple[int, Optional[int]]] = ()
            if plan_id is not None:
                candidates = self.cache.get_individual_packages_for_plan(plan_id)
            elif self._selected_package_id is not None:
                candidates = self.cache.get_individual_packages_for_package(self._selected_package_id)

            for pkg_id, aircraft_id in candidates:
                label = f"{pkg_id}"
                if aircraft_id is not None:
                    label += f"  (aircraft {aircraft_id})"
                item = QListWidgetItem(label)
                item.setData(Qt.UserRole, pkg_id)
                imp = self.cache.individual_packages.get(pkg_id)
                tooltip_data = {
                    "Path": self._relative_path(imp["path"]) if imp and imp.get("path") else "-",
                    "Plans": ", ".join(str(pid) for pid in (imp or {}).get("plan_ids", [])) or "-",
                }
                item.setToolTip(self._format_tooltip_lines(tooltip_data))
                self.individual_package_list.addItem(item)

            if self._pending_individual_package_id is not None:
                self._select_list_row(self.individual_package_list, self._pending_individual_package_id)
                self._pending_individual_package_id = None

    def _populate_path_list(self, entries: Iterable[Dict[str, Any]]) -> None:
        entries = list(entries)
        path_info: Dict[int, Dict[str, Any]] = {}
        for entry in entries:
            path_id = entry.get("path_id")
            if path_id is None:
                continue
            info = path_info.setdefault(path_id, {
                "count": 0,
                "individuals": set(),
                "input_ids": set(),
                "individual_packages": set(),
                "input_packages": set(),
                "plan_ids": set(),
                "aircraft": set(),
            })
            info["count"] += 1
            if entry.get("individual_id") is not None:
                info["individuals"].add(entry["individual_id"])
            if entry.get("input_mission_id") is not None:
                info["input_ids"].add(entry["input_mission_id"])
            if entry.get("package_id") is not None:
                info["individual_packages"].add(entry["package_id"])
            for pkg in entry.get("input_packages") or []:
                if pkg is not None:
                    info["input_packages"].add(pkg)
            for plan in entry.get("plan_ids") or []:
                if plan is not None:
                    info["plan_ids"].add(plan)
            if entry.get("aircraft_id") is not None:
                info["aircraft"].add(entry["aircraft_id"])

        with _SignalBlocker(self.path_list):
            self.path_list.clear()
            for path_id in sorted(path_info.keys()):
                info = path_info[path_id]
                label = f"{path_id}  (missions {info['count']})"
                item = QListWidgetItem(label)
                item.setData(Qt.UserRole, path_id)
                tooltip_data = {
                    "Individuals": self._join_ids(info["individuals"]),
                    "InputMissions": self._join_ids(info["input_ids"]),
                    "IndividualPackages": self._join_ids(info["individual_packages"]),
                    "InputPackages": self._join_ids(info["input_packages"]),
                    "MissionPlans": self._join_ids(info["plan_ids"]),
                    "Aircraft": self._join_ids(info["aircraft"]),
                }
                item.setToolTip(self._format_tooltip_lines(tooltip_data))
                self.path_list.addItem(item)

            target = self._pending_path_id if self._pending_path_id is not None else self._selected_path_id
            if target is not None:
                if not self._select_list_row(self.path_list, target):
                    if target == self._selected_path_id:
                        self._selected_path_id = None
            self._pending_path_id = None
            if self.path_list.count() == 0:
                self._selected_path_id = None

    def _filter_entries_by_path(self, entries: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
        base = list(entries)
        if self._selected_path_id is None:
            return base
        return [entry for entry in base if entry.get("path_id") == self._selected_path_id]

    def _update_individual_views(self, base_entries: Iterable[Dict[str, Any]]) -> None:
        base = list(base_entries)
        self._current_entries_base = base
        available_paths = {entry.get("path_id") for entry in base if entry.get("path_id") is not None}
        if self._selected_path_id not in available_paths:
            if self._selected_path_id is not None:
                self._selected_path_id = None
                self._pending_path_id = None
        elif self._pending_path_id is None and self._selected_path_id is not None:
            self._pending_path_id = self._selected_path_id
        self._populate_path_list(base)
        filtered = self._filter_entries_by_path(base)
        self._populate_individual_list(filtered)

    def _populate_individual_list(self, entries: Iterable[Dict[str, Any]]) -> None:
        entries = list(entries)
        with _SignalBlocker(self.individual_list):
            self.individual_list.clear()
            for entry in entries:
                individual_id = entry.get("individual_id")
                input_id = entry.get("input_mission_id")
                path_id = entry.get("path_id")
                plan_ids = entry.get("plan_ids") or []
                label = f"{individual_id}"
                if input_id is not None:
                    label += f"  (input {input_id})"
                if path_id is not None:
                    label += f"  [path {path_id}]"
                if plan_ids:
                    label += f"  <plan {'/'.join(str(pid) for pid in plan_ids)}>"
                item = QListWidgetItem(label)
                item.setData(Qt.UserRole, individual_id)

                tooltip_data = {
                    "IndividualPackage": entry.get("package_id"),
                    "Aircraft": entry.get("aircraft_id"),
                    "Plans": ", ".join(str(pid) for pid in plan_ids) or "-",
                    "File": self._relative_path(entry["file_path"]) if entry.get("file_path") else "-",
                }
                item.setToolTip(self._format_tooltip_lines(tooltip_data))
                self.individual_list.addItem(item)

            target = self._pending_individual_id if self._pending_individual_id is not None else self._selected_individual_id
            if target is not None:
                if not self._select_list_row(self.individual_list, target):
                    if target == self._selected_individual_id:
                        self._selected_individual_id = None
            self._pending_individual_id = None

    # Selection callbacks -------------------------------------------------
    def _on_package_changed(self, current: Optional[QListWidgetItem], _: Optional[QListWidgetItem]) -> None:
        self._selected_package_id = current.data(Qt.UserRole) if current else None
        self._selected_input_id = None
        self._selected_plan_id = None
        self._selected_individual_package_id = None
        self._selected_individual_id = None
        self._selected_path_id = None
        self._pending_path_id = None
        self._set_detail(None, None)

        self._populate_input_list(self._selected_package_id)
        self._populate_plan_list(self._selected_package_id)
        self._populate_individual_package_list(self._selected_plan_id)
        self._update_individual_views([])
        self._update_selection_label()

    def _on_input_mission_changed(self, current: Optional[QListWidgetItem], _: Optional[QListWidgetItem]) -> None:
        if current is None:
            self._selected_input_id = None
            self._populate_individual_list([])
            self._set_detail(None, None)
            self._update_selection_label()
            return

        pkg_id, input_id = current.data(Qt.UserRole)
        self._selected_package_id = pkg_id
        self._selected_input_id = input_id
        self._selected_path_id = None
        self._pending_path_id = None
        mission = self.cache.packages.get(pkg_id, {}).get("missions", {}).get(input_id)
        pkg_path = self.cache.packages.get(pkg_id, {}).get("path")
        self._set_detail_json(
            f"Input Mission {input_id} (package {pkg_id})",
            pkg_path,
            mission,
        )
        entries = self.cache.get_individual_entries(
            package_id=self._selected_package_id,
            input_mission_id=self._selected_input_id,
            plan_id=self._selected_plan_id,
            individual_package_id=self._selected_individual_package_id,
        )
        self._update_individual_views(entries)
        self._update_selection_label()

    def _on_plan_changed(self, current: Optional[QListWidgetItem], _: Optional[QListWidgetItem]) -> None:
        self._selected_plan_id = current.data(Qt.UserRole) if current else None
        self._selected_individual_package_id = None
        self._selected_individual_id = None
        self._selected_path_id = None
        self._pending_path_id = None

        if self._selected_plan_id is not None:
            mp = self.cache.mission_plans.get(self._selected_plan_id)
            self._set_detail_json(
                f"Mission Plan {self._selected_plan_id}",
                (mp or {}).get("path"),
                (mp or {}).get("data"),
            )
        else:
            self._set_detail(None, None)

        self._populate_individual_package_list(self._selected_plan_id)
        entries = self.cache.get_individual_entries(
            package_id=self._selected_package_id,
            input_mission_id=self._selected_input_id,
            plan_id=self._selected_plan_id,
            individual_package_id=self._selected_individual_package_id,
        )
        self._update_individual_views(entries)
        self._update_selection_label()

    def _on_individual_package_changed(self, current: Optional[QListWidgetItem], _: Optional[QListWidgetItem]) -> None:
        self._selected_individual_package_id = current.data(Qt.UserRole) if current else None
        self._selected_individual_id = None
        self._selected_path_id = None
        self._pending_path_id = None

        if self._selected_individual_package_id is not None:
            imp = self.cache.individual_packages.get(self._selected_individual_package_id)
            self._set_detail_json(
                f"Individual Package {self._selected_individual_package_id}",
                (imp or {}).get("path"),
                (imp or {}).get("data"),
            )
        else:
            self._set_detail(None, None)

        entries = self.cache.get_individual_entries(
            package_id=self._selected_package_id,
            input_mission_id=self._selected_input_id,
            plan_id=self._selected_plan_id,
            individual_package_id=self._selected_individual_package_id,
        )
        self._update_individual_views(entries)
        self._update_selection_label()

    def _on_path_changed(self, current: Optional[QListWidgetItem], _: Optional[QListWidgetItem]) -> None:
        self._selected_path_id = current.data(Qt.UserRole) if current else None
        self._pending_individual_id = None
        self._selected_individual_id = None

        filtered = self._filter_entries_by_path(self._current_entries_base)
        self._populate_individual_list(filtered)

        if self._selected_path_id is not None:
            entries = self.cache.path_entries.get(self._selected_path_id, [])
            info = {
                "Individuals": self._join_ids(entry.get("individual_id") for entry in entries),
                "InputMissions": self._join_ids(entry.get("input_mission_id") for entry in entries),
                "IndividualPackages": self._join_ids(entry.get("package_id") for entry in entries),
                "InputPackages": self._join_ids(
                    pkg for entry in entries for pkg in entry.get("input_packages") or []
                ),
                "MissionPlans": self._join_ids(
                    plan for entry in entries for plan in entry.get("plan_ids") or []
                ),
                "Aircraft": self._join_ids(entry.get("aircraft_id") for entry in entries),
            }
            detail_lines = [self._format_tooltip_lines(info)]
            for entry in entries:
                plans = ", ".join(str(pid) for pid in (entry.get("plan_ids") or [])) or "-"
                input_packages = ", ".join(str(pid) for pid in (entry.get("input_packages") or [])) or "-"
                detail_lines.append(
                    f"- Individual {entry.get('individual_id') or '-'} | "
                    f"Input {entry.get('input_mission_id') or '-'} | "
                    f"IndPkg {entry.get('package_id') or '-'} | "
                    f"InputPkg {input_packages} | Plans {plans} | "
                    f"Aircraft {entry.get('aircraft_id') or '-'}"
                )
            self._set_detail(f"Path {self._selected_path_id}", "\n".join(detail_lines))
        else:
            self._set_detail(None, None)

        self._update_selection_label()

    def _on_individual_changed(self, current: Optional[QListWidgetItem], _: Optional[QListWidgetItem]) -> None:
        self._selected_individual_id = current.data(Qt.UserRole) if current else None
        if self._selected_individual_id is None:
            self._set_detail(None, None)
            self._update_selection_label()
            return

        entry = self.cache.individual_entries.get(self._selected_individual_id)
        if not entry:
            self._set_detail(None, None)
            self._update_selection_label()
            return

        header = {
            "IndividualID": entry.get("individual_id"),
            "InputMissionID": entry.get("input_mission_id"),
            "PathID": entry.get("path_id"),
            "Plans": ", ".join(str(pid) for pid in entry.get("plan_ids") or []) or "-",
            "PackageID": entry.get("package_id"),
            "AircraftID": entry.get("aircraft_id"),
        }
        text_lines = [self._format_tooltip_lines(header), ""]
        try:
            text_lines.append(json.dumps(entry.get("raw"), ensure_ascii=False, indent=2))
        except TypeError:
            text_lines.append(str(entry.get("raw")))
        self._set_detail(f"Individual Mission {entry.get('individual_id')}", "\n".join(text_lines))
        self._update_selection_label()

    # Session scope/status ----------------------------------------------
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
        self.refresh()

    def update_input_status(self, *, cmpk_id: Optional[int] = None, mrpk_id: Optional[int] = None, plan_state: Optional[str] = None) -> None:
        if cmpk_id is not None:
            self._input_status["0201"] = cmpk_id
        if mrpk_id is not None:
            self._input_status["0203"] = mrpk_id
        if plan_state is not None:
            self._input_status["plan"] = plan_state
        self._update_status_label()

    def _update_status_label(self) -> None:
        parts = []
        cmpk = self._input_status.get("0201")
        mrpk = self._input_status.get("0203")
        plan = self._input_status.get("plan") or "임무계획 전"
        parts.append(f"0201: {cmpk if cmpk is not None else '미수신'}")
        parts.append(f"0203: {mrpk if mrpk is not None else '미수신'}")
        parts.append(f"Plan: {plan}")
        if hasattr(self, "scope_label"):
            self.scope_label.setText(" | ".join(parts))

    # Detail helpers ------------------------------------------------------
    def _set_detail(self, title: Optional[str], body: Optional[str]) -> None:
        self.detail_title.setText(title or "")
        if body is None:
            self.detail_body.clear()
        else:
            self.detail_body.setPlainText(body)

    def _set_detail_json(self, title: str, path: Optional[Path], payload: Any) -> None:
        if payload is None:
            self._set_detail(title, "No data available.")
            return
        lines = []
        if path is not None:
            lines.append(f"File: {self._relative_path(path)}")
            lines.append("")
        try:
            lines.append(json.dumps(payload, ensure_ascii=False, indent=2))
        except TypeError:
            lines.append(str(payload))
        self._set_detail(title, "\n".join(lines))

    # Utility -------------------------------------------------------------
    def _relative_path(self, path: Path) -> str:
        try:
            return str(Path(path).relative_to(self.db_root))
        except ValueError:
            return str(path)

    def _join_ids(self, values: Iterable[Any]) -> str:
        ids: Set[int] = set()
        for value in values:
            if value is None:
                continue
            try:
                ids.add(int(value))
            except (TypeError, ValueError):
                continue
        return ", ".join(str(v) for v in sorted(ids)) if ids else "-"

    def _format_tooltip_lines(self, data: Dict[str, Any]) -> str:
        return "\n".join(f"{key}: {value}" for key, value in data.items())

    def _update_selection_label(self) -> None:
        parts = []
        if self._selected_package_id is not None:
            parts.append(f"Package {self._selected_package_id}")
        if self._selected_input_id is not None:
            parts.append(f"Input {self._selected_input_id}")
        if self._selected_plan_id is not None:
            parts.append(f"Plan {self._selected_plan_id}")
        if self._selected_individual_package_id is not None:
            parts.append(f"IndPkg {self._selected_individual_package_id}")
        if self._selected_individual_id is not None:
            parts.append(f"Individual {self._selected_individual_id}")
        if self._selected_path_id is not None:
            parts.append(f"Path {self._selected_path_id}")
        self.selection_label.setText(" > ".join(parts) if parts else "No selection")

    def _select_list_row(self, widget: QListWidget, target_value: Any) -> bool:
        for row in range(widget.count()):
            item = widget.item(row)
            data = item.data(Qt.UserRole)
            if data == target_value:
                widget.setCurrentRow(row)
                return True
            if isinstance(target_value, tuple) and isinstance(data, tuple) and len(data) == len(target_value):
                if all(a == b for a, b in zip(data, target_value)):
                    widget.setCurrentRow(row)
                    return True
        return False

    def _auto_select_first(self, widget: QListWidget) -> None:
        if widget.count():
            widget.setCurrentRow(0)




