# -*- coding: utf-8 -*-
"""Direction-agnostic spatial capture-depth accounting.

One observation source represents one physical traversal/acquisition.  Every
frame from that source is unioned before depth is evaluated, so overlapping
video frames cannot accidentally satisfy a two-capture requirement.  Coverage
depth increases only where *different* acquisition sources overlap.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable

from shapely.geometry import GeometryCollection
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union


def _empty() -> BaseGeometry:
    return GeometryCollection()


def _clean(geometry: BaseGeometry | None) -> BaseGeometry:
    if geometry is None or geometry.is_empty:
        return _empty()
    try:
        if not geometry.is_valid:
            geometry = geometry.buffer(0)
    except Exception:
        return _empty()
    return geometry if geometry is not None and not geometry.is_empty else _empty()


def _union(geometries: Iterable[BaseGeometry | None]) -> BaseGeometry:
    parts = [_clean(geometry) for geometry in geometries]
    parts = [geometry for geometry in parts if not geometry.is_empty]
    if not parts:
        return _empty()
    try:
        return _clean(unary_union(parts))
    except Exception:
        merged = _empty()
        for geometry in parts:
            try:
                merged = geometry if merged.is_empty else merged.union(geometry)
            except Exception:
                continue
        return _clean(merged)


def stable_capture_source_id(
    *,
    aircraft_id: int | None,
    coverage_pass: str | None = None,
    acquisition_id: object | None = None,
    mission_id: int | None = None,
    generation_token: object | None = None,
) -> str:
    """Return a stable traversal key suitable for plan/replan carry-forward.

    Explicit acquisition IDs are authoritative and should be carried across an
    interrupted acquisition.  The fallback includes the individual mission ID
    and, when available, the FlightPath publication generation.  This keeps a
    newly published acquisition distinct even when the 0303 schema roundtrip
    drops custom waypoint extensions.  Reloading the same publication still
    unions into the same source.  Pass is provenance, never an obligation.
    """

    explicit = str(acquisition_id or "").strip()
    aircraft = int(aircraft_id) if aircraft_id is not None else 0
    if explicit:
        if explicit.startswith("aircraft:"):
            return explicit
        return f"aircraft:{aircraft}:acquisition:{explicit}"
    pass_name = str(coverage_pass or "").strip().lower()
    generation = (
        "" if generation_token is None else str(generation_token).strip()
    )
    parts = [f"aircraft:{aircraft}"]
    if mission_id is not None:
        parts.append(f"mission:{int(mission_id)}")
    if generation:
        parts.append(f"generation:{generation}")
    if pass_name:
        parts.append(f"pass:{pass_name}")
    if len(parts) == 1:
        parts.append("capture")
    return ":".join(parts)


@dataclass
class CoverageDepthMetrics:
    required_depth: int
    assignment_area_m2: float
    geometry_by_exact_depth: dict[int, BaseGeometry]
    geometry_at_least_depth: dict[int, BaseGeometry]
    area_m2_by_exact_depth: dict[int, float]
    work_required_m2: float
    work_covered_m2: float
    work_remaining_m2: float
    completed_geometry: BaseGeometry
    remaining_geometry: BaseGeometry
    satisfied: bool


@dataclass
class SpatialCoverageDepthLedger:
    """Union observations per source, then compute spatial capture depth."""

    required_depth: int = 2
    observations_by_source: dict[str, BaseGeometry] = field(default_factory=dict)
    attribution_by_source: dict[str, dict[str, Any]] = field(default_factory=dict)

    def add_observation(
        self,
        source_id: object,
        geometry: BaseGeometry | None,
        *,
        assignment_geometry: BaseGeometry | None = None,
        attribution: dict[str, Any] | None = None,
    ) -> None:
        source = str(source_id or "").strip()
        candidate = _clean(geometry)
        if not source or candidate.is_empty:
            return
        assignment = _clean(assignment_geometry)
        if not assignment.is_empty:
            try:
                candidate = _clean(assignment.intersection(candidate))
            except Exception:
                return
        if candidate.is_empty:
            return
        previous = self.observations_by_source.get(source)
        self.observations_by_source[source] = _union((previous, candidate))
        if attribution:
            merged = dict(self.attribution_by_source.get(source) or {})
            merged.update(attribution)
            self.attribution_by_source[source] = merged

    def clone(self) -> "SpatialCoverageDepthLedger":
        return SpatialCoverageDepthLedger(
            required_depth=max(1, int(self.required_depth)),
            observations_by_source=dict(self.observations_by_source),
            attribution_by_source={
                str(key): dict(value)
                for key, value in self.attribution_by_source.items()
            },
        )

    def metrics(
        self,
        assignment_geometry: BaseGeometry | None,
    ) -> CoverageDepthMetrics:
        assignment = _clean(assignment_geometry)
        required = max(1, int(self.required_depth))
        assignment_area = float(max(0.0, assignment.area or 0.0))
        sources: list[BaseGeometry] = []
        for geometry in self.observations_by_source.values():
            candidate = _clean(geometry)
            if candidate.is_empty:
                continue
            if not assignment.is_empty:
                try:
                    candidate = _clean(assignment.intersection(candidate))
                except Exception:
                    continue
            if not candidate.is_empty:
                sources.append(candidate)

        # Current Area contract requires two observations.  The loop also
        # supports higher configured depths via an incremental exact-band
        # overlay, without treating repeated frames from one source as layers.
        exact: dict[int, BaseGeometry] = {0: assignment}
        for source_geometry in sources:
            updated: dict[int, BaseGeometry] = {}
            max_existing = max(exact) if exact else 0
            for depth in range(max_existing, -1, -1):
                band = _clean(exact.get(depth))
                if band.is_empty:
                    continue
                try:
                    hit = _clean(band.intersection(source_geometry))
                    miss = _clean(band.difference(source_geometry))
                except Exception:
                    hit = _empty()
                    miss = band
                if not miss.is_empty:
                    updated[depth] = _union((updated.get(depth), miss))
                if not hit.is_empty:
                    next_depth = min(required, depth + 1)
                    updated[next_depth] = _union((updated.get(next_depth), hit))
            exact = updated

        for depth in range(required + 1):
            exact.setdefault(depth, _empty())
        at_least: dict[int, BaseGeometry] = {}
        for depth in range(1, required + 1):
            at_least[depth] = _union(
                exact.get(level) for level in range(depth, required + 1)
            )
        completed = _clean(at_least.get(required))
        try:
            remaining = _clean(assignment.difference(completed))
        except Exception:
            remaining = assignment
        areas = {
            depth: float(max(0.0, geometry.area or 0.0))
            for depth, geometry in exact.items()
        }
        work_required = assignment_area * required
        work_covered = sum(
            min(depth, required) * float(area)
            for depth, area in areas.items()
        )
        work_covered = max(0.0, min(work_required, float(work_covered)))
        tolerance = max(0.05, assignment_area * 1e-6)
        return CoverageDepthMetrics(
            required_depth=required,
            assignment_area_m2=assignment_area,
            geometry_by_exact_depth=exact,
            geometry_at_least_depth=at_least,
            area_m2_by_exact_depth=areas,
            work_required_m2=work_required,
            work_covered_m2=work_covered,
            work_remaining_m2=max(0.0, work_required - work_covered),
            completed_geometry=completed,
            remaining_geometry=remaining,
            satisfied=bool(assignment_area > 0.0 and float(remaining.area or 0.0) <= tolerance),
        )
