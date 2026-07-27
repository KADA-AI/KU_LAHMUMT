# -*- coding: utf-8 -*-
from __future__ import annotations

import copy
from collections import Counter
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rasterio
from pyproj import CRS, Transformer

from modules.common.regional_dem import (
    regional_dem_inventory,
    regional_dem_paths,
    select_regional_dem,
)

from .dem_cover import CoverAnalyzer, CoverConfig, DemGrid, Polygon2D


EXPECTED_SOURCE_PATTERN = (
    (1, 3),
    (1, 4),
    (2, 4),
    (2, 6),
    (1, 3),
    (1, 2),
)


NEW_TARGET_REFRESH_SOURCE_PATTERN = (
    (1, 3),
    (1, 4),
    (2, 4),
    (1, 5),
    (2, 5),
    (1, 6),
    (2, 6),
    (1, 5),
    (2, 6),
    (1, 3),
    (1, 2),
)


TYPE1_TARGET_ORDER_CHANGE_REASON = "임무 순서 변경으로 인한 재계획"
_TARGET_REGION_TYPE = 6
_TARGET_AREA_MISSION_TYPE = 2
_COORD_SIGNATURE_DECIMALS = 7
_TYPE1_REVIEW_PREFIX_PATTERN = ((1, 3), (1, 4), (2, 4))
_TYPE1_TARGET_BUNDLE_PATTERN = ((1, 5), (2, 5), (1, 6), (2, 6), (1, 5))
_TYPE1_BETWEEN_TARGET_PATTERN = (1, 4)
_TYPE1_REVIEW_SUFFIX_PATTERN = ((1, 3), (1, 2))


@dataclass(frozen=True)
class ReviewBuildResult:
    payload: dict[str, Any]
    summary: dict[str, Any]


class AntiArmorReviewError(ValueError):
    pass


_ANALYZER_LOCK = threading.RLock()
_ANALYZER_CACHE: tuple[str, int, CoverAnalyzer] | None = None


def _canonical_polygon_signature(coords: Any) -> tuple[tuple[float, float], ...] | None:
    if not isinstance(coords, list):
        return None
    points: list[tuple[float, float]] = []
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        try:
            point = (
                round(float(coord.get("latitude")), _COORD_SIGNATURE_DECIMALS),
                round(float(coord.get("longitude")), _COORD_SIGNATURE_DECIMALS),
            )
        except Exception:
            continue
        if not points or point != points[-1]:
            points.append(point)
    if len(points) >= 2 and points[0] == points[-1]:
        points.pop()
    if len(points) < 3:
        return None

    candidates: list[tuple[tuple[float, float], ...]] = []
    for sequence in (points, list(reversed(points))):
        for offset in range(len(sequence)):
            candidates.append(tuple(sequence[offset:] + sequence[:offset]))
    return min(candidates)


def _target_area_signature(mission: Any) -> tuple[Any, ...] | None:
    if not isinstance(mission, dict):
        return None
    if _safe_int(mission.get("inputMissionType")) != _TARGET_AREA_MISSION_TYPE:
        return None
    if _safe_int(mission.get("regionType")) != _TARGET_REGION_TYPE:
        return None
    detail = mission.get("missionDetail")
    if not isinstance(detail, dict):
        return None

    polygon_rows: list[tuple[bool, tuple[tuple[float, float], ...]]] = []
    area_list = detail.get("areaList")
    if isinstance(area_list, list):
        for area in area_list:
            if not isinstance(area, dict):
                continue
            signature = _canonical_polygon_signature(area.get("coordinateList"))
            if signature is not None:
                polygon_rows.append((bool(area.get("isHole", False)), signature))
    if not polygon_rows:
        signature = _canonical_polygon_signature(detail.get("coordinateList"))
        if signature is not None:
            polygon_rows.append((False, signature))
    if not polygon_rows:
        return None
    return tuple(sorted(polygon_rows))


def _ordered_target_area_rows(payload: Any) -> list[dict[str, Any]]:
    if not isinstance(payload, dict) or _safe_int(payload.get("inputMissionPackageType")) != 1:
        return []
    missions = payload.get("inputMissionList")
    if not isinstance(missions, list):
        return []
    rows: list[dict[str, Any]] = []
    for mission_index, mission in enumerate(missions):
        signature = _target_area_signature(mission)
        if signature is None:
            continue
        rows.append(
            {
                "missionIndex": int(mission_index),
                "inputMissionID": _safe_int(mission.get("inputMissionID")),
                "signature": signature,
            }
        )
    return rows


def detect_anti_armor_target_order_change(
    previous_payload: Any,
    current_payload: Any,
) -> dict[str, Any] | None:
    """Detect a pure ordering change across the same Type-1 target geometries."""

    previous_rows = _ordered_target_area_rows(previous_payload)
    current_rows = _ordered_target_area_rows(current_payload)
    if len(previous_rows) < 2 or len(previous_rows) != len(current_rows):
        return None

    previous_order = [row["signature"] for row in previous_rows]
    current_order = [row["signature"] for row in current_rows]
    if previous_order == current_order:
        return None
    if Counter(previous_order) != Counter(current_order):
        return None

    return {
        "reason": TYPE1_TARGET_ORDER_CHANGE_REASON,
        "targetCount": len(current_rows),
        "previousTargetInputMissionIDs": [row["inputMissionID"] for row in previous_rows],
        "currentTargetInputMissionIDs": [row["inputMissionID"] for row in current_rows],
        "previousTargetMissionIndexes": [row["missionIndex"] for row in previous_rows],
        "currentTargetMissionIndexes": [row["missionIndex"] for row in current_rows],
    }


def _mission_pattern_row(mission: Any) -> tuple[int | None, int | None]:
    if not isinstance(mission, dict):
        return (None, None)
    return (
        _safe_int(mission.get("inputMissionType")),
        _safe_int(mission.get("regionType")),
    )


def is_anti_armor_air_strike_review_source(payload: Any) -> bool:
    """Return True only for the six-mission Type-1 package this review owns."""

    if not isinstance(payload, dict) or _safe_int(payload.get("inputMissionPackageType")) != 1:
        return False
    missions = payload.get("inputMissionList")
    if not isinstance(missions, list) or len(missions) != len(EXPECTED_SOURCE_PATTERN):
        return False
    return tuple(_mission_pattern_row(mission) for mission in missions) == EXPECTED_SOURCE_PATTERN


def _parse_anti_armor_target_bundles(payload: Any) -> dict[str, Any]:
    """Parse the reviewed Type-1 mission sequence into reusable target sorties."""

    if not isinstance(payload, dict) or _safe_int(payload.get("inputMissionPackageType")) != 1:
        raise AntiArmorReviewError("target ordering requires a Type-1 input mission package")
    missions = payload.get("inputMissionList")
    if not isinstance(missions, list) or len(missions) < 10:
        raise AntiArmorReviewError("reviewed Type-1 mission sequence is unavailable")
    if tuple(_mission_pattern_row(row) for row in missions[:3]) != _TYPE1_REVIEW_PREFIX_PATTERN:
        raise AntiArmorReviewError("unexpected Type-1 reviewed mission prefix")
    if tuple(_mission_pattern_row(row) for row in missions[-2:]) != _TYPE1_REVIEW_SUFFIX_PATTERN:
        raise AntiArmorReviewError("unexpected Type-1 reviewed mission suffix")

    body_end = len(missions) - 2
    cursor = 3
    bundles: list[dict[str, Any]] = []
    transitions: list[dict[str, Any]] = []
    while cursor < body_end:
        if bundles:
            if cursor >= body_end or _mission_pattern_row(missions[cursor]) != _TYPE1_BETWEEN_TARGET_PATTERN:
                raise AntiArmorReviewError("unexpected Type-1 target transition mission")
            transitions.append(missions[cursor])
            cursor += 1
        if cursor + len(_TYPE1_TARGET_BUNDLE_PATTERN) > body_end:
            raise AntiArmorReviewError("incomplete Type-1 target mission bundle")
        rows = missions[cursor : cursor + len(_TYPE1_TARGET_BUNDLE_PATTERN)]
        if tuple(_mission_pattern_row(row) for row in rows) != _TYPE1_TARGET_BUNDLE_PATTERN:
            raise AntiArmorReviewError("unexpected Type-1 target mission bundle")
        target_id = _safe_int(rows[3].get("inputMissionID"))
        battle_id = _safe_int(rows[1].get("inputMissionID"))
        if target_id is None or target_id <= 0 or battle_id is None or battle_id <= 0:
            raise AntiArmorReviewError("target/battle inputMissionID is missing")
        bundles.append(
            {
                "targetInputMissionID": int(target_id),
                "battleInputMissionID": int(battle_id),
                "attackWaitToBattle": rows[0],
                "battleArea": rows[1],
                "battleToTarget": rows[2],
                "targetArea": rows[3],
                "targetToBattle": rows[4],
            }
        )
        cursor += len(_TYPE1_TARGET_BUNDLE_PATTERN)
    if cursor != body_end or not bundles or len(transitions) != max(0, len(bundles) - 1):
        raise AntiArmorReviewError("invalid Type-1 target mission sequence")

    target_ids = [int(bundle["targetInputMissionID"]) for bundle in bundles]
    if len(set(target_ids)) != len(target_ids):
        raise AntiArmorReviewError("target inputMissionID values must be unique")
    return {
        "missions": missions,
        "prefix": missions[:3],
        "bundles": bundles,
        "transitions": transitions,
        "suffix": missions[-2:],
    }


def describe_anti_armor_target_order(payload: Any) -> dict[str, Any]:
    """Return map-ready target polygons from a reviewed Type-1 mission plan."""

    parsed = _parse_anti_armor_target_bundles(payload)
    targets: list[dict[str, Any]] = []
    for order, bundle in enumerate(parsed["bundles"], start=1):
        coords = _first_area_coords(bundle["targetArea"], min_len=3)
        targets.append(
            {
                "order": int(order),
                "targetInputMissionID": int(bundle["targetInputMissionID"]),
                "battleInputMissionID": int(bundle["battleInputMissionID"]),
                "coordinateList": copy.deepcopy(coords),
                "centroid": {
                    "latitude": float(sum(float(row["latitude"]) for row in coords) / len(coords)),
                    "longitude": float(sum(float(row["longitude"]) for row in coords) / len(coords)),
                },
            }
        )
    return {
        "targetCount": len(targets),
        "targetInputMissionIDs": [int(row["targetInputMissionID"]) for row in targets],
        "targets": targets,
    }


def build_anti_armor_target_order_payload(
    source_payload: dict[str, Any],
    *,
    ordered_target_input_mission_ids: list[int],
    new_package_id: int,
    timestamp_ms: int,
) -> ReviewBuildResult:
    """Reorder complete Type-1 target sorties and reconnect their transit legs."""

    parsed = _parse_anti_armor_target_bundles(source_payload)
    bundles = list(parsed["bundles"])
    current_ids = [int(bundle["targetInputMissionID"]) for bundle in bundles]
    requested_ids = [int(value) for value in ordered_target_input_mission_ids]
    if len(requested_ids) != len(current_ids) or len(set(requested_ids)) != len(requested_ids):
        raise AntiArmorReviewError("target order must contain every target exactly once")
    if set(requested_ids) != set(current_ids):
        raise AntiArmorReviewError("target order does not match the current target set")
    if requested_ids == current_ids:
        raise AntiArmorReviewError("target order is unchanged")

    by_target_id = {int(bundle["targetInputMissionID"]): bundle for bundle in bundles}
    ordered_bundles = [by_target_id[target_id] for target_id in requested_ids]
    attack_wait_coords = _first_area_coords(parsed["prefix"][2], min_len=3)
    suffix_coords = _first_line_coords(parsed["suffix"][0], min_len=2)
    acp_coord = suffix_coords[-1]
    all_coords: list[dict[str, Any]] = [*attack_wait_coords, acp_coord]
    for bundle in ordered_bundles:
        all_coords.extend(_first_area_coords(bundle["battleArea"], min_len=3))
        all_coords.extend(_first_area_coords(bundle["targetArea"], min_len=3))
    dem, _analyzer = _get_analyzer_for_coords(all_coords)
    attack_wait_xy = [_coord_to_native(dem, coord) for coord in attack_wait_coords]
    acp_xy = _coord_to_native(dem, acp_coord)
    line_width = _line_width_hint(*parsed["missions"])

    def _reset_clone(mission: dict[str, Any]) -> dict[str, Any]:
        cloned = copy.deepcopy(mission)
        cloned["isDone"] = False
        _null_out_empty_shape_slots(cloned)
        return cloned

    output_missions = [_reset_clone(mission) for mission in parsed["prefix"]]
    previous_battle_xy: list[tuple[float, float]] | None = None
    transitions = list(parsed["transitions"])
    for order_index, bundle in enumerate(ordered_bundles):
        battle_coords = _first_area_coords(bundle["battleArea"], min_len=3)
        target_coords = _first_area_coords(bundle["targetArea"], min_len=3)
        battle_xy = [_coord_to_native(dem, coord) for coord in battle_coords]
        target_xy = [_coord_to_native(dem, coord) for coord in target_coords]
        altitude = _altitude_hint(target_coords or battle_coords)

        if order_index > 0:
            if previous_battle_xy is None:
                raise AntiArmorReviewError("previous target battle area is unavailable")
            transition_template = transitions[order_index - 1]
            transition_id = _safe_int(transition_template.get("inputMissionID"))
            if transition_id is None or transition_id <= 0:
                raise AntiArmorReviewError("target transition inputMissionID is missing")
            output_missions.append(
                _build_line_mission(
                    transition_template,
                    input_id=int(transition_id),
                    mission_type=1,
                    region_type=4,
                    coords=_native_line_to_coords(
                        dem,
                        _boundary_connection(previous_battle_xy, attack_wait_xy),
                        altitude=altitude,
                    ),
                    width=line_width,
                )
            )

        attack_template = bundle["attackWaitToBattle"]
        attack_id = _safe_int(attack_template.get("inputMissionID"))
        to_target_template = bundle["battleToTarget"]
        to_target_id = _safe_int(to_target_template.get("inputMissionID"))
        return_template = bundle["targetToBattle"]
        return_id = _safe_int(return_template.get("inputMissionID"))
        if any(value is None or value <= 0 for value in (attack_id, to_target_id, return_id)):
            raise AntiArmorReviewError("target bundle line inputMissionID is missing")
        output_missions.extend(
            [
                _build_line_mission(
                    attack_template,
                    input_id=int(attack_id),
                    mission_type=1,
                    region_type=5,
                    coords=_native_line_to_coords(
                        dem,
                        _boundary_connection(attack_wait_xy, battle_xy),
                        altitude=altitude,
                    ),
                    width=line_width,
                ),
                _reset_clone(bundle["battleArea"]),
                _build_line_mission(
                    to_target_template,
                    input_id=int(to_target_id),
                    mission_type=1,
                    region_type=6,
                    coords=_native_line_to_coords(
                        dem,
                        _boundary_connection(battle_xy, target_xy),
                        altitude=altitude,
                    ),
                    width=line_width,
                ),
                _reset_clone(bundle["targetArea"]),
                _build_line_mission(
                    return_template,
                    input_id=int(return_id),
                    mission_type=1,
                    region_type=5,
                    coords=_native_line_to_coords(
                        dem,
                        _boundary_connection(target_xy, battle_xy),
                        altitude=altitude,
                    ),
                    width=line_width,
                ),
            ]
        )
        previous_battle_xy = battle_xy

    if previous_battle_xy is None:
        raise AntiArmorReviewError("final target battle area is unavailable")
    suffix_template = parsed["suffix"][0]
    suffix_id = _safe_int(suffix_template.get("inputMissionID"))
    if suffix_id is None or suffix_id <= 0:
        raise AntiArmorReviewError("ACP return inputMissionID is missing")
    output_missions.append(
        _build_line_mission(
            suffix_template,
            input_id=int(suffix_id),
            mission_type=1,
            region_type=3,
            coords=_native_line_to_coords(
                dem,
                _boundary_to_point_connection(previous_battle_xy, acp_xy),
                altitude=_altitude_hint(_first_area_coords(ordered_bundles[-1]["battleArea"], min_len=3)),
            ),
            width=line_width,
        )
    )
    output_missions.append(_reset_clone(parsed["suffix"][1]))
    for mission in output_missions:
        _null_out_empty_shape_slots(mission)

    payload = copy.deepcopy(source_payload)
    payload["inputMissionList"] = output_missions
    for key in ("reviewSource", "reviewedFromInputMissionPackageID", "reviewKind"):
        payload.pop(key, None)
    _set_package_id_and_timestamp(
        payload,
        new_package_id=int(new_package_id),
        timestamp_ms=int(timestamp_ms),
    )
    return ReviewBuildResult(
        payload=payload,
        summary={
            "changeKind": "antiArmorTargetOrderChange",
            "reason": TYPE1_TARGET_ORDER_CHANGE_REASON,
            "targetCount": len(requested_ids),
            "previousTargetInputMissionIDs": current_ids,
            "currentTargetInputMissionIDs": requested_ids,
            "missionCount": len(output_missions),
            "demPath": str(getattr(dem, "path", "") or ""),
        },
    )


def is_anti_armor_new_target_refresh_payload(payload: Any) -> bool:
    if not isinstance(payload, dict):
        return False
    try:
        package_type = int(payload.get("inputMissionPackageType"))
    except Exception:
        return False
    if package_type != 1:
        return False
    missions = payload.get("inputMissionList")
    if not isinstance(missions, list):
        return False
    pattern = tuple(
        (_safe_int(mission.get("inputMissionType")), _safe_int(mission.get("regionType")))
        for mission in missions
        if isinstance(mission, dict)
    )
    return len(pattern) == len(missions) and pattern == NEW_TARGET_REFRESH_SOURCE_PATTERN


def build_anti_armor_new_target_refresh_payload(
    source_payload: dict[str, Any],
    *,
    new_package_id: int,
    timestamp_ms: int,
) -> ReviewBuildResult:
    """Expand a reviewed Type-1 plan when one new target area is appended."""

    if not is_anti_armor_new_target_refresh_payload(source_payload):
        raise AntiArmorReviewError("unexpected type=1 new-target refresh mission pattern")

    source_missions = source_payload.get("inputMissionList")
    assert isinstance(source_missions, list)

    source_ids = [_safe_int(mission.get("inputMissionID")) for mission in source_missions]
    if (
        any(input_id is None or input_id <= 0 for input_id in source_ids)
        or len(set(source_ids)) != len(source_ids)
    ):
        raise AntiArmorReviewError("new-target refresh requires unique positive inputMissionID values")
    next_new_id = max(int(input_id) for input_id in source_ids if input_id is not None) + 1
    old_battle_to_attack_wait_id = int(next_new_id)
    attack_wait_to_new_battle_id = int(next_new_id + 1)
    new_battle_area_id = int(next_new_id + 2)
    new_battle_to_target_id = int(next_new_id + 3)
    target_to_new_battle_id = int(next_new_id + 4)

    attack_wait_area = _first_area_coords(source_missions[2], min_len=3)
    old_battle_area = _first_area_coords(source_missions[4], min_len=3)
    new_target_area = _first_area_coords(source_missions[8], min_len=3)
    old_battle_to_acp_path = _first_line_coords(source_missions[9], min_len=2)
    acp_coord = old_battle_to_acp_path[-1]

    dem, analyzer = _get_analyzer_for_coords(
        [*attack_wait_area, *old_battle_area, *new_target_area, acp_coord]
    )
    altitude = _altitude_hint(new_target_area)
    attack_wait_xy = [_coord_to_native(dem, coord) for coord in attack_wait_area]
    old_battle_xy = [_coord_to_native(dem, coord) for coord in old_battle_area]
    new_target_xy = [_coord_to_native(dem, coord) for coord in new_target_area]
    new_target_polygon = Polygon2D(new_target_xy)
    acp_xy = _coord_to_native(dem, acp_coord)
    enemies = analyzer.sample_enemies(new_target_polygon)
    _require_sampled_enemies(dem, new_target_polygon, enemies)
    analysis_result = analyzer.analyze(
        new_target_polygon,
        enemies,
        analyzer.make_ref(*acp_xy),
    )
    new_battle_xy = [(float(x), float(y)) for x, y in analysis_result.recommended_polygon]
    if len(new_battle_xy) < 3:
        raise AntiArmorReviewError("DEM review produced fewer than 3 new battle-position vertices")

    new_battle_area_coords = _native_polygon_to_coords(
        dem,
        new_battle_xy,
        altitude=altitude,
    )
    new_battle_attack_coord = _native_to_coord(
        dem,
        (float(analysis_result.rep_xy[0]), float(analysis_result.rep_xy[1])),
        altitude=altitude,
    )
    new_battle_fire_coord = None
    if analysis_result.fire_position_xy is not None:
        new_battle_fire_coord = _native_to_coord(
            dem,
            (
                float(analysis_result.fire_position_xy[0]),
                float(analysis_result.fire_position_xy[1]),
            ),
            altitude=altitude,
        )

    old_battle_to_attack_wait_coords = _native_line_to_coords(
        dem,
        _boundary_connection(old_battle_xy, attack_wait_xy),
        altitude=altitude,
    )
    attack_wait_to_new_battle_coords = _native_line_to_coords(
        dem,
        _boundary_connection(attack_wait_xy, new_battle_xy),
        altitude=altitude,
    )
    new_battle_to_target_coords = _native_line_to_coords(
        dem,
        _boundary_connection(new_battle_xy, new_target_xy),
        altitude=altitude,
    )
    new_target_to_battle_coords = _native_line_to_coords(
        dem,
        _boundary_connection(new_target_xy, new_battle_xy),
        altitude=altitude,
    )
    new_battle_to_acp_coords = _native_line_to_coords(
        dem,
        _boundary_to_point_connection(new_battle_xy, acp_xy),
        altitude=altitude,
    )
    line_width = _line_width_hint(
        source_missions[3],
        source_missions[5],
        source_missions[7],
        source_missions[9],
        source_missions[10],
    )

    def _clone(index: int, *, is_done: bool | None = None) -> dict[str, Any]:
        mission = copy.deepcopy(source_missions[index])
        if is_done is not None:
            mission["isDone"] = bool(is_done)
        _null_out_empty_shape_slots(mission)
        return mission

    new_battle_area_mission = _build_area_mission(
        source_missions[4],
        input_id=new_battle_area_id,
        mission_type=2,
        region_type=5,
        coords=new_battle_area_coords,
    )
    new_battle_detail = new_battle_area_mission.setdefault("missionDetail", {})
    if isinstance(new_battle_detail, dict):
        new_battle_detail["battleAttackCoordinate"] = copy.deepcopy(new_battle_attack_coord)
        if new_battle_fire_coord is not None:
            new_battle_detail["battleFireCoordinate"] = copy.deepcopy(new_battle_fire_coord)

    output_missions = [
        # The new target is an additional sortie, not a replacement. Preserve
        # every existing mission and its current completion flag as received.
        *[_clone(index) for index in range(8)],
        _build_line_mission(
            source_missions[7],
            input_id=old_battle_to_attack_wait_id,
            mission_type=1,
            region_type=4,
            coords=old_battle_to_attack_wait_coords,
            width=line_width,
        ),
        _build_line_mission(
            source_missions[3],
            input_id=attack_wait_to_new_battle_id,
            mission_type=1,
            region_type=5,
            coords=attack_wait_to_new_battle_coords,
            width=line_width,
        ),
        new_battle_area_mission,
        _build_line_mission(
            source_missions[5],
            input_id=new_battle_to_target_id,
            mission_type=1,
            region_type=6,
            coords=new_battle_to_target_coords,
            width=line_width,
        ),
        _clone(8, is_done=False),
        _build_line_mission(
            source_missions[7],
            input_id=target_to_new_battle_id,
            mission_type=1,
            region_type=5,
            coords=new_target_to_battle_coords,
            width=line_width,
        ),
        _build_line_mission(
            source_missions[9],
            input_id=int(source_ids[9]),
            mission_type=1,
            region_type=3,
            coords=new_battle_to_acp_coords,
            width=line_width,
        ),
        _clone(10),
    ]
    for mission in output_missions:
        _null_out_empty_shape_slots(mission)

    source_package_id = _safe_int(source_payload.get("inputMissionPackageID"))
    reviewed_payload = copy.deepcopy(source_payload)
    reviewed_payload["inputMissionList"] = output_missions
    reviewed_payload["reviewSource"] = "MSM"
    reviewed_payload["reviewKind"] = "antiArmorNewTargetRefresh"
    if source_package_id is not None and source_package_id > 0:
        reviewed_payload["reviewedFromInputMissionPackageID"] = int(source_package_id)
    _set_package_id_and_timestamp(
        reviewed_payload,
        new_package_id=int(new_package_id),
        timestamp_ms=int(timestamp_ms),
    )

    return ReviewBuildResult(
        payload=reviewed_payload,
        summary={
            "reviewKind": "antiArmorNewTargetRefresh",
            "sourceMissionCount": len(source_missions),
            "reviewedMissionCount": len(output_missions),
            "preservedCompletedInputMissionIDs": [
                int(mission["inputMissionID"])
                for mission in source_missions
                if isinstance(mission, dict) and bool(mission.get("isDone"))
            ],
            "newInputMissionIDs": [
                int(old_battle_to_attack_wait_id),
                int(attack_wait_to_new_battle_id),
                int(new_battle_area_id),
                int(new_battle_to_target_id),
                int(target_to_new_battle_id),
            ],
            "newTargetInputMissionID": int(source_missions[8]["inputMissionID"]),
            "newBattleInputMissionID": int(new_battle_area_id),
            "newBattleAreaVertices": len(new_battle_xy),
            "newBattlePositionCoordinate": copy.deepcopy(new_battle_attack_coord),
            "newBattleFireCoordinate": (
                copy.deepcopy(new_battle_fire_coord)
                if new_battle_fire_coord is not None
                else None
            ),
            "enemyCount": len(enemies),
            "analysisElapsedS": round(float(analysis_result.elapsed_s), 3),
            "demPath": str(getattr(dem, "path", "") or ""),
        },
    )


def build_anti_armor_air_strike_review_payload(
    source_payload: dict[str, Any],
    *,
    new_package_id: int,
    timestamp_ms: int,
) -> ReviewBuildResult:
    if not isinstance(source_payload, dict):
        raise AntiArmorReviewError("source payload is not a dict")

    source_missions = source_payload.get("inputMissionList")
    if not isinstance(source_missions, list):
        raise AntiArmorReviewError("inputMissionList is missing")
    if len(source_missions) != len(EXPECTED_SOURCE_PATTERN):
        raise AntiArmorReviewError(
            f"expected {len(EXPECTED_SOURCE_PATTERN)} source missions, got {len(source_missions)}"
        )

    pattern = tuple((_safe_int(m.get("inputMissionType")), _safe_int(m.get("regionType"))) for m in source_missions)
    if pattern != EXPECTED_SOURCE_PATTERN:
        raise AntiArmorReviewError(f"unexpected type=1 source mission pattern: {pattern}")

    # 검토 패키지의 inputMissionID 규칙: 원본에서 승계된 임무는 원본 ID 를
    # 그대로 유지하고(0902 inputMissionIDList / 0302 RelatedMission 추적 키),
    # 검토로 신설된 임무만 원본 최대 ID+1 부터 이어서 부여한다.
    # 원본 ID 가 누락/중복이면 기존 순번(1..N) 방식으로 폴백한다.
    source_ids = [_safe_int(m.get("inputMissionID")) for m in source_missions]
    if any(sid is None or sid < 0 for sid in source_ids) or len(set(source_ids)) != len(source_ids):
        inherited_ids = list(range(1, len(source_missions) + 1))
        next_new_id = len(source_missions) + 1
    else:
        inherited_ids = list(source_ids)
        next_new_id = max(source_ids) + 1
    new_ids = [next_new_id + offset for offset in range(4)]

    attack_wait_area = _first_area_coords(source_missions[2], min_len=3)
    target_area = _first_area_coords(source_missions[3], min_len=3)
    acp2_path = _first_line_coords(source_missions[4], min_len=2)
    ref_coord = acp2_path[-1]
    dem, analyzer = _get_analyzer_for_coords([*target_area, ref_coord])

    target_xy = [_coord_to_native(dem, coord) for coord in target_area]
    target_polygon = Polygon2D(target_xy)
    ref_xy = _coord_to_native(dem, ref_coord)
    enemies = analyzer.sample_enemies(target_polygon)
    _require_sampled_enemies(dem, target_polygon, enemies)
    ref = analyzer.make_ref(*ref_xy)
    analysis_result = analyzer.analyze(target_polygon, enemies, ref)

    battle_xy = [(float(x), float(y)) for x, y in analysis_result.recommended_polygon]
    if len(battle_xy) < 3:
        raise AntiArmorReviewError("DEM review produced fewer than 3 battle-position vertices")

    altitude = _altitude_hint(target_area)
    attack_wait_xy = [_coord_to_native(dem, coord) for coord in attack_wait_area]

    battle_area_coords = _native_polygon_to_coords(dem, battle_xy, altitude=altitude)
    if len(battle_area_coords) < 3:
        raise AntiArmorReviewError("DEM review produced an invalid battle-position area")
    battle_attack_coord = _native_to_coord(
        dem,
        (float(analysis_result.rep_xy[0]), float(analysis_result.rep_xy[1])),
        altitude=altitude,
    )
    battle_fire_coord = None
    if analysis_result.fire_position_xy is not None:
        battle_fire_coord = _native_to_coord(
            dem,
            (
                float(analysis_result.fire_position_xy[0]),
                float(analysis_result.fire_position_xy[1]),
            ),
            altitude=altitude,
        )

    attack_to_battle_xy = _boundary_connection(attack_wait_xy, battle_xy)
    attack_to_battle_coords = _native_line_to_coords(dem, attack_to_battle_xy, altitude=altitude)
    battle_to_target_coords = _native_line_to_coords(
        dem,
        _boundary_connection(battle_xy, target_xy),
        altitude=altitude,
    )
    target_to_battle_coords = _native_line_to_coords(
        dem,
        _boundary_connection(target_xy, battle_xy),
        altitude=altitude,
    )
    battle_to_acp_coords = _native_line_to_coords(
        dem,
        _boundary_to_point_connection(battle_xy, ref_xy),
        altitude=altitude,
    )

    line_width = _line_width_hint(source_missions[1], source_missions[4], source_missions[5])
    battle_area_mission = _build_area_mission(
        source_missions[3],
        input_id=new_ids[1],
        mission_type=2,
        region_type=5,
        coords=battle_area_coords,
    )
    battle_detail = battle_area_mission.setdefault("missionDetail", {})
    if isinstance(battle_detail, dict):
        battle_detail["battleAttackCoordinate"] = copy.deepcopy(battle_attack_coord)
        if battle_fire_coord is not None:
            battle_detail["battleFireCoordinate"] = copy.deepcopy(battle_fire_coord)

    output_missions = [
        _clone_existing_mission(source_missions[0], input_id=inherited_ids[0], mission_type=1, region_type=3),
        _clone_existing_mission(source_missions[1], input_id=inherited_ids[1], mission_type=1, region_type=4),
        _clone_existing_mission(source_missions[2], input_id=inherited_ids[2], mission_type=2, region_type=4),
        _build_line_mission(
            source_missions[1],
            input_id=new_ids[0],
            mission_type=1,
            region_type=5,
            coords=attack_to_battle_coords,
            width=line_width,
        ),
        battle_area_mission,
        _build_line_mission(
            source_missions[1],
            input_id=new_ids[2],
            mission_type=1,
            region_type=6,
            coords=battle_to_target_coords,
            width=line_width,
        ),
        _clone_existing_mission(source_missions[3], input_id=inherited_ids[3], mission_type=2, region_type=6),
        _build_line_mission(
            source_missions[4],
            input_id=new_ids[3],
            mission_type=1,
            region_type=5,
            coords=target_to_battle_coords,
            width=line_width,
        ),
        _build_line_mission(
            source_missions[4],
            input_id=inherited_ids[4],
            mission_type=1,
            region_type=3,
            coords=battle_to_acp_coords,
            width=line_width,
        ),
        _clone_existing_mission(source_missions[5], input_id=inherited_ids[5], mission_type=1, region_type=2),
    ]

    for mission in output_missions:
        _null_out_empty_shape_slots(mission)

    reviewed_payload = copy.deepcopy(source_payload)
    reviewed_payload["inputMissionList"] = output_missions
    _set_package_id_and_timestamp(
        reviewed_payload,
        new_package_id=int(new_package_id),
        timestamp_ms=int(timestamp_ms),
    )
    # Provenance marker: this file is an MSM review artifact, not an externally
    # received 0201. The review picker must skip these so a stale reviewed file
    # (still inputMissionPackageType=1) is never re-reviewed in a later scenario,
    # which would re-send 0204 in non-type-1 runs.
    reviewed_payload["reviewSource"] = "MSM"
    source_package_id = None
    for key in ("inputMissionPackageID", "InputMissionPackageID", "inputMissionPackageId"):
        try:
            value = int(source_payload.get(key))
        except Exception:
            continue
        if value > 0:
            source_package_id = value
            break
    if source_package_id is not None:
        reviewed_payload["reviewedFromInputMissionPackageID"] = int(source_package_id)

    summary = {
        "sourceMissionCount": len(source_missions),
        "reviewedMissionCount": len(output_missions),
        "inheritedInputMissionIDs": list(inherited_ids),
        "newInputMissionIDs": list(new_ids),
        "battleAreaVertices": len(battle_area_coords),
        "battleAreaM2": round(float(analysis_result.area_m2), 1),
        "enemyCount": len(enemies),
        "demPath": str(analyzer.dem.path),
        "analysisElapsedS": round(float(analysis_result.elapsed_s), 3),
        "hasFireAccess": bool(analysis_result.has_fire_access),
        "battleAttackCoordinate": copy.deepcopy(battle_attack_coord),
        "battleFireCoordinate": copy.deepcopy(battle_fire_coord) if battle_fire_coord is not None else None,
    }
    return ReviewBuildResult(payload=reviewed_payload, summary=summary)


def _get_analyzer_for_coords(coords: list[dict[str, Any]]) -> tuple[DemGrid, CoverAnalyzer]:
    config = _select_dem_config(coords)
    dem_path = str(config.dem_full_path)
    cache_key = (dem_path, int(config.analysis_max_dim))
    global _ANALYZER_CACHE
    with _ANALYZER_LOCK:
        if _ANALYZER_CACHE is None or _ANALYZER_CACHE[0:2] != cache_key:
            dem = DemGrid(
                config.dem_full_path,
                max_dim=int(config.analysis_max_dim),
                fallback_epsg=int(config.dem_fallback_epsg),
            )
            _ANALYZER_CACHE = (dem_path, int(config.analysis_max_dim), CoverAnalyzer(dem, config))
        analyzer = _ANALYZER_CACHE[2]
    return analyzer.dem, analyzer


def _require_sampled_enemies(
    dem: DemGrid,
    polygon: Polygon2D,
    enemies: list[Any],
) -> None:
    if enemies:
        return
    target_bounds = polygon.bounds()
    dem_bounds = (dem.x_min, dem.y_min, dem.x_max, dem.y_max)
    vertices_inside = sum(1 for x, y in polygon.vertices if dem.contains_native(x, y))
    try:
        inside_mask = polygon.contains_points(dem.X, dem.Y)
        target_grid_cells = int(inside_mask.sum())
        valid_target_grid_cells = int((inside_mask & dem.valid).sum())
    except Exception:
        target_grid_cells = 0
        valid_target_grid_cells = 0
    raise AntiArmorReviewError(
        "dem_target_sampling_empty: "
        f"demName={dem.path.name}; demPath={dem.path}; "
        f"targetAreaM2={polygon.area_m2():.1f}; targetNativeBounds={target_bounds}; "
        f"demNativeBounds={dem_bounds}; verticesInsideDem={vertices_inside}/{len(polygon.vertices)}; "
        f"targetGridCells={target_grid_cells}; validTargetGridCells={valid_target_grid_cells}"
    )


def _select_dem_config(coords: list[dict[str, Any]]) -> CoverConfig:
    base = CoverConfig()
    normalized_coords = [coord for coord in (_normalize_coord(value) for value in coords) if coord is not None]
    if not normalized_coords:
        raise AntiArmorReviewError("dem_coordinate_input_empty: no valid latitude/longitude was provided")

    inventory = dict(regional_dem_inventory(base.dem_full_path.parent))
    required_names: list[str] = []
    required_paths: list[str] = []
    missing_required_names: list[str] = []
    missing_required_paths: list[str] = []
    outside_samples: list[str] = []
    first_sample_by_name: dict[str, dict[str, Any]] = {}
    for coord in normalized_coords:
        latitude = float(coord["latitude"])
        longitude = float(coord["longitude"])
        spec = select_regional_dem(latitude, longitude)
        if spec is None:
            outside_samples.append(f"{latitude:.7f},{longitude:.7f}")
            continue
        expected_path = base.dem_full_path.parent / spec.filename
        first_sample_by_name.setdefault(spec.filename, coord)
        if spec.filename not in required_names:
            required_names.append(spec.filename)
            required_paths.append(str(expected_path))
        if not expected_path.is_file() and spec.filename not in missing_required_names:
            missing_required_names.append(spec.filename)
            missing_required_paths.append(str(expected_path))

    if missing_required_names:
        sample = first_sample_by_name.get(missing_required_names[0], normalized_coords[0])
        raise AntiArmorReviewError(
            "required_dem_file_missing: "
            f"expectedDemName={missing_required_names[0]}; "
            f"expectedDemPath={missing_required_paths[0]}; "
            f"sample={float(sample['latitude']):.7f},{float(sample['longitude']):.7f}; "
            f"availableDemNames={list(inventory.get('availableDemNames') or ())}; "
            f"detectedTifNames={list(inventory.get('detectedTifNames') or ())}; "
            f"unregisteredTifNames={list(inventory.get('unregisteredTifNames') or ())}"
        )
    if outside_samples:
        raise AntiArmorReviewError(
            "dem_coordinate_outside_operational_coverage: "
            f"samples={outside_samples[:3]}; requiredDemNames={required_names}; "
            f"availableDemNames={list(inventory.get('availableDemNames') or ())}"
        )
    if len(required_names) > 1:
        raise AntiArmorReviewError(
            "multiple_operational_dem_sources_required: "
            f"requiredDemNames={required_names}; requiredDemPaths={required_paths}"
        )

    candidates = _candidate_dem_paths(base)
    if not candidates:
        raise AntiArmorReviewError(
            "operational_dem_inventory_empty: "
            f"expectedDemPaths={list(inventory.get('expectedDemPaths') or ())}; "
            f"detectedTifNames={list(inventory.get('detectedTifNames') or ())}"
        )
    best_path = base.dem_full_path
    best_count = -1
    required = len(normalized_coords)
    read_errors: list[dict[str, str]] = []
    for path in candidates:
        count = _projected_dem_coverage_count(
            path,
            normalized_coords,
            fallback_epsg=int(base.dem_fallback_epsg),
            errors=read_errors,
        )
        if count is None:
            continue
        if count == required:
            return base.with_overrides(dem_path=str(path))
        if count > best_count:
            best_path = path
            best_count = int(count)
    if read_errors and best_count < required:
        raise AntiArmorReviewError(
            "required_dem_read_error: "
            f"requiredDemNames={required_names}; requiredDemPaths={required_paths}; "
            f"errors={read_errors}; availableDemNames={list(inventory.get('availableDemNames') or ())}"
        )
    raise AntiArmorReviewError(
        "operational_dem_coverage_incomplete: "
        f"requiredCoordinateCount={required}; bestCoverageCount={max(0, best_count)}; "
        f"bestDemPath={best_path}; requiredDemNames={required_names}; "
        f"availableDemNames={list(inventory.get('availableDemNames') or ())}; "
        f"detectedTifNames={list(inventory.get('detectedTifNames') or ())}"
    )


def _candidate_dem_paths(config: CoverConfig) -> list[Path]:
    paths: list[Path] = []
    default_path = config.dem_full_path.resolve()
    if default_path.exists():
        paths.append(default_path)
    resource_dir = default_path.parent
    for path in regional_dem_paths(resource_dir):
        resolved = path.resolve()
        if resolved not in paths:
            paths.append(resolved)
    return paths


def _projected_dem_coverage_count(
    path: Path,
    coords: list[dict[str, Any]],
    *,
    fallback_epsg: int,
    errors: list[dict[str, str]] | None = None,
) -> int | None:
    try:
        with rasterio.open(path) as ds:
            crs = ds.crs if ds.crs is not None else CRS.from_epsg(int(fallback_epsg))
            if not bool(getattr(crs, "is_projected", False)):
                return None
            transformer = Transformer.from_crs(CRS.from_epsg(4326), crs, always_xy=True)
            bounds = ds.bounds
    except Exception as exc:
        if errors is not None:
            errors.append(
                {
                    "demName": path.name,
                    "demPath": str(path),
                    "errorType": type(exc).__name__,
                    "error": str(exc)[:1000],
                }
            )
        return None
    count = 0
    for coord in coords:
        norm = _normalize_coord(coord)
        if norm is None:
            continue
        x, y = transformer.transform(float(norm["longitude"]), float(norm["latitude"]))
        if bounds.left <= x <= bounds.right and bounds.bottom <= y <= bounds.top:
            count += 1
    return int(count)


def _safe_int(value: Any) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _mission_detail(mission: Any) -> dict[str, Any]:
    if not isinstance(mission, dict):
        raise AntiArmorReviewError("mission is not a dict")
    detail = mission.get("missionDetail")
    if not isinstance(detail, dict):
        raise AntiArmorReviewError(f"mission {mission.get('inputMissionID')} has no missionDetail")
    return detail


def _normalize_coord(value: Any) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    try:
        lat = float(value.get("latitude"))
        lon = float(value.get("longitude"))
    except Exception:
        return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    try:
        altitude = float(value.get("altitude", 0) or 0)
    except Exception:
        altitude = 0.0
    return {
        "latitude": float(lat),
        "longitude": float(lon),
        "altitude": int(round(altitude)),
    }


def _normalize_coord_list(value: Any, *, min_len: int = 0) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        return []
    coords: list[dict[str, Any]] = []
    for item in value:
        coord = _normalize_coord(item)
        if coord is not None:
            coords.append(coord)
    if len(coords) < int(min_len):
        return []
    return coords


def _first_line_coords(mission: Any, *, min_len: int = 2) -> list[dict[str, Any]]:
    detail = _mission_detail(mission)
    line_list = detail.get("lineList")
    if isinstance(line_list, list):
        for row in line_list:
            if not isinstance(row, dict):
                continue
            coords = _normalize_coord_list(row.get("coordinateList"), min_len=min_len)
            if coords:
                return coords
    coords = _normalize_coord_list(detail.get("coordinateList"), min_len=min_len)
    if coords:
        return coords
    raise AntiArmorReviewError(f"mission {mission.get('inputMissionID')} has no valid line coordinates")


def _first_area_coords(mission: Any, *, min_len: int = 3) -> list[dict[str, Any]]:
    detail = _mission_detail(mission)
    area_list = detail.get("areaList")
    if isinstance(area_list, list):
        for row in area_list:
            if not isinstance(row, dict) or bool(row.get("isHole")):
                continue
            coords = _normalize_coord_list(row.get("coordinateList"), min_len=min_len)
            if coords:
                return coords
    coords = _normalize_coord_list(detail.get("coordinateList"), min_len=min_len)
    if coords:
        return coords
    raise AntiArmorReviewError(f"mission {mission.get('inputMissionID')} has no valid area coordinates")


def _altitude_hint(coords: list[dict[str, Any]]) -> int:
    for coord in coords:
        try:
            return int(round(float(coord.get("altitude", 0) or 0)))
        except Exception:
            continue
    return 0


def _line_width_hint(*missions: Any) -> int:
    widths: list[float] = []
    for mission in missions:
        try:
            detail = _mission_detail(mission)
        except AntiArmorReviewError:
            continue
        for row in detail.get("lineList") or []:
            if not isinstance(row, dict):
                continue
            try:
                width = float(row.get("width", 0) or 0)
            except Exception:
                width = 0.0
            if width > 0:
                widths.append(width)
    width_value = max(widths) if widths else 1000.0
    return max(1, min(50000, int(round(width_value))))


def _coord_to_native(dem: DemGrid, coord: dict[str, Any]) -> tuple[float, float]:
    return dem.latlon_to_native(float(coord["latitude"]), float(coord["longitude"]))


def _native_to_coord(dem: DemGrid, point_xy: tuple[float, float], *, altitude: int) -> dict[str, Any]:
    lat, lon = dem.native_to_latlon(float(point_xy[0]), float(point_xy[1]))
    return {
        "latitude": round(float(lat), 8),
        "longitude": round(float(lon), 8),
        "altitude": int(altitude),
    }


def _native_polygon_to_coords(
    dem: DemGrid,
    polygon_xy: list[tuple[float, float]],
    *,
    altitude: int,
) -> list[dict[str, Any]]:
    return [_native_to_coord(dem, point, altitude=altitude) for point in polygon_xy]


def _native_line_to_coords(
    dem: DemGrid,
    line_xy: list[tuple[float, float]],
    *,
    altitude: int,
) -> list[dict[str, Any]]:
    coords = [_native_to_coord(dem, point, altitude=altitude) for point in line_xy]
    return _dedupe_coord_path(coords)


def _dedupe_coord_path(coords: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    last_key: tuple[int, int, int] | None = None
    for coord in coords:
        norm = _normalize_coord(coord)
        if norm is None:
            continue
        key = (
            int(round(float(norm["latitude"]) * 1_000_000)),
            int(round(float(norm["longitude"]) * 1_000_000)),
            int(norm.get("altitude", 0) or 0),
        )
        if key == last_key:
            continue
        out.append(norm)
        last_key = key
    return out


def _polygon_centroid(points: list[tuple[float, float]]) -> tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    area2 = 0.0
    cx = 0.0
    cy = 0.0
    n = len(points)
    for i, p0 in enumerate(points):
        p1 = points[(i + 1) % n]
        cross = p0[0] * p1[1] - p1[0] * p0[1]
        area2 += cross
        cx += (p0[0] + p1[0]) * cross
        cy += (p0[1] + p1[1]) * cross
    if abs(area2) < 1e-9:
        return (
            float(sum(p[0] for p in points) / len(points)),
            float(sum(p[1] for p in points) / len(points)),
        )
    return (float(cx / (3.0 * area2)), float(cy / (3.0 * area2)))


def _closest_point_on_segment(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> tuple[float, float]:
    px, py = point
    x0, y0 = start
    x1, y1 = end
    dx = x1 - x0
    dy = y1 - y0
    denom = dx * dx + dy * dy
    if denom <= 1e-9:
        return (float(x0), float(y0))
    t = ((px - x0) * dx + (py - y0) * dy) / denom
    t = max(0.0, min(1.0, float(t)))
    return (float(x0 + dx * t), float(y0 + dy * t))


def _closest_point_on_polygon_boundary(
    polygon: list[tuple[float, float]],
    target: tuple[float, float],
) -> tuple[float, float]:
    if not polygon:
        return target
    best = polygon[0]
    best_d = float("inf")
    for idx, start in enumerate(polygon):
        end = polygon[(idx + 1) % len(polygon)]
        candidate = _closest_point_on_segment(target, start, end)
        dist = math.hypot(candidate[0] - target[0], candidate[1] - target[1])
        if dist < best_d:
            best = candidate
            best_d = dist
    return (float(best[0]), float(best[1]))


def _cross_2d(left: tuple[float, float], right: tuple[float, float]) -> float:
    return float(left[0] * right[1] - left[1] * right[0])


def _segment_edge_intersection_parameters(
    start: tuple[float, float],
    end: tuple[float, float],
    edge_start: tuple[float, float],
    edge_end: tuple[float, float],
    *,
    eps: float = 1e-9,
) -> list[float]:
    """Return intersection positions ``t`` on ``start + t * (end-start)``."""

    ray = (float(end[0] - start[0]), float(end[1] - start[1]))
    edge = (
        float(edge_end[0] - edge_start[0]),
        float(edge_end[1] - edge_start[1]),
    )
    ray_len_sq = ray[0] * ray[0] + ray[1] * ray[1]
    if ray_len_sq <= eps:
        return []

    offset = (
        float(edge_start[0] - start[0]),
        float(edge_start[1] - start[1]),
    )
    denominator = _cross_2d(ray, edge)
    if abs(denominator) > eps:
        t = _cross_2d(offset, edge) / denominator
        u = _cross_2d(offset, ray) / denominator
        if -eps <= t <= 1.0 + eps and -eps <= u <= 1.0 + eps:
            return [max(0.0, min(1.0, float(t)))]
        return []

    if abs(_cross_2d(offset, ray)) > eps:
        return []

    # Collinear polygon edge: retain both projected endpoints; the caller
    # selects the first/last boundary crossing appropriate for its direction.
    values: list[float] = []
    for point in (edge_start, edge_end):
        delta = (float(point[0] - start[0]), float(point[1] - start[1]))
        t = (delta[0] * ray[0] + delta[1] * ray[1]) / ray_len_sq
        if -eps <= t <= 1.0 + eps:
            values.append(max(0.0, min(1.0, float(t))))
    return values


def _polygon_boundary_parameters(
    polygon: list[tuple[float, float]],
    start: tuple[float, float],
    end: tuple[float, float],
) -> list[float]:
    values: list[float] = []
    for index, edge_start in enumerate(polygon):
        edge_end = polygon[(index + 1) % len(polygon)]
        values.extend(
            _segment_edge_intersection_parameters(start, end, edge_start, edge_end)
        )
    values.sort()
    deduped: list[float] = []
    for value in values:
        if not deduped or abs(value - deduped[-1]) > 1e-8:
            deduped.append(float(value))
    return deduped


def _polygon_boundary_point_on_centerline(
    polygon: list[tuple[float, float]],
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    leaving_start_polygon: bool,
) -> tuple[float, float]:
    parameters = _polygon_boundary_parameters(polygon, start, end)
    if parameters:
        # For the source polygon use its last exit; for the destination use its
        # first entry. This keeps the emitted portion outside both polygons even
        # for a mildly concave boundary crossed more than once.
        t = max(parameters) if leaving_start_polygon else min(parameters)
        return (
            float(start[0] + (end[0] - start[0]) * t),
            float(start[1] + (end[1] - start[1]) * t),
        )
    target = end if leaving_start_polygon else start
    return _closest_point_on_polygon_boundary(polygon, target)


def _boundary_connection(
    start_polygon: list[tuple[float, float]],
    end_polygon: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Connect polygon centroids, clipped to the two polygon boundaries."""

    start_centroid = _polygon_centroid(start_polygon)
    end_centroid = _polygon_centroid(end_polygon)
    return [
        _polygon_boundary_point_on_centerline(
            start_polygon,
            start_centroid,
            end_centroid,
            leaving_start_polygon=True,
        ),
        _polygon_boundary_point_on_centerline(
            end_polygon,
            start_centroid,
            end_centroid,
            leaving_start_polygon=False,
        ),
    ]


def _boundary_to_point_connection(
    start_polygon: list[tuple[float, float]],
    end_point: tuple[float, float],
) -> list[tuple[float, float]]:
    """Connect a polygon centroid to a point, omitting the in-polygon portion."""

    start_centroid = _polygon_centroid(start_polygon)
    return [
        _polygon_boundary_point_on_centerline(
            start_polygon,
            start_centroid,
            end_point,
            leaving_start_polygon=True,
        ),
        (float(end_point[0]), float(end_point[1])),
    ]


def _clone_existing_mission(
    source: dict[str, Any],
    *,
    input_id: int,
    mission_type: int,
    region_type: int,
) -> dict[str, Any]:
    mission = copy.deepcopy(source)
    mission["inputMissionID"] = int(input_id)
    mission["inputMissionType"] = int(mission_type)
    mission["regionType"] = int(region_type)
    mission["isDone"] = False
    if not isinstance(mission.get("missionDetail"), dict):
        # Keep [] here (not null): a mission with every shape slot null makes the
        # external 53110 serializer throw (shapeType=0) and kills the whole message.
        mission["missionDetail"] = {"coordinateList": [], "lineList": [], "areaList": []}
    return mission


def _null_out_empty_shape_slots(mission: dict[str, Any]) -> None:
    """Replace empty-list shape slots with null when another slot has data.

    The external 0204->53110 converter picks the shape by null-checks in the
    order areaList -> lineList -> coordinateList, so an empty (non-null) list
    shadows the populated slot behind it and the real geometry is dropped.
    """
    detail = mission.get("missionDetail")
    if not isinstance(detail, dict):
        return
    slots = ("coordinateList", "lineList", "areaList")
    if not any(detail.get(k) for k in slots):
        return
    for k in slots:
        value = detail.get(k)
        if isinstance(value, list) and not value:
            detail[k] = None


def _build_line_mission(
    template: dict[str, Any],
    *,
    input_id: int,
    mission_type: int,
    region_type: int,
    coords: list[dict[str, Any]],
    width: int,
) -> dict[str, Any]:
    if len(coords) < 2:
        raise AntiArmorReviewError(f"line mission {input_id} has fewer than 2 coordinates")
    mission = _clone_existing_mission(
        template,
        input_id=input_id,
        mission_type=mission_type,
        region_type=region_type,
    )
    # Unused shape slots must be null, not [] — the external 0204->53110
    # converter dispatches on "areaList != null" first, so an empty areaList
    # would misclassify this as a polygon mission and drop the polyline.
    mission["missionDetail"] = {
        "coordinateList": None,
        "lineList": [
            {
                "width": max(1, min(50000, int(width))),
                "coordinateList": copy.deepcopy(coords),
            }
        ],
        "areaList": None,
    }
    return mission


def _build_area_mission(
    template: dict[str, Any],
    *,
    input_id: int,
    mission_type: int,
    region_type: int,
    coords: list[dict[str, Any]],
) -> dict[str, Any]:
    if len(coords) < 3:
        raise AntiArmorReviewError(f"area mission {input_id} has fewer than 3 coordinates")
    mission = _clone_existing_mission(
        template,
        input_id=input_id,
        mission_type=mission_type,
        region_type=region_type,
    )
    # Unused shape slots must be null, not [] — see _build_line_mission.
    mission["missionDetail"] = {
        "coordinateList": None,
        "lineList": None,
        "areaList": [
            {
                "isHole": False,
                "coordinateList": copy.deepcopy(coords),
            }
        ],
    }
    return mission


def _set_package_id_and_timestamp(
    payload: dict[str, Any],
    *,
    new_package_id: int,
    timestamp_ms: int,
) -> None:
    for key in ("inputMissionPackageID", "InputMissionPackageID", "inputMissionPackageId"):
        if key in payload or key == "inputMissionPackageID":
            payload[key] = int(new_package_id)
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in payload or key == "timestamp":
            payload[key] = int(timestamp_ms)
