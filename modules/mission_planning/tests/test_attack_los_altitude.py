from __future__ import annotations

import math

from modules.common.terrain_los import (
    EFFECTIVE_EARTH_RADIUS_M,
    ENEMY_OBSERVER_HEIGHT_M,
    LOS_CLEARANCE_M,
)
from modules.mission_planning.pipelines.attack_los_altitude import (
    profile_with_batch_dem,
    solve_minimum_attack_altitude,
)
from modules.mission_planning.replanning.triggers.attack.pipeline import (
    _apply_attack_los_profile_altitude,
    _copy_cached_attack_point,
)
from modules.mission_planning.replanning.triggers.attack import pipeline as attack_pipeline


def test_deployed_attack_los_policy_matches_sim_policy(monkeypatch) -> None:
    monkeypatch.setattr(
        attack_pipeline,
        "get_runtime_attack_float",
        lambda key, default=0.0: (
            99.0
            if key in {"attack_los_clearance_m", "attack_los_target_height_m"}
            else default
        ),
    )

    assert attack_pipeline._attack_los_clearance_m() == LOS_CLEARANCE_M
    assert attack_pipeline._attack_los_target_height_m() == ENEMY_OBSERVER_HEIGHT_M


def test_direct_attack_altitude_solver_uses_shared_los_defaults() -> None:
    profile = solve_minimum_attack_altitude([100.0, 100.0, 100.0], 1000.0)

    assert profile["verified"] is True
    assert profile["clearance_m"] == LOS_CLEARANCE_M
    assert profile["target_height_m"] == ENEMY_OBSERVER_HEIGHT_M


def test_ridge_raises_attack_altitude_until_entire_los_is_clear() -> None:
    terrain = [100.0] * 101
    terrain[50] = 600.0
    profile = solve_minimum_attack_altitude(
        terrain,
        1000.0,
        target_height_m=10.0,
        clearance_m=10.0,
    )

    assert profile["verified"] is True
    assert profile["controlling_distance_m"] == 500.0
    selected_altitude_m = math.ceil(float(profile["required_altitude_m"]))
    target_altitude_m = terrain[0] + 10.0
    for idx, ground_m in enumerate(terrain[1:-1], start=1):
        fraction = idx / (len(terrain) - 1)
        along_m = fraction * 1000.0
        effective_ground_m = ground_m - (
            along_m * along_m / (2.0 * EFFECTIVE_EARTH_RADIUS_M)
        )
        sightline_m = target_altitude_m + (selected_altitude_m - target_altitude_m) * fraction
        # Shared terrain_los blocks only when terrain > ray + clearance.
        assert effective_ground_m <= sightline_m + 10.0 + 1e-9
    assert selected_altitude_m > terrain[-1] + 10.0


def test_existing_terrain_plus_300_floor_is_kept_when_it_already_clears_los() -> None:
    profile = solve_minimum_attack_altitude(
        [100.0] * 11,
        1000.0,
        target_height_m=10.0,
        clearance_m=10.0,
    )
    result = {}
    _apply_attack_los_profile_altitude(
        result,
        base_altitude_m=100.0,
        altitude_offset_m=300.0,
        los_profile=profile,
    )

    assert result["altitude"] == 400
    assert result["los_verified"] is True
    assert result["los_altitude_adjusted"] is False


def test_cached_los_altitude_is_not_replaced_by_plain_terrain_offset() -> None:
    cached = {
        "latitude": 37.0,
        "longitude": 128.0,
        "altitude": 1111,
        "terrain_altitude_m": 100,
        "altitude_offset_m": 300.0,
        "los_verified": True,
        "los_required_altitude_m": 1110.2,
    }

    restored = _copy_cached_attack_point(
        cached,
        {"latitude": 37.0, "longitude": 128.0, "altitude": 1200},
    )

    assert restored["altitude"] == 1200
    assert restored["los_selected_altitude_m"] == 1111
    assert restored["lah_altitude_floor_m"] == 1200


def test_batch_dem_profile_samples_target_to_attack_once() -> None:
    enemy = {"latitude": 37.0, "longitude": 128.0}
    attack = {"latitude": 37.0, "longitude": 128.09}
    calls = []

    def lookup_many(pairs):
        rows = list(pairs)
        calls.append(rows)
        terrain = [100.0] * len(rows)
        terrain[len(rows) // 2] = 500.0
        return terrain

    profile = profile_with_batch_dem(
        attack,
        enemy,
        lookup_many,
        target_height_m=10.0,
        clearance_m=10.0,
        sample_step_m=10.0,
        max_samples=1024,
    )

    assert profile["verified"] is True
    assert len(calls) == 1
    assert profile["sample_count"] == len(calls[0])
    assert 7000.0 < profile["distance_m"] < 9000.0
    assert profile["required_altitude_m"] > 800.0


def test_distance_dependent_earth_curvature_is_included() -> None:
    terrain = [100.0] * 101
    terrain[50] = 200.0
    short = solve_minimum_attack_altitude(terrain, 1000.0)
    long = solve_minimum_attack_altitude(terrain, 9000.0)

    assert short["verified"] is True
    assert long["verified"] is True
    assert long["required_altitude_m"] < short["required_altitude_m"]
    assert long["earth_curvature_model"] == "shared_4_3_earth_source_tangent"


def test_attack_pipeline_batch_dem_fallback_applies_verified_altitude(monkeypatch) -> None:
    state = {"sample_count": 0}

    def terrain_many(pairs):
        rows = list(pairs)
        state["sample_count"] = len(rows)
        terrain = [100.0] * len(rows)
        terrain[max(1, len(rows) // 2)] = 550.0
        return terrain

    monkeypatch.setattr(attack_pipeline, "terrain_elev_many", terrain_many)
    monkeypatch.setattr(attack_pipeline, "reset_terrain_elev_many_metrics", lambda: None)
    monkeypatch.setattr(
        attack_pipeline,
        "get_terrain_elev_many_metrics",
        lambda **_kwargs: {"demResolvedByTile": state["sample_count"]},
    )

    result, error = attack_pipeline._compute_attack_los_altitude_batch_dem(
        {"latitude": 37.0, "longitude": 128.01, "altitude": 300},
        {"latitude": 37.0, "longitude": 128.0, "altitude": 0},
        lah_floor_coord={"latitude": 37.0, "longitude": 128.01, "altitude": 300},
    )

    assert error is None
    assert result is not None
    assert result["los_verified"] is True
    assert result["los_dem_resolved_sample_count"] == result["los_profile_sample_count"]
    assert result["altitude"] > 900
    assert attack_pipeline._preserve_attack_point_altitude(result) is True
