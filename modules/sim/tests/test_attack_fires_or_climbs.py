"""SIM owns the vertical popup performed at an armed low-level attack WP."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from modules.sim.runtime import sim_service
from modules.sim.runtime.controllers.waypoint_pid import WaypointTarget
from modules.sim.runtime.geo import GeoConverter
from modules.sim.runtime.lah import DEFAULT_ENVELOPE


def _scenario(*, base_altitude_m: float = 100.0, los_altitude_m: float = 124.0):
    sim = sim_service.SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    waypoint = WaypointTarget(
        pos=(900.0, 0.0, float(base_altitude_m)),
        wp_id=7,
        attack={"targetID": 11, "weaponType": 1},
    )
    current = [waypoint]
    advances: list[int] = []

    def _advance() -> None:
        advances.append(1)
        current[0] = None

    controller = SimpleNamespace(
        current_target=lambda: current[0],
        _advance_wp=_advance,
        is_hovering=False,
        hover_timer=0.0,
        is_loitering=False,
        loiter_timer=0.0,
        force_hover=False,
    )
    state = SimpleNamespace(
        x=900.0,
        y=0.0,
        z=float(base_altitude_m),
        u=0.0,
        yaw=0.0,
        roll=0.0,
        pitch=0.0,
    )
    vehicle = SimpleNamespace(
        label="LAH1",
        aircraft_id=1,
        airframe="lah",
        alive=True,
        vehicle=SimpleNamespace(s=state),
        controller=controller,
        shinil_profile_phase="baseline",
    )
    sim.vehicles = {"LAH1": vehicle}
    target = sim._build_target(
        type_id=1,
        x=1000.0,
        y=0.0,
        z=0.0,
        id_override=11,
    )
    sim.targets = [target]
    # Keep this unit scenario independent from a previously persisted
    # targetInfo record in the active Logs directory.
    sim._attack_target_is_confirmed_destroyed = lambda _target_id: False
    sim._threat_pair_terrain_los = (
        lambda _target, _vehicle, current_state: (
            float(current_state.z) >= float(los_altitude_m)
        )
    )
    shots: list[dict] = []
    effects: list[dict] = []
    sim._spawn_projectile = lambda **kwargs: shots.append(dict(kwargs))
    sim._spawn_effect = lambda **kwargs: effects.append(dict(kwargs))
    sim._handle_target_destroyed = lambda *_args, **_kwargs: None
    return sim, vehicle, target, waypoint, shots, effects, advances


def test_commanded_attack_climbs_fires_once_and_descends_to_base() -> None:
    sim, vehicle, target, _waypoint, shots, effects, advances = _scenario()
    try:
        elapsed_s = 0.0
        for _ in range(100):
            sim._evaluate_vehicle_attacks(0.5)
            sim.sim_time += 0.5
            elapsed_s += 0.5
            if advances:
                break

        assert shots[0]["start"][2] >= 124.0
        assert float(vehicle.vehicle.s.z) == pytest.approx(100.0)
        assert len(shots) == 1
        assert shots[0]["p_hit"] == 1.0
        assert shots[0]["force_hit"] is True
        assert target.alive is False
        assert len(effects) == 1
        assert advances == [1]
        assert "LAH1" not in sim._attack_holds
        assert elapsed_s <= 0.5
    finally:
        sim.shutdown()


def test_open_los_does_not_fire_before_reaching_the_attack_point() -> None:
    sim, vehicle, _target, _waypoint, shots, _effects, advances = _scenario(
        los_altitude_m=0.0
    )
    try:
        vehicle.vehicle.s.x = 700.0
        sim._evaluate_vehicle_attacks(0.5)

        assert shots == []
        assert advances == []
        assert float(vehicle.vehicle.s.z) == pytest.approx(100.0)
    finally:
        sim.shutdown()


def test_high_popup_returns_to_base_within_demo_pacing_budget() -> None:
    sim, vehicle, _target, waypoint, _shots, _effects, advances = _scenario(
        base_altitude_m=100.0
    )
    try:
        sim._ensure_attack_hold(vehicle, waypoint)
        vehicle.vehicle.s.z = 1441.0
        sim._begin_attack_descent(
            vehicle,
            waypoint,
            reason="shot_fired",
            shot_fired=True,
        )

        elapsed_s = 0.0
        for _ in range(20):
            sim._step_attack_descent(vehicle, waypoint, 0.5)
            elapsed_s += 0.5
            if advances:
                break

        assert float(vehicle.vehicle.s.z) == pytest.approx(100.0)
        assert advances == [1]
        assert elapsed_s <= 5.5
    finally:
        sim.shutdown()


def test_descent_finishes_even_after_the_attack_command_is_replaced() -> None:
    sim, vehicle, _target, waypoint, shots, _effects, advances = _scenario(
        base_altitude_m=100.0
    )
    try:
        sim._ensure_attack_hold(vehicle, waypoint)
        vehicle.vehicle.s.z = 1441.0
        sim._begin_attack_descent(
            vehicle,
            waypoint,
            reason="shot_fired",
            shot_fired=True,
        )
        vehicle.controller.current_target = lambda: None
        sim.lah_auto_attack = False

        for _ in range(20):
            sim._evaluate_vehicle_attacks(0.5)
            if "LAH1" not in sim._attack_holds:
                break

        assert shots == []
        assert float(vehicle.vehicle.s.z) == pytest.approx(100.0)
        # The old attack waypoint is no longer in the new controller, so
        # completing its descent must not advance the replacement route.
        assert advances == []
        assert "LAH1" not in sim._attack_holds
    finally:
        sim.shutdown()


def test_replan_preserves_only_descent_and_rebases_a_stale_popup_anchor() -> None:
    sim, vehicle, _target, waypoint, _shots, _effects, _advances = _scenario(
        base_altitude_m=100.0
    )
    try:
        sim._ensure_attack_hold(vehicle, waypoint)
        vehicle.vehicle.s.z = 1441.0
        sim._begin_attack_descent(
            vehicle,
            waypoint,
            reason="shot_fired",
            shot_fired=True,
        )
        attack_lon, attack_lat = sim.geo.xy_to_lonlat(900.0, 0.0)
        replacement_paths = [
            sim_service.PathDefinition(
                label="LAH1",
                aircraft_id=1,
                airframe="lah",
                path_id=99,
                waypoints=[
                    {
                        "lat": attack_lat,
                        "lon": attack_lon,
                        "alt": 1441.0,
                        "speed": 40.0,
                        "wp_id": 100,
                        "hover_time": 102.0,
                        "attack": {"targetID": 0, "weaponType": 0},
                    },
                    {
                        "lat": attack_lat + 0.0001,
                        "lon": attack_lon + 0.0001,
                        "alt": 100.0,
                        "speed": 40.0,
                        "wp_id": 101,
                        "attack": {"targetID": 0, "weaponType": 0},
                    },
                ],
            )
        ]
        sim._build_vehicles(replacement_paths, reset_detection_state=False)

        restored = sim._attack_holds.get("LAH1")
        assert restored is not None
        assert restored["phase"] == "descending_after_attack"
        assert restored["baseAltitudeM"] == pytest.approx(100.0)
        assert restored["replanAnchorRebased"] is True
        assert sim.vehicles["LAH1"].controller.current_target().pos[2] == pytest.approx(
            100.0
        )

        restored["phase"] = "climbing_for_los"
        sim._build_vehicles(replacement_paths, reset_detection_state=False)
        assert "LAH1" not in sim._attack_holds
    finally:
        sim.shutdown()


def test_commanded_attack_climbs_past_the_operational_envelope_until_los_opens() -> None:
    former_ceiling_m = float(DEFAULT_ENVELOPE.max_altitude_m)
    sim, vehicle, _target, _waypoint, shots, _effects, advances = _scenario(
        los_altitude_m=former_ceiling_m + 125.0
    )
    try:
        for _ in range(1000):
            sim._evaluate_vehicle_attacks(0.5)
            sim.sim_time += 0.5
            if advances:
                break

        assert len(shots) == 1
        assert float(shots[0]["start"][2]) > former_ceiling_m
        assert float(vehicle.vehicle.s.z) == pytest.approx(100.0)
        assert advances == [1]
        assert "LAH1" not in sim._attack_los_blocks
    finally:
        sim.shutdown()


def test_popup_rates_are_accelerated_without_changing_the_route_clearance() -> None:
    assert sim_service._ATTACK_LOS_CLIMB_RATE_MPS == pytest.approx(125.0)
    assert sim_service._ATTACK_LOS_DESCENT_RATE_MPS == pytest.approx(250.0)
    assert sim_service._LAH_MISSION_GROUND_CLEARANCE_M == pytest.approx(30.0)
