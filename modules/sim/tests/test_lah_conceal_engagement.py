"""Concealment and engagement coupling in SIM.

Terrain masking is an individual property of each aircraft, and the terrain
that conceals an aircraft also blocks its own weapon.  These tests pin both
directions so a manned aircraft that hides is really unengageable, and one that
shoots is really exposed.
"""

from __future__ import annotations

from types import SimpleNamespace

from modules.sim.runtime.controllers.waypoint_pid import WaypointTarget
from modules.sim.runtime.geo import GeoConverter
from modules.sim.runtime.sim_service import SimulationService


def _vehicle(label, aircraft_id, airframe, *, x, y, z, controller=None):
    return SimpleNamespace(
        label=label,
        aircraft_id=aircraft_id,
        airframe=airframe,
        alive=True,
        vehicle=SimpleNamespace(
            s=SimpleNamespace(x=x, y=y, z=z, u=0.0, yaw=0.0, roll=0.0, pitch=0.0)
        ),
        controller=(
            controller
            if controller is not None
            else SimpleNamespace(current_target=lambda: None)
        ),
        shinil_profile_phase="baseline",
    )


def _mask_terrain(sim: SimulationService, hidden_labels: set[str]) -> None:
    """Stub terrain so the named aircraft are masked from every enemy."""

    def _los(sx, sy, sz, tx, ty, tz, **_kwargs):
        for label, veh in sim.vehicles.items():
            state = veh.vehicle.s
            if (
                abs(float(state.x) - float(sx)) < 1.0
                and abs(float(state.y) - float(sy)) < 1.0
                and abs(float(state.z) - float(sz)) < 1.0
            ):
                return label not in hidden_labels
        return True

    sim._check_los_terrain = _los
    sim._threat_los_cache = {}


def _record_shots(sim: SimulationService) -> list[tuple]:
    fired: list[tuple] = []

    def _spawn(**kwargs):
        fired.append(
            (
                kwargs.get("side"),
                kwargs.get("source_id"),
                kwargs.get("target_id"),
            )
        )

    sim._spawn_projectile = _spawn
    sim._apply_vehicle_hit = lambda simv: None
    return fired


def _run_threats(sim: SimulationService, *, seconds: float = 120.0, dt: float = 0.5) -> None:
    for _ in range(int(seconds / dt)):
        sim._evaluate_threats(dt)
        sim.sim_time += dt


def _scenario(hidden: set[str]) -> tuple[SimulationService, list[tuple]]:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    sim.enemy_hit_scale = 1.0
    # LAH1 is the closest aircraft to the enemy; UAV4 sits further out.
    sim.vehicles = {
        "LAH1": _vehicle("LAH1", 1, "lah", x=900.0, y=0.0, z=300.0),
        "UAV4": _vehicle("UAV4", 4, "uav", x=0.0, y=0.0, z=300.0),
    }
    sim.targets = [sim._build_target(type_id=1, x=1000.0, y=0.0, z=0.0, id_override=11)]
    _mask_terrain(sim, hidden)
    return sim, _record_shots(sim)


def test_masked_manned_aircraft_is_never_engaged() -> None:
    sim, fired = _scenario({"LAH1"})
    try:
        _run_threats(sim)
        engaged_labels = {row[2] for row in fired}
        assert "LAH1" not in engaged_labels
        assert sim._threat_exposure["LAH1"]["exposedTo"] == []
        assert sim._threat_exposure["LAH1"]["inRange"] == [11]
    finally:
        sim.shutdown()


def test_masked_aircraft_does_not_shield_an_exposed_aircraft_behind_it() -> None:
    """The nearest aircraft being hidden must not blank the enemy's picture."""

    sim, fired = _scenario({"LAH1"})
    try:
        _run_threats(sim)
        assert {row[2] for row in fired} == {"UAV4"}
    finally:
        sim.shutdown()


def test_exposure_history_is_kept_per_aircraft() -> None:
    sim, _fired = _scenario({"LAH1"})
    try:
        _run_threats(sim, seconds=30.0)
        enemy = sim.targets[0]
        masked_state = sim._threat_pair_state[sim._threat_pair_key(enemy, "LAH1")]
        exposed_state = sim._threat_pair_state[sim._threat_pair_key(enemy, "UAV4")]
        assert masked_state.t_exposed == 0.0
        assert masked_state.detected is False
        assert exposed_state.t_exposed > 0.0
    finally:
        sim.shutdown()


def test_exposed_aircraft_is_engaged_when_all_others_are_masked() -> None:
    sim, fired = _scenario({"UAV4"})
    try:
        _run_threats(sim)
        assert {row[2] for row in fired} == {"LAH1"}
    finally:
        sim.shutdown()


def test_concealment_is_reported_even_when_lethality_is_disabled() -> None:
    sim, fired = _scenario({"LAH1"})
    try:
        sim.enemy_hit_scale = 0.0  # deployment default: no friendly losses
        _run_threats(sim, seconds=30.0)
        assert fired == []
        frame = sim._build_frame(
            geo=sim.geo,
            vehicles=list(sim.vehicles.values()),
            targets=[],
            sim_time=sim.sim_time,
            step_count=1,
        )
        lah_entry = frame["vehicles"]["LAH1"]
        uav_entry = frame["vehicles"]["UAV4"]
        assert lah_entry["threatMasked"] is True
        assert lah_entry["threatInRangeTargetIDs"] == [11]
        assert uav_entry["threatMasked"] is False
        assert uav_entry["threatExposedTargetIDs"] == [11]
    finally:
        sim.shutdown()


def test_auto_attack_will_not_shoot_through_terrain() -> None:
    sim, fired = _scenario({"LAH1"})
    try:
        sim.lah_auto_attack = True
        for _ in range(60):
            sim._evaluate_vehicle_attacks(0.5)
            sim.sim_time += 0.5
        assert fired == []

        _mask_terrain(sim, set())
        sim.sim_time += 1.0
        for _ in range(10):
            sim._evaluate_vehicle_attacks(0.5)
            sim.sim_time += 0.5
        assert {row[1] for row in fired} == {"LAH1"}
    finally:
        sim.shutdown()


def _commanded_attack_scenario(hidden: set[str]):
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    sim.enemy_hit_scale = 1.0
    waypoint = WaypointTarget(
        pos=(900.0, 0.0, 300.0),
        wp_id=7,
        attack={"targetID": 11, "weaponType": 1},
    )
    controller = SimpleNamespace(
        current_target=lambda: waypoint,
        is_hovering=False,
        hover_timer=0.0,
        is_loitering=False,
        loiter_timer=0.0,
        _advance_wp=lambda: None,
    )
    sim.vehicles = {
        "LAH1": _vehicle(
            "LAH1", 1, "lah", x=900.0, y=0.0, z=300.0, controller=controller
        )
    }
    sim.targets = [sim._build_target(type_id=1, x=1000.0, y=0.0, z=0.0, id_override=11)]
    _mask_terrain(sim, hidden)
    return sim, _record_shots(sim), waypoint


def test_commanded_attack_holds_fire_while_masked_then_fires_when_unmasked() -> None:
    sim, fired, _waypoint = _commanded_attack_scenario({"LAH1"})
    try:
        for _ in range(20):
            sim._evaluate_vehicle_attacks(0.5)
            sim.sim_time += 0.5
        assert fired == []
        assert sim._attack_los_blocks["LAH1"]["targetID"] == 11

        frame = sim._build_frame(
            geo=sim.geo,
            vehicles=list(sim.vehicles.values()),
            targets=[],
            sim_time=sim.sim_time,
            step_count=1,
        )
        assert frame["vehicles"]["LAH1"]["attackLosBlocked"] is True
        assert frame["vehicles"]["LAH1"]["attackLosBlockedTargetID"] == 11

        # Unmasking the attack position releases the held shot.
        _mask_terrain(sim, set())
        sim.sim_time += 1.0
        for _ in range(10):
            sim._evaluate_vehicle_attacks(0.5)
            sim.sim_time += 0.5
        assert ("friendly", "LAH1", 11) in fired
        assert "LAH1" not in sim._attack_los_blocks
    finally:
        sim.shutdown()


def test_destroying_a_target_drops_its_per_aircraft_threat_state() -> None:
    sim, _fired = _scenario(set())
    try:
        _run_threats(sim, seconds=10.0)
        enemy = sim.targets[0]
        key = sim._threat_pair_key(enemy, "LAH1")
        assert key in sim._threat_pair_state
        sim._forget_threat_pairs_for_target(enemy)
        assert key not in sim._threat_pair_state
        assert key not in sim._threat_los_cache
    finally:
        sim.shutdown()


def test_virtual_and_raw_targets_do_not_share_a_sightline_cache() -> None:
    """Report IDs and raw IDs are separate namespaces holding equal integers."""

    sim, _fired = _scenario(set())
    try:
        raw = sim.targets[0]
        virtual = sim._build_target(
            type_id=1, x=raw.x, y=raw.y, z=raw.z, id_override=int(raw.id)
        )
        sim._virtual_targets[int(raw.id)] = virtual
        assert sim._threat_pair_key(raw, "LAH1") != sim._threat_pair_key(virtual, "LAH1")
    finally:
        sim.shutdown()
