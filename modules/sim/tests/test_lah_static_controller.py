from __future__ import annotations

import pytest

from modules.common.regional_dem import REGIONAL_DEM_SPECS
from modules.mission_planning.MissionPlanner.data_def.mission_helpers import terrain_elev
from modules.sim.runtime.controllers.lah_static import LAHStaticWaypointController
from modules.sim.runtime.controllers.waypoint_pid import (
    WaypointPIDController,
    WaypointTarget,
)
from modules.sim.runtime.geo import GeoConverter
from modules.sim.runtime.lah import DEFAULT_ENVELOPE, LAH, LAHParams
from modules.sim.runtime.sim_service import (
    PathDefinition,
    SimVehicle,
    SimulationService,
    _lah_plan_deviation_m,
)
from modules.sim.runtime.targets import GroundTarget
from modules.sim.runtime.uav import UAV, UAVParams


def _lah_controller(
    *targets: WaypointTarget,
) -> tuple[LAH, LAHStaticWaypointController]:
    lah = LAH(LAHParams())
    lah.s.x = 0.0
    lah.s.y = 0.0
    lah.s.z = 100.0
    controller = LAHStaticWaypointController(
        lah,
        list(targets),
        speed_target=20.0,
        pos_tol=0.1,
        allow_hover=True,
    )
    return lah, controller


def _ground_target(target_id: int = 11) -> GroundTarget:
    return GroundTarget(
        id=target_id,
        type_id=2,
        name=f"TARGET{target_id}",
        x=100.0,
        y=0.0,
        z=100.0,
        moving=False,
        vmin=0.0,
        vmax=0.0,
        roam_center=None,
        roam_radius=None,
        threat=None,
    )


def _lah_attack_service(
    *targets: WaypointTarget,
) -> tuple[SimulationService, LAHStaticWaypointController]:
    lah, controller = _lah_controller(*targets)
    service = SimulationService()
    service.vehicles = {
        "LAH1": SimVehicle(
            label="LAH1",
            aircraft_id=1,
            airframe="lah",
            vehicle=lah,
            controller=controller,
            path_id=None,
        )
    }
    return service, controller


def _mission_payload(
    *,
    mission_plan_id: int,
    aircraft_id: int,
    path_id: int,
    altitude_m: float,
    path_has_aircraft_id: bool = True,
    attack_target_id: int = 0,
) -> dict:
    waypoint_key = "lahWaypointList" if aircraft_id <= 3 else "uavWaypointList"
    path = {
        "pathID": int(path_id),
        waypoint_key: [
            {
                "waypointID": int(path_id) * 10 + 1,
                "nextWaypointID": int(path_id) * 10 + 2,
                "isDone": False,
                "coordinate": {
                    "latitude": 37.947035,
                    "longitude": 127.309533,
                    "altitude": float(altitude_m),
                },
                "speed": 40.0,
                "attack": {
                    "targetID": int(attack_target_id),
                    "weaponType": 3 if attack_target_id > 0 else 0,
                },
            },
            {
                "waypointID": int(path_id) * 10 + 2,
                "nextWaypointID": 0,
                "isDone": False,
                "coordinate": {
                    "latitude": 37.947035,
                    "longitude": 127.309533,
                    "altitude": float(altitude_m),
                },
                "speed": 40.0,
            },
        ],
    }
    if path_has_aircraft_id:
        path["aircraftID"] = int(aircraft_id)
    return {
        "missionPlanID": int(mission_plan_id),
        "individualMissionPlans": [
            {
                "aircraftID": int(aircraft_id),
                "individualMissionList": [
                    {
                        "individualMissionID": int(path_id) + 9000,
                        "pathID": int(path_id),
                        "isDone": False,
                    }
                ],
            }
        ],
        "flightPaths": [path],
    }


def test_lah_uses_waypoint_speed_and_fixed_vertical_rates_without_pid_commands() -> (
    None
):
    lah, controller = _lah_controller(
        WaypointTarget(pos=(100.0, 0.0, 130.0), speed=20.0),
    )

    assert controller.update(0.5) is True

    assert lah.s.x == pytest.approx(10.0)
    assert lah.s.y == pytest.approx(0.0)
    assert lah.s.z == pytest.approx(113.35)
    assert lah.s.u == pytest.approx(20.0)
    assert lah.cmd_yaw_rate == 0.0
    assert lah.cmd_pitch_rate == 0.0
    assert lah.cmd_roll_rate == 0.0
    assert lah.cmd_throttle == 0.0


def test_lah_descent_is_deterministic_and_does_not_change_horizontal_course() -> None:
    lah, controller = _lah_controller(
        WaypointTarget(pos=(60.0, 80.0, 70.0), speed=10.0),
    )

    controller.update(0.5)

    assert lah.s.x == pytest.approx(3.0)
    assert lah.s.y == pytest.approx(4.0)
    assert lah.s.z == pytest.approx(89.5)
    assert lah.s.u == pytest.approx(10.0)


def test_lah_hover_mission_waits_then_continues_to_next_waypoint() -> None:
    lah, controller = _lah_controller(
        WaypointTarget(pos=(0.0, 0.0, 100.0), speed=10.0, hover_time=0.5),
        WaypointTarget(pos=(20.0, 0.0, 100.0), speed=10.0),
    )

    controller.update(0.1)
    assert controller.is_hovering is True
    assert controller.curr_idx == 0

    controller.update(0.5)
    assert controller.is_hovering is False
    assert controller.curr_idx == 1
    assert lah.s.x == pytest.approx(0.0)

    controller.update(0.5)
    assert lah.s.x == pytest.approx(5.0)


def test_sim_service_does_not_apply_lah_dynamics_a_second_time() -> None:
    lah, controller = _lah_controller(
        WaypointTarget(pos=(100.0, 0.0, 100.0), speed=20.0),
    )
    service = SimulationService()
    service.vehicles = {
        "LAH1": SimVehicle(
            label="LAH1",
            aircraft_id=1,
            airframe="lah",
            vehicle=lah,
            controller=controller,
            path_id=None,
        )
    }

    service._step_once(0.5)

    assert lah.s.x == pytest.approx(10.0)


def test_attack_waypoint_waits_and_retries_when_target_lookup_is_transient(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    attack_wp = WaypointTarget(
        pos=(0.0, 0.0, 100.0),
        speed=10.0,
        wp_id=19475,
        attack={"targetID": 11, "weaponType": 3},
    )
    service, controller = _lah_attack_service(attack_wp)
    target = _ground_target()
    service.targets = [target]
    monkeypatch.setattr(service, "_attack_target_is_confirmed_destroyed", lambda _id: False)
    monkeypatch.setattr(service, "_resolve_attack_target", lambda _id: None)

    service._step_once(0.1)

    assert controller.current_target() is attack_wp
    assert controller.curr_idx == 0
    assert service._attack_holds["LAH1"]["wp"] is attack_wp
    assert service._get_weapon_counts(service.vehicles["LAH1"])["type3"] == 100

    def _raise_lookup_error(_target_id: int) -> GroundTarget:
        raise RuntimeError("target cache refresh in progress")

    monkeypatch.setattr(service, "_resolve_attack_target", _raise_lookup_error)
    service._step_once(0.1)

    assert controller.current_target() is attack_wp
    assert service._get_weapon_counts(service.vehicles["LAH1"])["type3"] == 100

    monkeypatch.setattr(service, "_resolve_attack_target", lambda _id: target)
    service._step_once(0.1)

    assert controller.current_target() is attack_wp
    assert service._get_weapon_counts(service.vehicles["LAH1"])["type3"] == 99
    assert any(projectile.target_id == 11 for projectile in service._projectiles)


def test_destroyed_attack_target_releases_hold_and_advances_waypoint(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    attack_wp = WaypointTarget(
        pos=(0.0, 0.0, 100.0),
        wp_id=19475,
        attack={"targetID": 11, "weaponType": 3},
    )
    egress_wp = WaypointTarget(pos=(100.0, 0.0, 100.0), wp_id=19476)
    service, controller = _lah_attack_service(attack_wp, egress_wp)
    service.targets = [_ground_target()]
    monkeypatch.setattr(service, "_attack_target_is_confirmed_destroyed", lambda _id: True)

    service._step_once(0.1)

    assert controller.current_target() is egress_wp
    assert controller.curr_idx == 1
    assert "LAH1" not in service._attack_holds
    assert attack_wp.hover_time is None


def test_attack_target_mapping_miss_does_not_raise_name_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    service = SimulationService()
    service._target_id_map_0402 = {"bad": "invalid", 99: 10}
    monkeypatch.setattr(service, "_resolve_actual_target_from_info", lambda _id: None)
    monkeypatch.setattr(service, "_virtual_target_from_info", lambda _id: None)

    assert service._resolve_attack_target(11) is None


def test_uav_keeps_the_existing_pid_controller_path() -> None:
    uav = UAV(UAVParams())
    controller = WaypointPIDController(
        uav,
        [WaypointTarget(pos=(100.0, 0.0, 100.0), speed=20.0)],
    )

    assert isinstance(controller, LAHStaticWaypointController) is False
    assert getattr(controller, "updates_vehicle_state_directly", False) is False


def test_sim_build_selects_static_controller_only_for_lah() -> None:
    service = SimulationService()
    service.geo = GeoConverter(127.0, 38.0)
    paths = [
        PathDefinition(
            label="LAH1",
            aircraft_id=1,
            airframe="lah",
            path_id=1001,
            waypoints=[
                {"lat": 38.001, "lon": 127.001, "alt": 100.0, "speed": 20.0},
                {"lat": 38.002, "lon": 127.002, "alt": 120.0, "speed": 20.0},
            ],
        ),
        PathDefinition(
            label="UAV1",
            aircraft_id=4,
            airframe="uav",
            path_id=4001,
            waypoints=[
                {"lat": 38.001, "lon": 127.001, "alt": 300.0, "speed": 50.0},
                {"lat": 38.002, "lon": 127.002, "alt": 300.0, "speed": 50.0},
            ],
        ),
    ]

    service._build_vehicles(paths)

    assert isinstance(service.vehicles["LAH1"].controller, LAHStaticWaypointController)
    assert type(service.vehicles["UAV1"].controller) is WaypointPIDController


@pytest.mark.parametrize(
    ("hover_time", "loiter", "attack"),
    [
        (None, None, None),
        (300.0, None, None),
        (None, {"time": 30.0, "radius": 100.0}, None),
        (None, None, {"targetID": 7, "weaponType": 3}),
    ],
)
def test_lah_valid_mission_altitude_is_preserved_for_every_waypoint_mode(
    monkeypatch: pytest.MonkeyPatch,
    hover_time: float | None,
    loiter: dict | None,
    attack: dict | None,
) -> None:
    service = SimulationService()

    def _unexpected_dem_lookup(_lat: float, _lon: float) -> float:
        raise AssertionError("valid MSL altitude must not be replaced from DEM")

    monkeypatch.setattr(service, "_terrain_elev", _unexpected_dem_lookup)

    altitude = service._adjust_lah_altitude(
        1,
        37.947035,
        127.309533,
        246.125,
        hover_time=hover_time,
        loiter=loiter,
        attack=attack,
    )

    assert altitude == pytest.approx(246.125)


@pytest.mark.parametrize("invalid_altitude", [None, 0.0, -1.0, float("nan"), float("inf")])
def test_lah_invalid_mission_altitude_uses_safe_terrain_fallback(
    monkeypatch: pytest.MonkeyPatch,
    invalid_altitude: float | None,
) -> None:
    service = SimulationService()
    monkeypatch.setattr(service, "_terrain_elev", lambda _lat, _lon: 147.3)

    altitude = service._adjust_lah_altitude(
        1,
        37.947035,
        127.309533,
        invalid_altitude,
        hover_time=300.0,
        loiter={"time": 30.0},
        attack={"targetID": 7},
    )

    assert altitude == pytest.approx(347.3)


def test_lah_loader_accepts_attack_path_above_ordinary_ceiling(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    service = SimulationService()
    monkeypatch.setattr(service, "_terrain_elev", lambda _lat, _lon: 147.3)

    accepted = service.load_mission(
        _mission_payload(
            mission_plan_id=7001,
            aircraft_id=1,
            path_id=1001,
            altitude_m=DEFAULT_ENVELOPE.max_altitude_m,
        )
    )
    assert accepted["ok"] is True
    original_vehicle = service.vehicles["LAH1"]
    original_targets = list(original_vehicle.controller.targets)

    attack_plan = service.load_mission(
        _mission_payload(
            mission_plan_id=7002,
            aircraft_id=1,
            path_id=1002,
            altitude_m=DEFAULT_ENVELOPE.max_altitude_m + 1.0,
            path_has_aircraft_id=False,
            attack_target_id=7,
        )
    )

    assert attack_plan["ok"] is True
    assert service.current_mission_plan_id() == 7002
    # A regular mission load rebuilds the vehicle; acceptance and retained
    # high-altitude attack waypoints are the behavior under test here.
    assert service.vehicles["LAH1"] is not original_vehicle
    assert service.vehicles["LAH1"].controller.targets != original_targets
    assert [
        target.pos[2] for target in service.vehicles["LAH1"].controller.targets
    ] == pytest.approx([DEFAULT_ENVELOPE.max_altitude_m + 1.0] * 2)


def test_lah_loader_accepts_nonattack_path_above_former_ceiling() -> None:
    service = SimulationService()

    accepted = service.load_mission(
        _mission_payload(
            mission_plan_id=7004,
            aircraft_id=1,
            path_id=1004,
            altitude_m=DEFAULT_ENVELOPE.max_altitude_m + 1.0,
        )
    )

    assert accepted["ok"] is True
    assert service.current_mission_plan_id() == 7004
    assert [
        target.pos[2] for target in service.vehicles["LAH1"].controller.targets
    ] == pytest.approx([DEFAULT_ENVELOPE.max_altitude_m + 1.0] * 2)


def test_lah_altitude_ceiling_does_not_restrict_uav_mission_altitude(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    service = SimulationService()
    monkeypatch.setattr(service, "_terrain_elev", lambda _lat, _lon: 147.3)

    result = service.load_mission(
        _mission_payload(
            mission_plan_id=7003,
            aircraft_id=4,
            path_id=4001,
            altitude_m=DEFAULT_ENVELOPE.max_altitude_m + 1.0,
        )
    )

    assert result["ok"] is True
    assert service.vehicles["UAV1"].vehicle.s.z == pytest.approx(
        DEFAULT_ENVELOPE.max_altitude_m + 1.0
    )
    assert [
        target.pos[2] for target in service.vehicles["UAV1"].controller.targets
    ] == pytest.approx(
        [DEFAULT_ENVELOPE.max_altitude_m + 1.0] * 2
    )


def test_lah_rtb_accepts_altitude_above_former_operational_ceiling() -> None:
    service, _controller = _lah_attack_service(
        WaypointTarget(pos=(0.0, 0.0, 500.0), wp_id=1)
    )
    service.geo = GeoConverter(128.2, 37.8)
    simv = service.vehicles["LAH1"]

    assert service._make_rtb_controller(
        simv,
        rtb=(37.8, 128.2, DEFAULT_ENVELOPE.max_altitude_m),
    ) is not None
    high_controller = service._make_rtb_controller(
        simv,
        rtb=(37.8, 128.2, DEFAULT_ENVELOPE.max_altitude_m + 0.001),
    )
    assert high_controller is not None
    assert high_controller.targets[0].pos[2] == pytest.approx(
        DEFAULT_ENVELOPE.max_altitude_m + 0.001
    )


def test_lah_altitude_resolution_does_not_change_uav_values(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    service = SimulationService()

    def _unexpected_dem_lookup(_lat: float, _lon: float) -> float:
        raise AssertionError("LAH-only altitude resolver must not touch UAV")

    monkeypatch.setattr(service, "_terrain_elev", _unexpected_dem_lookup)

    assert service._adjust_lah_altitude(
        4,
        38.0,
        127.0,
        1_200.0,
        hover_time=None,
        loiter=None,
        attack=None,
    ) == pytest.approx(1_200.0)
    assert service._adjust_lah_altitude(
        4,
        38.0,
        127.0,
        None,
        hover_time=None,
        loiter=None,
        attack=None,
    ) is None


def test_lah_loader_keeps_raw_msl_for_transit_and_hover_waypoints(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    service = SimulationService()
    monkeypatch.setattr(service, "_terrain_elev", lambda _lat, _lon: 147.3)

    result = service.load_mission(
        {
            "missionPlanID": 7001,
            "inputMissionPlans": [
                {
                    "timestamp": 1,
                    "inputMissionList": [
                        {
                            "inputMissionID": 101,
                            "inputMissionType": 1,
                            "regionType": 4,
                        }
                    ],
                }
            ],
            "individualMissionPlans": [
                {
                    "aircraftID": 1,
                    "individualMissionList": [
                        {
                            "individualMissionID": 9001,
                            "pathID": 1001,
                            "isDone": False,
                            "relatedMission": {"inputMissionID": 101},
                        }
                    ],
                }
            ],
            "flightPaths": [
                {
                    "pathID": 1001,
                    "aircraftID": 1,
                    "lahWaypointList": [
                        {
                            "waypointID": 5001,
                            "nextWaypointID": 5002,
                            "isDone": False,
                            "coordinate": {
                                "latitude": 37.947035,
                                "longitude": 127.309533,
                                "altitude": 246.0,
                            },
                            "speed": 40.0,
                            "attack": {"targetID": 0, "weaponType": 0},
                        },
                        {
                            "waypointID": 5002,
                            "nextWaypointID": 0,
                            "isDone": False,
                            "coordinate": {
                                "latitude": 37.947035,
                                "longitude": 127.309533,
                                "altitude": 246.0,
                            },
                            "speed": 40.0,
                            "hovering": {"time": 300.0},
                            "attack": {"targetID": 0, "weaponType": 0},
                        },
                    ],
                }
            ],
        }
    )

    assert result["ok"] is True
    simv = service.vehicles["LAH1"]
    assert simv.vehicle.s.z == pytest.approx(246.0)
    assert [target.pos[2] for target in simv.controller.targets] == pytest.approx(
        [246.0, 246.0]
    )


def test_lah_plan_deviation_uses_active_generated_plan_leg() -> None:
    lah, controller = _lah_controller(
        WaypointTarget(pos=(0.0, 0.0, 100.0), wp_id=1),
        WaypointTarget(pos=(100.0, 0.0, 100.0), wp_id=2),
    )
    controller.curr_idx = 1
    lah.s.x = 40.0
    lah.s.y = 12.0
    simv = SimVehicle(
        label="LAH1",
        aircraft_id=1,
        airframe="lah",
        vehicle=lah,
        controller=controller,
        path_id=10,
    )

    assert _lah_plan_deviation_m(simv) == pytest.approx(12.0)


def test_lah_frame_always_reports_plan_deviation_and_dem_clearance(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    lah, controller = _lah_controller(
        WaypointTarget(pos=(0.0, 0.0, 500.0), wp_id=1),
        WaypointTarget(pos=(100.0, 0.0, 500.0), wp_id=2),
    )
    controller.curr_idx = 1
    lah.s.x = 40.0
    lah.s.y = 8.0
    lah.s.z = 500.0
    simv = SimVehicle(
        label="LAH1",
        aircraft_id=1,
        airframe="lah",
        vehicle=lah,
        controller=controller,
        path_id=10,
    )
    service = SimulationService(pos_tol=30.0)
    service.geo = GeoConverter(128.2, 37.8)
    monkeypatch.setattr(service, "_terrain_source_name", lambda _lat, _lon: "Inje_10m.tif")
    monkeypatch.setattr(service, "_terrain_elev", lambda _lat, _lon: 420.0)

    frame = service._build_frame(
        geo=service.geo,
        vehicles=[simv],
        targets=[],
        sim_time=0.0,
        step_count=1,
    )
    entry = frame["vehicles"]["LAH1"]

    assert entry["planDeviationM"] == pytest.approx(8.0)
    assert entry["planFollowing"] is True
    assert entry["demSource"] == "Inje_10m.tif"
    assert entry["demGroundM"] == pytest.approx(420.0)
    assert entry["terrainClearanceM"] == pytest.approx(80.0)
    assert entry["demAvailable"] is True


def test_lah_frame_clearance_uses_the_same_real_operational_dem() -> None:
    spec = next(item for item in REGIONAL_DEM_SPECS if item.filename == "Inje_10m.tif")
    latitude = (spec.south + spec.north) * 0.5
    longitude = (spec.west + spec.east) * 0.5
    ground_m = terrain_elev(latitude, longitude)

    lah, controller = _lah_controller(
        WaypointTarget(pos=(0.0, 0.0, ground_m + 50.0), wp_id=1),
        WaypointTarget(pos=(100.0, 0.0, ground_m + 50.0), wp_id=2),
    )
    lah.s.z = ground_m + 50.0
    service = SimulationService()
    service.geo = GeoConverter(longitude, latitude)
    simv = SimVehicle(
        label="LAH1",
        aircraft_id=1,
        airframe="lah",
        vehicle=lah,
        controller=controller,
        path_id=10,
    )

    frame = service._build_frame(
        geo=service.geo,
        vehicles=[simv],
        targets=[],
        sim_time=0.0,
        step_count=1,
    )
    entry = frame["vehicles"]["LAH1"]

    assert entry["demSource"] == "Inje_10m.tif"
    assert entry["demGroundM"] == pytest.approx(ground_m)
    assert entry["terrainClearanceM"] == pytest.approx(50.0)
