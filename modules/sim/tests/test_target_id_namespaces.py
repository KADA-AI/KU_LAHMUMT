from __future__ import annotations

from types import SimpleNamespace

from modules.sim.runtime.sim_service import SimulationService
from modules.sim.runtime.targets import GroundTarget


def _target(target_id: int, *, x: float) -> GroundTarget:
    return GroundTarget(
        id=target_id,
        type_id=2,
        name=f"TARGET{target_id}",
        x=x,
        y=0.0,
        z=100.0,
        moving=False,
        vmin=0.0,
        vmax=0.0,
        roam_center=None,
        roam_radius=None,
        threat=None,
    )


def _viewer() -> SimpleNamespace:
    state = SimpleNamespace(x=0.0, y=0.0, z=200.0)
    return SimpleNamespace(
        label="UAV1",
        aircraft_id=4,
        airframe="uav",
        vehicle=SimpleNamespace(s=state),
    )


def _make_all_targets_geometrically_visible(service: SimulationService, monkeypatch) -> None:
    monkeypatch.setattr(service, "_camera_view_polygon", lambda *_args, **_kwargs: [(0.0, 0.0)] * 4)
    monkeypatch.setattr(
        service,
        "_point_in_camera_view_with_polygon",
        lambda *_args, **_kwargs: True,
    )
    monkeypatch.setattr(
        service,
        "_target_info_visibility_context",
        lambda: {
            "entries_by_id": {},
            "ignored_ids": set(),
            "ignored_xy": [],
            "destroyed_xy": [],
        },
    )


def test_destroyed_report_id_does_not_hide_equal_raw_target_id(monkeypatch) -> None:
    service = SimulationService()
    # raw 4 was reported as 8; a different raw target 8 was later reported as
    # 9. Neither numeric overlap may suppress the live second target.
    service._target_id_map_0402 = {4: 8, 8: 9}
    service._destroyed_raw_target_ids = {4}
    service._destroyed_report_target_ids = {8}
    raw_target_8 = _target(8, x=100.0)
    service.targets = [raw_target_8]
    _make_all_targets_geometrically_visible(service, monkeypatch)

    visible = service._visible_tracking_targets(_viewer(), 5.7)

    assert visible == [raw_target_8]


def test_destroying_raw_target_records_namespaces_separately(monkeypatch) -> None:
    service = SimulationService()
    raw_target_4 = _target(4, x=100.0)
    raw_target_8 = _target(8, x=200.0)
    service.targets = [raw_target_4, raw_target_8]
    service._target_id_map_0402 = {4: 8}
    monkeypatch.setattr(service, "_update_target_info_destroyed", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(service, "_record_0402_target_destroyed", lambda *_args, **_kwargs: None)
    _make_all_targets_geometrically_visible(service, monkeypatch)

    service._handle_target_destroyed(raw_target_4, watcher_id=4)

    assert service._destroyed_raw_target_ids == {4}
    assert service._destroyed_report_target_ids == {8}
    assert service._visible_tracking_targets(_viewer(), 5.7) == [raw_target_8]
