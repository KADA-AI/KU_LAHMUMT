from __future__ import annotations

import pytest

from modules.sim.runtime.sim_service import SimulationService


@pytest.mark.parametrize("type_id", range(1, 7))
def test_all_enemy_target_types_remain_at_spawn_position(type_id: int) -> None:
    service = SimulationService()
    target = service._build_target(type_id=type_id, x=100.0, y=200.0, z=30.0)

    assert target.moving is False
    assert target.v == 0.0
    assert target.vmin == 0.0
    assert target.vmax == 0.0
    assert target.roam_center is None
    assert target.roam_radius is None
    assert target.threat is not None

    target.step(60.0)

    assert target.x == pytest.approx(100.0)
    assert target.y == pytest.approx(200.0)
    assert target.z == pytest.approx(30.0)


def test_pending_enemy_target_is_reported_as_static() -> None:
    service = SimulationService()

    pending = service._make_pending_target(type_id=1, lat=38.0, lon=127.0, alt=100.0)

    assert pending["moving"] is False
