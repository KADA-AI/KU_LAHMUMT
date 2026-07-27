from types import SimpleNamespace

import pytest

from modules.sim.runtime.sim_service import SimulationService


def _service() -> SimulationService:
    service = SimulationService.__new__(SimulationService)
    service._friendly_attack_attempts = {}
    return service


def test_friendly_missile_hit_probability_is_doubled_and_capped() -> None:
    service = _service()
    attacker = SimpleNamespace(label="LAH2")

    far_probability, forced = service._friendly_attack_probability(
        simv=attacker,
        target_id=7,
        kind="missile",
        dist=8000.0,
        max_range=8000.0,
    )
    near_probability, _ = service._friendly_attack_probability(
        simv=attacker,
        target_id=8,
        kind="missile",
        dist=0.0,
        max_range=8000.0,
    )

    assert far_probability == pytest.approx(0.5)
    assert near_probability == pytest.approx(1.0)
    assert forced is False


def test_friendly_gun_hit_probability_is_doubled_and_capped() -> None:
    probability, forced = _service()._friendly_attack_probability(
        simv=SimpleNamespace(label="LAH3"),
        target_id=9,
        kind="gun",
        dist=500.0,
        max_range=8000.0,
    )

    assert probability == pytest.approx(1.0)
    assert forced is False
