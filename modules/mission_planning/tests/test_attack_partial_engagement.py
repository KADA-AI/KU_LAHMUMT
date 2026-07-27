"""One unengageable target must not cancel the ones that can be hit.

The firing point is solved per target from the certified hide point. A target
whose pop-up exceeds the airframe ceiling used to `return` an abort package for
the whole aircraft on the first failure, so an attack option that contained one
out-of-reach contact produced no attack at all - only the exclusion plan.
"""

from __future__ import annotations

import inspect
from typing import Any

from modules.mission_planning.replanning.triggers.attack import pipeline as ap


def _builder_source() -> str:
    return inspect.getsource(ap._build_lah_attack_sequence_package)


def test_every_firing_point_is_solved_before_any_target_is_abandoned() -> None:
    source = _builder_source()

    assert "firing_point_by_index" in source
    assert "unengageable_targets" in source
    # The pre-pass runs ahead of the per-target loop.
    assert source.index("unengageable_targets") < source.index(
        "for idx, assigned in enumerate(valid_targets):"
    )


def test_a_clean_sweep_of_popup_failures_uses_precomputed_attack_points() -> None:
    source = _builder_source()

    assert "if unengageable_targets and not engageable_targets:" not in source
    assert "using each target's precomputed closest attack point instead" in source


def test_a_partial_popup_failure_keeps_every_assigned_target() -> None:
    source = _builder_source()

    assert "if unengageable_targets:" in source
    tail = source.split("if unengageable_targets:", 1)[1][:700]
    assert "precomputed closest attack point" in tail
    assert "Dropping" not in tail
    assert "valid_targets = engageable_targets" in source


def test_the_loop_reuses_the_solved_firing_point_instead_of_resolving() -> None:
    """Re-solving would double the DEM cost of every attack replan."""

    source = _builder_source()

    assert "attack_from_hide = (\n            firing_point_by_index.get(int(idx))" in source
    # Exactly one call site remains: the pre-pass.
    assert source.count("_attack_coordinate_at_hide_endpoint(") == 1
