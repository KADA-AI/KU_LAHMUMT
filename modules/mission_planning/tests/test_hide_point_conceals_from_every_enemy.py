"""A hide point must be proven concealed from *every* detected enemy.

The defects pinned here all shipped together and all had the same effect: a
point in plain view of a real contact was certified as cover.

1. The friendly weapon range (5 km) was reused as the enemy *observation*
   range.  An enemy farther than that was deleted from the masking proof, and
   the result reported enemyVisibleCount=0 for a point with no cover at all.
   Live geometry put every aircraft 5.3-6.3 km from its enemy - outside the
   old cap - so in that run every hide point was certified against no enemies.
2. An unevaluated (observer, point) pair was filled with ``+inf``, which is the
   neutral element of the ``np.min`` that builds the concealment ceiling, so a
   ray that was never traced raised no constraint.  The same sentinel meant
   fail-closed for UAV links and fail-open for enemies.
3. The configured 5 m ``hide_safety_margin_m`` reached only the coarse pass and
   the degraded fallback.  The native stage that produces the endpoint we
   actually fly certified on 1.25 m, and the route verifier on 0.25 m.
"""

from __future__ import annotations

import numpy as np

from modules.monitoring.logic.dem_cover import hide_com_refine
from modules.monitoring.logic.dem_cover.config import CoverConfig


class _Dem:
    height = width = 100
    cell_m = 10.0

    @staticmethod
    def native_to_rowcol(x: float, y: float):
        return float(y) / 10.0 + 0.5, float(x) / 10.0 + 0.5


def test_enemy_observation_range_is_unlimited_by_default() -> None:
    config = CoverConfig()
    assert config.enemy_observation_range_effective_m == float("inf")
    # The friendly weapon range must not be what gates concealment.
    assert config.weapon_range_m == 5000.0


def test_non_positive_observation_range_means_unlimited() -> None:
    for value in (0.0, -1.0, -5000.0):
        config = CoverConfig().with_overrides(enemy_observation_range_m=value)
        assert config.enemy_observation_range_effective_m == float("inf")


def test_explicit_observation_range_is_honoured() -> None:
    config = CoverConfig().with_overrides(enemy_observation_range_m=7000.0)
    assert config.enemy_observation_range_effective_m == 7000.0


def test_unevaluated_enemy_collapses_the_ceiling_instead_of_vanishing() -> None:
    """A never-traced enemy must read as 'sees us', not as 'no constraint'."""

    # One enemy with a real threshold, one that was never evaluated.
    with_sentinel = np.array([[500.0], [-np.inf]])
    assert np.min(with_sentinel, axis=0)[0] == -np.inf

    # The old +inf sentinel silently dropped the unevaluated enemy.
    legacy = np.array([[500.0], [np.inf]])
    assert np.min(legacy, axis=0)[0] == 500.0


def test_observer_requirements_fill_direction_is_caller_chosen() -> None:
    """Enemies fill unevaluated cells with -inf; UAV links keep +inf."""

    rows = np.array([0.5, 1.5], dtype=np.float64)
    cols = np.array([0.5, 1.5], dtype=np.float64)
    # Both candidates sit far from the observer below, so with a 1 m range no
    # ray is traced for either and the array keeps the caller's fill value.
    xs = np.array([5_000.0, 10_000.0], dtype=np.float64)
    ys = np.array([0.0, 0.0], dtype=np.float64)

    class _Observer:
        x = 0.0
        y = 0.0
        row = 0.5
        col = 0.5
        alt_m = 100.0

    config = CoverConfig()
    # max_distance_m of 1 m leaves every point out of range, so nothing is
    # traced and the whole array keeps the caller's fill value.
    enemy_fill = hide_com_refine._observer_requirements_chunked(
        _Dem(),
        config,
        _Observer(),
        rows=rows,
        cols=cols,
        xs=xs,
        ys=ys,
        max_distance_m=1.0,
        chunk_size=16,
        unevaluated_fill_m=-np.inf,
    )
    assert np.all(np.isneginf(enemy_fill))

    uav_fill = hide_com_refine._observer_requirements_chunked(
        _Dem(),
        config,
        _Observer(),
        rows=rows,
        cols=cols,
        xs=xs,
        ys=ys,
        max_distance_m=1.0,
        chunk_size=16,
    )
    assert np.all(np.isposinf(uav_fill))


def test_native_selection_margin_covers_the_tactical_margin() -> None:
    """The stage that ships the endpoint must honour hide_safety_margin_m."""

    from modules.mission_planning.pipelines import lah_enemy_contact

    config = CoverConfig()
    selection_margin_m = (
        float(config.hide_safety_margin_m)
        + hide_com_refine._ALTITUDE_MARGIN_M
        + hide_com_refine._ENEMY_CEILING_SELECTION_MARGIN_M
    )
    verification_margin_m = (
        float(config.hide_safety_margin_m)
        + lah_enemy_contact._ROUTE_CONCEALMENT_VERIFY_MARGIN_M
    )
    # Both stages clear the configured tactical margin ...
    assert selection_margin_m >= float(config.hide_safety_margin_m)
    assert verification_margin_m >= float(config.hide_safety_margin_m)
    # ... and selection stays strictly stricter than verification, so an
    # endpoint chosen at the selection ceiling still passes the route check
    # that recomputes thresholds over a different sample batch.
    assert selection_margin_m - verification_margin_m >= 1.0


def test_point_status_counts_unevaluated_enemies_as_visible() -> None:
    """A skipped enemy must reject the candidate, not pass it silently."""

    import inspect

    from modules.monitoring.logic.dem_cover.hide_com import CommunicationHideAnalyzer

    source = inspect.getsource(CommunicationHideAnalyzer._point_status)
    # The out-of-range branch has to record the enemy rather than `continue`
    # past it, and the total has to reach the caller's `enemy_visible` gate.
    assert "enemy_unevaluated += 1" in source
    assert "return enemy_visible + enemy_unevaluated" in source


def test_los_api_reports_whether_a_ray_was_actually_traced() -> None:
    """Two short-circuits return visible=False without sampling terrain."""

    import inspect

    from modules.monitoring.logic.dem_cover import los_api

    source = inspect.getsource(los_api.evaluate_regional_los)
    # Every return must declare whether a ray was traced ...
    assert source.count('"evaluated"') == source.count('"reason"')
    # ... and the two no-ray short-circuits that still say visible=False must
    # be marked unevaluated, or a caller reading only the boolean certifies
    # cover it never proved.
    for reason in ("OUT_OF_RANGE", "ENDPOINT_NOT_ABOVE_TERRAIN"):
        marker = '"reason": "%s",\n            "evaluated": False,' % reason
        assert marker in source, reason
    assert '"reason": "VISIBLE" if visible else "TERRAIN_BLOCKED",\n        "evaluated": True,' in source


def test_inherited_hide_certification_requires_a_traced_ray() -> None:
    """Re-certification must not range-gate or accept an untraced enemy."""

    import inspect

    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    source = inspect.getsource(ap._certify_incremental_append_hide_endpoint)
    # No range gate on the enemy loop.
    assert "max_range_m=None," in source
    # Unknown / never-traced results defer the append.
    assert 'assessment.get("evaluated") is not True' in source
    # A missing hide altitude is refused rather than defaulted to 0 m MSL,
    # which would sit under terrain and read as concealed from every enemy.
    assert 'emit("[ATTACK][APPEND][WARN] inherited hide altitude unavailable; append deferred.")' in source
    assert 'float(hide.get("altitude") or 0.0)' not in source
