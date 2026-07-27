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


def test_enemy_masking_proof_requires_a_traced_ray() -> None:
    """The shared masking proof must not range-gate or accept an untraced enemy.

    Both the incremental-append certification and the committed-package
    re-certification route through this one helper, so the invariant is checked
    once here instead of drifting between two copies of the loop.
    """

    import inspect

    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    source = inspect.getsource(ap._hide_point_masked_from_every_enemy)
    # No range gate on the enemy loop.
    assert "max_range_m=None," in source
    # Unknown / never-traced results are treated as exposed.
    assert 'assessment.get("evaluated") is not True' in source
    # A missing hide altitude is refused rather than defaulted to 0 m MSL,
    # which would sit under terrain and read as concealed from every enemy.
    assert 'emit(f"{tag} hide altitude unavailable; treated as exposed.")' in source
    assert 'float(hide.get("altitude") or 0.0)' not in source


def test_both_hide_certifiers_share_the_one_masking_proof() -> None:
    """Neither caller may re-implement the enemy loop with weaker rules."""

    import inspect

    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    for func in (
        ap._certify_incremental_append_hide_endpoint,
        ap._preserved_lah_cover_still_masked,
    ):
        source = inspect.getsource(func)
        assert "_hide_point_masked_from_every_enemy(" in source, func.__name__
        # The enemy LOS call itself lives only in the shared helper.
        assert "observer_height_m=float(ENEMY_OBSERVER_HEIGHT_M)" not in source, func.__name__


# ---------------------------------------------------------------------------
# 4. A committed LAH package was reused verbatim on every later replan, so its
#    cover point was never rechecked against enemies discovered afterwards.  A
#    contact that appears behind the masking ridge sees straight onto it.
# ---------------------------------------------------------------------------


def _committed_cover_scenario(monkeypatch, *, assessments, rows=None, path=None):
    """Drive _preserved_lah_cover_still_masked with a stubbed DEM/DB."""

    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    calls: list[dict] = []

    def _fake_los(**kwargs):
        calls.append(dict(kwargs))
        return assessments[min(len(calls) - 1, len(assessments) - 1)]

    monkeypatch.setattr(ap, "_attack_los_resource_dir", lambda: "resource")
    monkeypatch.setattr(ap, "evaluate_regional_los", _fake_los)
    monkeypatch.setattr(
        ap.db_paths, "get_db_subpath", lambda *_a, **_k: "FlightPath/1.json"
    )
    monkeypatch.setattr(
        ap,
        "read_json_cached",
        lambda *_a, **_k: (
            path
            if path is not None
            else {
                "aircraftID": 2,
                "lahWaypointList": [
                    {"coordinate": {"latitude": 37.95, "longitude": 127.35,
                                    "altitude": 700}}
                ],
            }
        ),
    )
    ctx = {
        "_preserved_source_attack_rows": (
            rows
            if rows is not None
            else [{"aircraftID": 2, "pathID": 1, "individualMissionID": 5,
                   "waypointID": 9, "targetID": 3}]
        )
    }
    contact = {
        "enemy_targets": [
            {"coordinate": {"latitude": 37.97, "longitude": 127.37, "altitude": 600}},
            {"coordinate": {"latitude": 37.99, "longitude": 127.39, "altitude": 610}},
            {"coordinate": {"latitude": 38.01, "longitude": 127.41, "altitude": 620}},
        ]
    }
    verdict = ap._preserved_lah_cover_still_masked(
        aircraft_id=2, ctx=ctx, enemy_contact=contact, emit=lambda _m: None
    )
    return verdict, calls, ctx


_MASKED = {"visible": False, "evaluated": True, "reason": "TERRAIN_BLOCKED"}
_SEEN = {"visible": True, "evaluated": True, "reason": "VISIBLE"}
_UNTRACED = {"visible": False, "evaluated": False, "reason": "OUT_OF_RANGE"}


def test_committed_cover_is_rechecked_against_every_current_enemy(monkeypatch) -> None:
    verdict, calls, _ctx = _committed_cover_scenario(monkeypatch, assessments=[_MASKED])

    assert verdict is True
    # All three contacts ray-traced, none of them range-gated away.
    assert len(calls) == 3
    assert all(call["max_range_m"] is None for call in calls)


def test_committed_cover_visible_to_one_new_enemy_is_discarded(monkeypatch) -> None:
    """The third contact sees it: reuse must be refused, not averaged away."""

    verdict, _calls, _ctx = _committed_cover_scenario(
        monkeypatch, assessments=[_MASKED, _MASKED, _SEEN]
    )

    assert verdict is False


def test_committed_cover_with_an_untraced_ray_is_discarded(monkeypatch) -> None:
    verdict, _calls, _ctx = _committed_cover_scenario(
        monkeypatch, assessments=[_UNTRACED]
    )

    assert verdict is False


def test_committed_cover_without_an_altitude_is_discarded(monkeypatch) -> None:
    """0 m MSL would sit under terrain and read as concealed from everything."""

    verdict, calls, _ctx = _committed_cover_scenario(
        monkeypatch,
        assessments=[_MASKED],
        path={
            "aircraftID": 2,
            "lahWaypointList": [
                {"coordinate": {"latitude": 37.95, "longitude": 127.35}}
            ],
        },
    )

    assert verdict is False
    assert calls == [], "no ray may be traced against an unknown altitude"


def test_committed_cover_without_a_committed_identity_fails_closed(monkeypatch) -> None:
    verdict, _calls, _ctx = _committed_cover_scenario(
        monkeypatch, assessments=[_MASKED], rows=[]
    )

    assert verdict is False


def test_an_unreadable_committed_path_fails_closed(monkeypatch) -> None:
    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    monkeypatch.setattr(ap, "_attack_los_resource_dir", lambda: "resource")
    monkeypatch.setattr(
        ap.db_paths, "get_db_subpath", lambda *_a, **_k: "FlightPath/1.json"
    )

    def _boom(*_a, **_k):
        raise OSError("path missing")

    monkeypatch.setattr(ap, "read_json_cached", _boom)
    verdict = ap._preserved_lah_cover_still_masked(
        aircraft_id=2,
        ctx={"_preserved_source_attack_rows": [{"aircraftID": 2, "pathID": 1}]},
        enemy_contact={"enemy_targets": [{"coordinate": {"latitude": 37.9, "longitude": 127.3}}]},
        emit=lambda _m: None,
    )

    assert verdict is False


def test_a_path_owned_by_another_aircraft_fails_closed(monkeypatch) -> None:
    verdict, _calls, _ctx = _committed_cover_scenario(
        monkeypatch,
        assessments=[_MASKED],
        path={
            "aircraftID": 3,  # not the aircraft being re-certified
            "lahWaypointList": [
                {"coordinate": {"latitude": 37.95, "longitude": 127.35,
                                "altitude": 700}}
            ],
        },
    )

    assert verdict is False


def test_enemy_set_fingerprint_changes_when_a_contact_is_added_or_moves() -> None:
    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    two = [
        {"coordinate": {"latitude": 37.97, "longitude": 127.37}},
        {"coordinate": {"latitude": 37.99, "longitude": 127.39}},
    ]
    base = ap._enemy_set_fingerprint(two)

    # Order must not matter.
    assert ap._enemy_set_fingerprint(list(reversed(two))) == base
    # Sub-metre jitter on a re-reported contact must not read as a new enemy.
    assert ap._enemy_set_fingerprint(
        [
            {"coordinate": {"latitude": 37.9700004, "longitude": 127.37}},
            {"coordinate": {"latitude": 37.99, "longitude": 127.39}},
        ]
    ) == base
    # A third contact, or one that genuinely moved, must not.
    assert ap._enemy_set_fingerprint(two + [{"coordinate": {"latitude": 38.01, "longitude": 127.41}}]) != base
    assert ap._enemy_set_fingerprint(
        [{"coordinate": {"latitude": 37.975, "longitude": 127.37}}, two[1]]
    ) != base


def test_discarding_preservation_also_withdraws_the_continuity_rows() -> None:
    """Otherwise the continuity check demands the package we are replacing."""

    import inspect

    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    source = inspect.getsource(ap._apply_attack_plan_overrides)
    assert "_preserved_lah_cover_still_masked(" in source
    assert 'ctx["_preserved_source_attack_rows"] = [' in source
    assert 'ctx["_preserved_lah_attack_aircraft_ids"] = [' in source
    assert 'ctx.pop("_incremental_attack_append", None)' in source


# ---------------------------------------------------------------------------
# 5. When full masking plus the required relay links is impossible, the
#    fallback ranked "has a UAV link" ABOVE "no enemy can see it".  For the
#    command aircraft - which requires 3 links - that routinely shipped a point
#    in plain view of every contact because it kept one link.
# ---------------------------------------------------------------------------


def test_concealment_outranks_the_relay_link_in_the_fallback() -> None:
    """A hidden point with no link must beat a seen point that keeps one."""

    import inspect

    from modules.monitoring.logic.dem_cover import hide_com

    source = inspect.getsource(hide_com.CommunicationHideAnalyzer.analyze)
    hidden_first = """                    key = (
                        int(enemy_visible[idx]),
                        0 if int(uav_links[idx]) > 0 else 1,"""
    assert hidden_first in source, "enemy count must be the primary fallback key"
    # The sentinel for a cell with no evaluated event must also be enemy-first,
    # otherwise an unscored cell outranks a genuinely hidden one.
    assert "quality = best_event_key[idx] or (999, 1, 0, float(\"inf\"))" in source


def test_refined_stage_also_puts_concealment_first() -> None:
    """The stage that ships the flown endpoint must use the same ordering."""

    import inspect

    from modules.monitoring.logic.dem_cover import hide_com_refine

    source = inspect.getsource(hide_com_refine)
    assert """            key = (
                int(enemy_visible[index]),
                0 if int(uav_links[index]) > 0 else 1,""" in source
    assert "quality = best_keys[index] or (999, 1, 0, float(\"inf\"))" in source


def test_the_fallback_key_ordering_prefers_hidden_over_linked() -> None:
    """Exercise the tuple ordering itself, not just its source text."""

    # (enemy_visible, no_link, -links, altitude_delta)
    hidden_no_link = (0, 1, 0, 10.0)
    seen_by_three_with_link = (3, 0, -1, 0.0)
    assert hidden_no_link < seen_by_three_with_link

    # Among equally hidden points, more links still wins.
    hidden_two_links = (0, 0, -2, 5.0)
    hidden_one_link = (0, 0, -1, 5.0)
    assert hidden_two_links < hidden_one_link < hidden_no_link

    # An unscored cell must never outrank a hidden one.
    unscored = (999, 1, 0, float("inf"))
    assert hidden_no_link < unscored
