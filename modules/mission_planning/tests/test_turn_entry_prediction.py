"""The predicted next-collab entry point has to land where the aircraft goes.

Three defects made a UAV miss the first point of a replanned mission:

1. Speed was taken from telemetry unconditionally.  A live feed shipped ~1/3.6
   of the true ground speed, and since radius = v/omega and lead distance =
   v*t, every entry-point estimate collapsed with it.
2. The projection was a closed-form constant-turn-rate arc, so the roll
   transition - the seconds spent rolling into the bank - was assumed away.
   That is exactly the phase a replan is triggered in.
3. The turn rate was clamped by the reference radius table, which is a nominal
   cruise turn; a real bank steeper than the table's implied ~20 deg was
   clipped and the projection under-turned.

The vehicle classes we fly against (our sim / external sim / verification sim)
share coordinated-turn physics but not roll response, so the roll slew and the
steady bank are learned per aircraft instead of assumed.
"""

from __future__ import annotations

import math

from modules.monitoring.logic import turn_radius_monitor as trm

G = 9.80665
EARTH_RADIUS_M = 6_371_008.8
REF_LAT = 38.0
REF_LON = 127.0


def _xy_to_latlon(x_m: float, y_m: float) -> tuple[float, float]:
    lat = REF_LAT + math.degrees(y_m / EARTH_RADIUS_M)
    lon = REF_LON + math.degrees(
        x_m / (EARTH_RADIUS_M * math.cos(math.radians(REF_LAT)))
    )
    return lat, lon


def _distance_m(lat1, lon1, lat2, lon2) -> float:
    lat = math.radians((float(lat1) + float(lat2)) / 2.0)
    return math.hypot(
        math.radians(float(lon2) - float(lon1)) * EARTH_RADIUS_M * math.cos(lat),
        math.radians(float(lat2) - float(lat1)) * EARTH_RADIUS_M,
    )


def _simulate_roll_into_turn(
    *,
    speed_mps: float = 40.0,
    roll_slew_dps: float = 3.0,
    steady_bank_deg: float = 24.0,
    turn_sign: int = 1,
    straight_s: float = 6.0,
    total_s: float = 40.0,
    dt: float = 0.2,
    telemetry_speed_scale: float = 1.0,
) -> list[dict]:
    """A UAV flying straight, then rolling into a coordinated turn."""

    samples: list[dict] = []
    t = 0.0
    x_m = y_m = 0.0
    heading_deg = 40.0
    roll_deg = 0.0
    while t <= total_s + 1e-9:
        if t >= straight_s:
            target_deg = steady_bank_deg * turn_sign
            step_deg = roll_slew_dps * dt * (1.0 if target_deg > roll_deg else -1.0)
            roll_deg = (
                target_deg
                if abs(target_deg - roll_deg) <= abs(step_deg)
                else roll_deg + step_deg
            )
        omega_dps = (
            math.degrees(G * math.tan(math.radians(roll_deg)) / speed_mps)
            if abs(roll_deg) > 1e-6
            else 0.0
        )
        mid_heading_rad = math.radians(heading_deg + (omega_dps * dt * 0.5))
        x_m += speed_mps * math.sin(mid_heading_rad) * dt
        y_m += speed_mps * math.cos(mid_heading_rad) * dt
        heading_deg = (heading_deg + (omega_dps * dt)) % 360.0
        lat, lon = _xy_to_latlon(x_m, y_m)
        samples.append(
            {
                "t": t,
                "lat": lat,
                "lon": lon,
                "alt": 1000.0,
                "speed": speed_mps * telemetry_speed_scale,
                "heading": heading_deg,
                "roll": roll_deg,
                "pitch": 0.0,
                "yaw": heading_deg,
            }
        )
        t += dt
    return samples


def _truth_at(samples: list[dict], t_target: float) -> tuple[float, float] | None:
    low = high = None
    for sample in samples:
        if sample["t"] <= t_target:
            low = sample
        if sample["t"] >= t_target and high is None:
            high = sample
            break
    if low is None or high is None:
        return None
    if high["t"] == low["t"]:
        return low["lat"], low["lon"]
    frac = (t_target - low["t"]) / (high["t"] - low["t"])
    return (
        low["lat"] + ((high["lat"] - low["lat"]) * frac),
        low["lon"] + ((high["lon"] - low["lon"]) * frac),
    )


def _feed(monkeypatch, samples: list[dict], *, until_s: float, overrides=None):
    """Run the real monitor over samples, isolated from persisted learning."""

    base = dict(trm._path_deviation_config())
    base.update(overrides or {})
    monkeypatch.setattr(trm, "_path_deviation_config", lambda: base)
    monkeypatch.setattr(trm, "_load_adaptive_aircraft_state", lambda _aid: {})
    monkeypatch.setattr(trm, "_save_adaptive_aircraft_state", lambda _aid, _state: None)

    monitor = trm.AircraftTurnMonitor(4, "UAV4")
    view = None
    for sample in samples:
        if sample["t"] > until_s + 1e-9:
            break
        monitor.update(
            timestamp_s=sample["t"],
            wall_time_s=sample["t"],
            latitude=sample["lat"],
            longitude=sample["lon"],
            altitude_m=sample["alt"],
            speed_mps=sample["speed"],
            raw_heading_deg=sample["heading"],
            roll_deg=sample["roll"],
            pitch_deg=sample["pitch"],
            yaw_deg=sample["yaw"],
            flying=1,
        )
        view = monitor.build_view(now_wall_time_s=sample["t"], include_paths=False)
    return monitor, view


def _entry_error_m(samples, view, *, at_s) -> float | None:
    predicted = getattr(view, "line_predicted_entry_coordinate", None)
    lead_s = getattr(view, "line_prediction_lead_s", None)
    if not predicted or not lead_s:
        return None
    truth = _truth_at(samples, at_s + float(lead_s))
    if truth is None:
        return None
    return _distance_m(
        predicted["latitude"], predicted["longitude"], truth[0], truth[1]
    )


def test_broken_telemetry_speed_is_replaced_by_track_speed(monkeypatch) -> None:
    """Positions are the honest input; a lying speed must not be believed."""

    samples = _simulate_roll_into_turn(telemetry_speed_scale=1.0 / 3.6)
    _monitor, view = _feed(monkeypatch, samples, until_s=8.0)

    assert view.speed_source == "track"
    # The true speed is 40 m/s; telemetry claimed ~11.
    assert view.speed_mps == view.track_speed_mps
    assert 35.0 <= float(view.speed_mps) <= 45.0
    assert float(view.reported_speed_mps) < 15.0


def test_consistent_telemetry_speed_is_kept(monkeypatch) -> None:
    """The correction only fires on a sustained disagreement."""

    samples = _simulate_roll_into_turn(telemetry_speed_scale=1.0)
    _monitor, view = _feed(monkeypatch, samples, until_s=8.0)

    assert view.speed_source == "reported"
    assert abs(float(view.speed_mps) - 40.0) < 1.0


def test_entry_prediction_survives_broken_telemetry_speed(monkeypatch) -> None:
    """The entry point must land in the same place either way."""

    good = _simulate_roll_into_turn(telemetry_speed_scale=1.0)
    broken = _simulate_roll_into_turn(telemetry_speed_scale=1.0 / 3.6)
    _m1, good_view = _feed(monkeypatch, good, until_s=8.0)
    _m2, broken_view = _feed(monkeypatch, broken, until_s=8.0)

    good_err = _entry_error_m(good, good_view, at_s=8.0)
    broken_err = _entry_error_m(broken, broken_view, at_s=8.0)
    assert good_err is not None and broken_err is not None
    assert good_err < 60.0
    assert broken_err < 60.0


def test_roll_transition_beats_the_constant_rate_arc(monkeypatch) -> None:
    """Mid roll-in, modelling the transition must beat assuming it away."""

    samples = _simulate_roll_into_turn()
    at_s = 9.0  # 3 s into an 8 s roll-in: the transition dominates

    _m_new, new_view = _feed(monkeypatch, samples, until_s=at_s)
    _m_old, old_view = _feed(
        monkeypatch,
        samples,
        until_s=at_s,
        overrides={"line_projection_attitude_enabled": False},
    )

    new_err = _entry_error_m(samples, new_view, at_s=at_s)
    old_err = _entry_error_m(samples, old_view, at_s=at_s)
    assert new_err is not None and old_err is not None
    assert new_view.line_prediction_model == "roll-transition-coordinated-turn"
    assert old_view.line_prediction_model == "constant-rate-arc"
    # The transition model should be dramatically closer, not marginally.
    assert new_err < old_err * 0.5


def test_roll_onset_predicts_a_turn_before_the_heading_shows_it(monkeypatch) -> None:
    """Roll leads heading, so a developing bank must not wait for the trend."""

    samples = _simulate_roll_into_turn()
    at_s = 8.0  # only 2 s of roll-in; heading has barely moved

    _monitor, view = _feed(monkeypatch, samples, until_s=at_s)
    error_m = _entry_error_m(samples, view, at_s=at_s)
    assert error_m is not None
    assert error_m < 60.0


def test_prediction_lead_includes_pipeline_latency(monkeypatch) -> None:
    """The point must describe activation time, not capture time."""

    samples = _simulate_roll_into_turn()
    _m_default, default_view = _feed(monkeypatch, samples, until_s=9.0)
    _m_zero, zero_view = _feed(
        monkeypatch,
        samples,
        until_s=9.0,
        overrides={"next_collab_entry_pipeline_latency_s": 0.0},
    )

    assert default_view.line_prediction_lead_s > zero_view.line_prediction_lead_s
    assert (
        abs(
            (default_view.line_prediction_lead_s - zero_view.line_prediction_lead_s)
            - 2.5
        )
        < 0.01
    )


def test_roll_response_is_learned_per_aircraft(monkeypatch) -> None:
    """A slow-rolling airframe must not be modelled with the default slew."""

    samples = _simulate_roll_into_turn(roll_slew_dps=3.0)
    monitor, view = _feed(monkeypatch, samples, until_s=20.0)

    learned = float(view.adaptive_roll_slew_dps)
    # Learned from the observed ~3 deg/s ramp, well below the 40 deg/s the
    # shared profile assumes for a generic airframe.
    assert 1.0 <= learned <= 12.0
    assert learned < 20.0
    assert 10.0 <= float(view.adaptive_steady_bank_deg) <= 45.0


def test_learned_state_is_persisted_with_the_other_adaptive_values() -> None:
    import inspect

    source = inspect.getsource(trm.AircraftTurnMonitor._save_adaptive_radius_state_if_due)
    for key in ("rollSlewDps", "steadyBankDeg"):
        assert key in source


def _simulate_reversal(
    *,
    speed_mps: float = 40.0,
    roll_slew_dps: float = 3.0,
    bank_deg: float = 24.0,
    established_s: float = 14.0,
    reverse_s: float = 20.0,
    total_s: float = 55.0,
    dt: float = 0.2,
) -> list[dict]:
    """Established LEFT turn, then a roll through level into a RIGHT turn."""

    samples: list[dict] = []
    t = 0.0
    x_m = y_m = 0.0
    heading_deg = 40.0
    roll_deg = 0.0
    while t <= total_s + 1e-9:
        if t < established_s:
            target_deg = -bank_deg           # left/negative bank
        elif t < reverse_s:
            target_deg = -bank_deg
        else:
            target_deg = bank_deg            # reverse to right/positive bank
        step_deg = roll_slew_dps * dt * (1.0 if target_deg > roll_deg else -1.0)
        roll_deg = (
            target_deg
            if abs(target_deg - roll_deg) <= abs(step_deg)
            else roll_deg + step_deg
        )
        omega_dps = (
            math.degrees(G * math.tan(math.radians(roll_deg)) / speed_mps)
            if abs(roll_deg) > 1e-6
            else 0.0
        )
        mid_heading_rad = math.radians(heading_deg + (omega_dps * dt * 0.5))
        x_m += speed_mps * math.sin(mid_heading_rad) * dt
        y_m += speed_mps * math.cos(mid_heading_rad) * dt
        heading_deg = (heading_deg + (omega_dps * dt)) % 360.0
        lat, lon = _xy_to_latlon(x_m, y_m)
        samples.append(
            {
                "t": t,
                "lat": lat,
                "lon": lon,
                "alt": 1000.0,
                "speed": speed_mps,
                "heading": heading_deg,
                "roll": roll_deg,
                "pitch": 0.0,
                "yaw": heading_deg,
            }
        )
        t += dt
    return samples


def test_roll_overrules_a_stale_trend_at_a_reversal(monkeypatch) -> None:
    """At a reversal the bank is right and the heading history is wrong.

    The heading trend is a fit over seconds of past motion, so just after the
    aircraft rolls the other way it still reports the turn that just ended.
    Following it sends the entry point the wrong way for the entire lead.
    """

    samples = _simulate_reversal()
    # Far enough past the reversal that the bank has clearly crossed over, but
    # while the multi-second heading fit still leans the old way.
    at_s = 28.0

    _m_new, new_view = _feed(monkeypatch, samples, until_s=at_s)
    _m_old, old_view = _feed(
        monkeypatch,
        samples,
        until_s=at_s,
        overrides={"line_projection_attitude_enabled": False},
    )

    new_err = _entry_error_m(samples, new_view, at_s=at_s)
    old_err = _entry_error_m(samples, old_view, at_s=at_s)
    assert new_err is not None and old_err is not None
    assert new_err < old_err


def test_frozen_position_feed_does_not_collapse_the_speed(monkeypatch) -> None:
    """A stalled feed is not a hover; substituting ~0 would collapse geometry."""

    samples = _simulate_roll_into_turn(telemetry_speed_scale=1.0 / 3.6)
    # Establish the correction, then freeze the position while time advances.
    frozen_from_s = 12.0
    last = None
    for sample in samples:
        if sample["t"] <= frozen_from_s:
            last = sample
            continue
        sample["lat"] = last["lat"]
        sample["lon"] = last["lon"]

    _monitor, view = _feed(monkeypatch, samples, until_s=20.0)
    # Whatever source it settles on, it must never hand downstream a speed that
    # would drive radius = v/omega to zero.
    assert view.speed_mps is not None
    assert float(view.speed_mps) > 5.0


def test_poisoned_legacy_adaptive_state_is_not_inherited(monkeypatch, tmp_path) -> None:
    """v1 state was fitted against unvalidated speed and must be discarded."""

    state_file = tmp_path / "path_deviation_adaptive_state.json"
    state_file.write_text(
        '{"version": 1, "aircraft": {"4": {"attitudeRateScale": 0.65, '
        '"attitudeRateSampleCount": 5000, "radiusScale": 1.45}}}',
        encoding="utf-8",
    )
    monkeypatch.setattr(trm, "_adaptive_state_path", lambda: state_file)
    assert trm._load_adaptive_aircraft_state(4) == {}

    state_file.write_text(
        '{"version": %d, "aircraft": {"4": {"attitudeRateScale": 1.1}}}'
        % trm._ADAPTIVE_STATE_VERSION,
        encoding="utf-8",
    )
    assert trm._load_adaptive_aircraft_state(4).get("attitudeRateScale") == 1.1


def test_planner_radius_band_rejects_a_collapsed_estimate() -> None:
    """A radius the airframe cannot fly must not reach the entry geometry."""

    from modules.mission_planning.runtime import next_collab_line_runner as runner

    # v/omega collapses to ~90 m when the speed behind it was wrong; the
    # airframe needs ~450 m at 40 m/s.
    radius_m, note = runner._band_clamp_turn_radius_m(
        estimate_m=90.0, speed_mps=40.0, speed_source="track"
    )
    assert radius_m is not None and radius_m > 300.0
    assert note == "band_clamped_low"

    # A plausible measured radius is left alone.
    radius_m, note = runner._band_clamp_turn_radius_m(
        estimate_m=470.0, speed_mps=40.0, speed_source="track"
    )
    assert radius_m == 470.0
    assert note is None


def test_untrusted_speed_source_falls_back_to_the_reference_radius() -> None:
    """A held-over speed cannot justify a specific-looking derived radius."""

    from modules.mission_planning.runtime import next_collab_line_runner as runner

    reference_m = runner._reference_turn_radius_for_speed_m(40.0)
    for source in ("track_hold", "stale", "none"):
        radius_m, note = runner._band_clamp_turn_radius_m(
            estimate_m=470.0, speed_mps=40.0, speed_source=source
        )
        assert radius_m == reference_m
        assert note is not None and source in note


def test_trigger_velocity_echo_stays_telemetry() -> None:
    """A payload that echoes 0401 must not be silently corrected."""

    import inspect

    from modules.monitoring.logic import path_deviation_replan

    source = inspect.getsource(path_deviation_replan)
    marker = source[source.index('"triggerVelocity"') :][:1200]
    assert "reported_speed_mps" in marker
    # The corrected value is still available, just not in the echo field.
    assert "resolvedSpeedMps" in marker
    assert "speedSource" in marker
