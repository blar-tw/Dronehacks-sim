"""VioLoss: ev_bridge's policy for weak or lost VIO (decision-1.md)."""
import math

import pytest

from onboard.vio.vio_health import VioHealth, VioLoss, inflate_xyz


def test_good_tracking_is_ok():
    loss = VioLoss()
    loss.features(1.0, 200)
    d = loss.decide(1.01, True)
    assert d.send and d.state == "OK" and d.pos_var == 0 and d.vel_var == 0 and not d.reset


def test_no_feature_count_yet_is_unknown_not_weak():
    """Cold start / a cuVSLAM build without observations_count: send unchanged, never DEGRADED forever."""
    loss = VioLoss(pos_sigma_max=0.5, vel_sigma_max=0.4)
    d = loss.decide(0.0, True)
    assert d.send and d.state == "OK" and d.pos_var == 0 and d.vel_var == 0
    assert not loss.known(0.0) and loss.lost_s(0.0) == 0.0
    assert VioLoss().decide(500.0, True).state == "OK"      # still OK after a long silent run


def test_feature_counts_going_silent_fall_back_to_unknown_after_the_timeout():
    loss = VioLoss(features_timeout_s=1.0)
    loss.features(10.0, 200)
    loss.features(10.1, 5)
    assert loss.decide(10.9, True).state == "DEGRADED"      # within the timeout: weak is trusted
    d = loss.decide(11.5, True)                              # 1.4 s without a count: unknown
    assert d.state == "OK" and d.pos_var == 0 and not loss.known(11.5) and loss.lost_s(11.5) == 0.0
    loss.features(11.6, 5)                                   # counts back and still weak: DEGRADED again
    assert loss.decide(11.65, True).state == "DEGRADED"


def test_weak_tracking_grows_sigma_until_the_cap():
    loss = VioLoss(min_features=30, pos_sigma_rate=0.25, pos_sigma_max=0.5)
    loss.features(10.0, 200)
    loss.features(10.1, 5)
    d = loss.decide(11.0, True)
    assert d.send and d.state == "DEGRADED" and math.isclose(d.pos_var, 0.25 ** 2)
    loss.features(13.9, 5)
    assert math.isclose(loss.decide(14.0, True).pos_var, 0.5 ** 2)
    assert math.isclose(loss.lost_s(14.0), 4.0)


def test_weak_from_the_first_frame_ramps_from_zero_and_lost_s_stays_finite():
    """Starting in the dark: lost_s counts from the first count heard, never inf (goes on the wire)."""
    loss = VioLoss(pos_sigma_rate=0.25)
    loss.features(5.0, 0)
    d = loss.decide(5.0, True)
    assert d.state == "DEGRADED" and d.pos_var == 0.0 and loss.lost_s(5.0) == 0.0
    loss.features(5.9, 0)
    assert math.isclose(loss.decide(6.0, True).pos_var, 0.25 ** 2) and math.isfinite(loss.lost_s(6.0))


def test_negative_count_means_unavailable_and_counts_as_weak_like_cuvslam():
    loss = VioLoss()
    loss.features(1.0, -1)
    assert loss.decide(1.01, True).state == "DEGRADED"


def test_tracking_back_is_ok_again():
    loss = VioLoss()
    loss.features(1.0, 200)
    loss.features(2.0, 0)
    assert loss.decide(2.5, True).state == "DEGRADED"
    loss.features(3.1, 150)
    assert loss.decide(3.15, True).state == "OK"


def test_insane_samples_are_withheld_at_most_the_cap_then_sent_with_one_reset():
    loss = VioLoss(withhold_max_s=2.0, pos_sigma_max=0.5)
    loss.features(0.0, 200)
    assert not loss.decide(1.0, False).send
    assert not loss.decide(2.9, False).send
    d = loss.decide(3.0, False)
    assert d.send and d.forced and d.reset and math.isclose(d.pos_var, 0.25)
    d = loss.decide(3.1, False)
    assert d.send and d.forced and not d.reset
    assert loss.decide(3.2, True).state == "OK"
    assert not loss.decide(4.0, False).send          # a new episode starts its own cap


def test_a_fresh_violation_restarts_the_clock_and_is_never_forced_out():
    """Only settling samples (consistent with their predecessor) may be forced; a sample that is
    itself a jump/over-speed is garbage for EKF2 and restarts the 2 s cap."""
    loss = VioLoss(withhold_max_s=2.0)
    for i in range(60):                                       # 4 s of fresh violations at 15 Hz
        assert not loss.decide(i / 15, False, fresh=True).send
    assert not loss.decide(4.5, False).send                   # cap restarted at 59/15: not yet
    d = loss.decide(6.0, False)                               # settling, > 2 s after the last fresh one
    assert d.send and d.forced and d.reset
    assert not loss.decide(6.1, False, fresh=True).send       # another fresh jump: withheld again
    assert not loss.decide(8.0, False).send
    d = loss.decide(8.2, False)
    assert d.send and d.forced and d.reset                    # the new episode earns a new reset


def test_vio_health_marks_only_the_violating_sample_fresh():
    health = VioHealth(jump_m=1.0, quiet_s=5.0)
    health.update(0.0, (0.0, 0.0, 0.8))
    assert not health.fresh
    sane, _ = health.update(0.1, (1.5, 0.0, 0.8))             # the jump itself
    assert not sane and health.fresh
    sane, reason = health.update(0.2, (1.51, 0.0, 0.8))       # consistent with the jumped pose: settling only
    assert not sane and not health.fresh and reason.startswith("settling")


def test_a_jump_never_stops_vision_longer_than_the_cap():
    """VioHealth settles 5 s after a 1.5 m jump; PX4 must still hear vision within 2 s."""
    health, loss = VioHealth(jump_m=1.0, quiet_s=5.0), VioLoss(withhold_max_s=2.0)
    last_sent, worst_gap, resets = 0.0, 0.0, 0
    for i in range(15 * 20):                         # 20 s at 15 Hz
        t = i / 15
        loss.features(t, 200)
        x = 0.1 * t + (1.5 if t >= 5.0 else 0.0)
        sane, _ = health.update(t, (x, 0.0, 0.8))
        d = loss.decide(t, sane, health.fresh)
        resets += d.reset
        if d.send:
            worst_gap, last_sent = max(worst_gap, t - last_sent), t
    assert worst_gap <= 2.0 + 1 / 15 + 1e-9 and resets == 1


def test_runaway_vio_is_never_fed_to_px4():
    """7.5 m/s every frame (0.5 m per frame at 15 Hz): fresh each time, so nothing is sent for 6 s
    and the mission's 3 s WITHHELD landing acts before EKF2_NOAID_TOUT."""
    health, loss = VioHealth(max_speed_mps=5.0, jump_m=1.0), VioLoss(withhold_max_s=2.0)
    sent = 0
    for i in range(15 * 6):
        t = i / 15
        loss.features(t, 200)
        sane, _ = health.update(t, (0.5 * i, 0.0, 0.8))
        sent += loss.decide(t, sane, health.fresh).send
    assert sent == 1                                 # only the very first sample, before any violation


def test_tunnel_timeline_degraded_then_withheld_then_forced_with_one_reset():
    """Dark tunnel: ~0 features for 3 s (sigma ramps 0.25 m/s to the 0.5 m cap), then cuVSLAM
    relocalises 1.8 m at the exit: WITHHELD for 2 s, forced out with a reset, OK after settle."""
    health, loss = VioHealth(jump_m=1.0, quiet_s=5.0), VioLoss()
    states, resets = [], []
    for i in range(15 * 14):
        t = i / 15
        dark = 2.0 <= t < 5.0
        loss.features(t, 0 if dark else 200)
        x = 0.4 * t + (1.8 if t >= 5.0 else 0.0)
        sane, _ = health.update(t, (x, 0.0, 0.8))
        d = loss.decide(t, sane, health.fresh)
        states.append((t, d.state, math.sqrt(d.pos_var)))
        if d.reset:
            resets.append(t)
    by_t = lambda t: next(s for s in states if s[0] >= t - 1e-9)
    assert by_t(1.0)[1] == "OK"
    assert by_t(3.0)[1] == "DEGRADED" and math.isclose(by_t(3.0)[2], 0.25, abs_tol=0.02)
    assert by_t(4.5)[1] == "DEGRADED" and math.isclose(by_t(4.5)[2], 0.5)
    assert by_t(5.0)[1] == "WITHHELD" and by_t(6.9)[1] == "WITHHELD"
    assert by_t(7.1)[1] == "DEGRADED" and math.isclose(by_t(7.1)[2], 0.5)
    assert by_t(10.1)[1] == "OK"                     # VioHealth quiet (5 s after the jump) is over
    assert len(resets) == 1 and 7.0 <= resets[0] <= 7.1


def test_bad_parameters_are_refused():
    for kw in ({"withhold_max_s": 0.0}, {"pos_sigma_max": math.nan}, {"features_timeout_s": -1.0}):
        with pytest.raises(ValueError):
            VioLoss(**kw)


def test_inflate_xyz_adds_to_the_position_diagonal_only():
    cov = [0.0] * 36
    cov[1] = 0.3
    out = inflate_xyz(cov, 0.04)
    assert [out[i] for i in (0, 7, 14)] == [0.04] * 3
    assert out[1] == 0.3 and out[21] == 0.0 and math.isclose(sum(out), 0.3 + 0.12)


def test_inflate_xyz_survives_a_non_finite_or_negative_diagonal():
    """frames._diag maps NaN/negative to 0; the inflation must not be lost with it."""
    cov = [0.0] * 36
    cov[0], cov[7] = math.nan, -1.0
    out = inflate_xyz(cov, 0.25)
    assert out[0] == 0.25 and out[7] == 0.25 and out[14] == 0.25
    assert all(x == 0.0 for i, x in enumerate(out) if i not in (0, 7, 14))
