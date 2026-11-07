"""VioLoss: ev_bridge's policy for weak or lost VIO (decision-1.md)."""
import math

from onboard.vio.vio_health import VioHealth, VioLoss, inflate_xyz


def test_good_tracking_is_ok():
    loss = VioLoss()
    loss.features(1.0, 200)
    d = loss.decide(1.01, True)
    assert d.send and d.state == "OK" and d.pos_var == 0 and d.vel_var == 0 and not d.reset


def test_no_feature_count_yet_sends_at_max_sigma():
    d = VioLoss(pos_sigma_max=0.5, vel_sigma_max=0.4).decide(0.0, True)
    assert d.send and d.state == "DEGRADED"
    assert math.isclose(d.pos_var, 0.25) and math.isclose(d.vel_var, 0.16)


def test_weak_tracking_grows_sigma_until_the_cap():
    loss = VioLoss(min_features=30, pos_sigma_rate=0.25, pos_sigma_max=0.5)
    loss.features(10.0, 200)
    loss.features(10.1, 5)
    d = loss.decide(11.0, True)
    assert d.send and d.state == "DEGRADED" and math.isclose(d.pos_var, 0.25 ** 2)
    assert math.isclose(loss.decide(14.0, True).pos_var, 0.5 ** 2)


def test_tracking_back_is_ok_again():
    loss = VioLoss()
    loss.features(1.0, 200)
    loss.features(2.0, 0)
    assert loss.decide(3.0, True).state == "DEGRADED"
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


def test_a_jump_never_stops_vision_longer_than_the_cap():
    """VioHealth settles 5 s after a 1.5 m jump; PX4 must still hear vision within 2 s."""
    health, loss = VioHealth(jump_m=1.0, quiet_s=5.0), VioLoss(withhold_max_s=2.0)
    last_sent, worst_gap = 0.0, 0.0
    for i in range(15 * 20):                         # 20 s at 15 Hz
        t = i / 15
        loss.features(t, 200)
        x = 0.1 * t + (1.5 if t >= 5.0 else 0.0)
        sane, _ = health.update(t, (x, 0.0, 0.8))
        if loss.decide(t, sane).send:
            worst_gap, last_sent = max(worst_gap, t - last_sent), t
    assert worst_gap <= 2.0 + 1 / 15 + 1e-9


def test_inflate_xyz_adds_to_the_position_diagonal_only():
    cov = [0.0] * 36
    cov[1] = 0.3
    out = inflate_xyz(cov, 0.04)
    assert [out[i] for i in (0, 7, 14)] == [0.04] * 3
    assert out[1] == 0.3 and out[21] == 0.0 and math.isclose(sum(out), 0.3 + 0.12)


def test_a_runaway_vio_is_never_forced_into_px4():
    """cuVSLAM diverging (every sample 10 m/s off): each sample is a fresh insane one, so the bridge
    keeps withholding and the mission's WITHHELD landing can fire. Forcing it out after 2 s would
    feed PX4 the runaway."""
    health, loss = VioHealth(jump_m=1.0, max_speed_mps=5.0, quiet_s=5.0), VioLoss(withhold_max_s=2.0)
    sent = []
    for i in range(15 * 10):
        t = i / 15
        loss.features(t, 200)
        x = 0.1 * t if t < 2.0 else 0.2 + 10.0 * (t - 2.0)
        sane, _ = health.update(t, (x, 0.0, 0.8))
        if loss.decide(t, sane, fresh_bad=health.sample_bad).send:
            sent.append(t)
    assert not [t for t in sent if t > 2.0]          # the runaway starts after t = 2.0


def test_a_single_jump_is_still_forced_after_the_cap():
    """One jump then smooth motion: the settling samples are forced out after withhold_max_s."""
    health, loss = VioHealth(jump_m=1.0, quiet_s=5.0), VioLoss(withhold_max_s=2.0)
    last_sent, worst_gap = 0.0, 0.0
    for i in range(15 * 20):
        t = i / 15
        loss.features(t, 200)
        x = 0.1 * t + (1.5 if t >= 5.0 else 0.0)
        sane, _ = health.update(t, (x, 0.0, 0.8))
        if loss.decide(t, sane, fresh_bad=health.sample_bad).send:
            worst_gap, last_sent = max(worst_gap, t - last_sent), t
    assert worst_gap <= 2.0 + 1 / 15 + 1e-9
