"""S8: one test per failure trigger (hover instead of land, resume once fine)."""
import sys
from pathlib import Path

import numpy as np  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav import course as COURSE  # noqa: E402
from nav import planner  # noqa: E402
from nav.follower import Follower  # noqa: E402
from nav.guard import Guard  # noqa: E402
from nav.yaw import ViewScorer  # noqa: E402

AT = (0.0, 0.0, 0.6)
SP = (0.0, 0.0, 0.6)


def fresh():
    g = Guard()
    g.start(0.0)
    g.on_frame(0.0)
    return g


def test_vio_degraded_slows_only():
    g = fresh()
    g.on_vio("DEGRADED")
    assert g.slow and not g.check(1.0, AT, SP)


def test_vio_withheld_hovers_then_resumes():
    g = fresh()
    g.on_vio("WITHHELD")
    assert g.check(1.0, AT, SP) and g.why == "VIO withheld"
    g.on_vio("OK")
    assert g.check(1.5, AT, SP)                 # fine for less than resume_s
    assert not g.check(2.6, AT, SP)


def test_map_sigma_stops_and_looks():
    c = COURSE.load()
    wps = planner.plan(c)
    f = Follower(wps, ViewScorer(c, 0.8))
    f.step(0.0, 0.05, np.array([wps[0].x, wps[0].y]), 1.57, 0.20)
    assert f.mode == "look"


def test_no_camera_frames_10s_hovers():
    g = fresh()
    assert not g.check(9.0, AT, SP)
    assert g.check(10.5, AT, SP) and "camera" in g.why
    g.on_frame(11.0)
    assert not g.check(12.1, AT, SP)


def test_behind_setpoint_2s_hovers():
    g = fresh()
    far = (1.2, 0.0, 0.6)
    assert not g.check(1.0, far, SP)
    assert not g.check(2.5, far, SP)
    assert g.check(3.1, far, SP) and g.why == "behind the setpoint"
    low = (0.0, 0.0, 0.1)
    g2 = fresh()
    g2.check(1.0, low, SP)
    assert g2.check(3.1, low, SP)


def test_window_goes_through_without_gate_b():
    # decision: no Gate B; past window_wait_s the follower goes through anyway
    c = COURSE.load()
    wps = planner.plan(c)
    f = Follower(wps, ViewScorer(c, 0.8))
    i = next(k for k, w in enumerate(wps) if w.kind == "window") - 1
    f.i, f.s, f.mode = i, 0.0, "window"
    f.look_yaw = f._heading(i + 1)
    pos = np.array([wps[i + 1].x, wps[i + 1].y])
    f.step(0.0, 0.05, pos, f.look_yaw, 0.09)
    f.step(5.1, 0.05, pos, f.look_yaw, 0.09)
    assert f.mode == "go" and f.i == i + 1


def test_climb_lag_does_not_hover():
    g = fresh()
    ground = (0.0, 0.0, 0.0)
    assert not g.check(1.0, ground, SP, behind=False)
    assert not g.check(3.1, ground, SP, behind=False)
