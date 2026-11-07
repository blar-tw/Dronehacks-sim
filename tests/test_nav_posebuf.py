"""nav.posebuf: the pose nearest a depth frame's stamp, and the yaw rate around it."""
import math

from nav.posebuf import PoseBuffer, yaw_rate_dps


def test_nearest_pose_within_tolerance():
    b = PoseBuffer(keep_s=1.0)
    for i in range(60):                            # 0 .. 1.18 s
        b.add(i * 0.02, i)
    assert b.at(0.501, 0.05) == (0.5, 25)
    assert b.at(5.0, 0.05) is None                 # no sample near that stamp
    assert b.at(0.0, 0.05) is None                 # dropped: older than keep_s
    assert b.latest()[1] == 59


def test_yaw_rate_across_the_wrap():
    b = PoseBuffer()
    for i in range(20):                            # 60 deg/s, through +-180 deg
        b.add(i * 0.02, math.radians(170 + 60 * i * 0.02) - (2 * math.pi if 170 + 60 * i * 0.02 > 180 else 0))
    assert abs(yaw_rate_dps(b, 0.2) - 60.0) < 1.0
    still = PoseBuffer()
    for i in range(20):
        still.add(i * 0.02, 1.0)
    assert yaw_rate_dps(still, 0.2) == 0.0


def test_clock_going_back_clears_the_buffer():
    b = PoseBuffer()
    b.add(10.0, "a")
    b.add(1.0, "b")
    assert b.latest() == (1.0, "b") and b.at(10.0, 0.1) is None
