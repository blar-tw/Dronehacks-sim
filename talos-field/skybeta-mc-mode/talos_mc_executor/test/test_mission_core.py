"""Tests for mission_core against a simple simulated vehicle.

No ROS needed (from the package root):
    PYTHONPATH=. python3 -m unittest discover -s test -v
On the drone (colcon): team_build.sh, then colcon test / pytest.

The simulated vehicle is deliberately crude: it moves towards the last
setpoint at up to 1.5 m/s while Offboard is active, sits on the ground at
z = 0, and turns towards a commanded yaw at 30 deg/s. Setpoints go through
the same altitude clamp the flight interface applies (0.3 m to 3.0 m above
the origin). It checks the sequencing and the safety behaviour, not
flight dynamics.
"""
import math
import unittest

from talos_mc_executor import mission_core as mc

DT = 0.1


class SimVehicle:

    def __init__(self, pos=(0.0, 0.0, 0.0), yaw=0.0, max_x=None,
                 yaw_offset_deg=0.0, yaw_rate_dps=30.0):
        self.pos = tuple(pos)
        # PX4 flies its own heading; odometry reports it shifted by a
        # constant offset (the competitor guide's frame caveat).
        self.yaw_offset = math.radians(yaw_offset_deg)
        self.px4_yaw = mc.wrap_pi(yaw + self.yaw_offset)
        self.yaw_rate = math.radians(yaw_rate_dps)
        self.max_x = max_x            # an "obstacle": cannot pass this x
        self.origin = tuple(pos)      # envelope origin = first odometry
        self.setpoint = None

    def clamp(self, sp):
        x, y, z = sp.position
        z = min(max(z, self.origin[2] + 0.3), self.origin[2] + 3.0)
        return mc.Setpoint((x, y, z), sp.yaw)

    def update(self, sp, offboard):
        if sp is not None:
            self.setpoint = self.clamp(sp)
        if not offboard or self.setpoint is None:
            return  # pilot has it: we model that as holding still
        err = [s - p for s, p in zip(self.setpoint.position, self.pos)]
        vel = [1.5 * e for e in err]
        speed = math.sqrt(sum(v * v for v in vel))
        if speed > 1.5:
            vel = [v * 1.5 / speed for v in vel]
        x, y, z = (p + v * DT for p, v in zip(self.pos, vel))
        if self.max_x is not None:
            x = min(x, self.max_x)
        self.pos = (x, y, max(z, 0.0))
        if self.setpoint.yaw is not None:
            # The interface sends our odom-frame yaw to PX4 uncorrected.
            d = mc.wrap_pi(self.setpoint.yaw - self.px4_yaw)
            step = self.yaw_rate * DT
            self.px4_yaw = mc.wrap_pi(self.px4_yaw + max(-step, min(step, d)))

    @property
    def yaw(self):
        return mc.wrap_pi(self.px4_yaw - self.yaw_offset)


def run(executor, vehicle, schedule, duration_s, t0=0.0):
    """schedule(t) -> dict of Inputs overrides. Returns per-tick records."""
    records = []
    for k in range(int(round(duration_s / DT))):
        t = t0 + k * DT
        flags = dict(state_received=True, px4_link=True, armed=False,
                     offboard_active=False, odom_age_s=0.0)
        flags.update(schedule(t))
        inp = mc.Inputs(now_s=t, position=vehicle.pos, yaw=vehicle.yaw,
                        **flags)
        sp = executor.step(inp)
        vehicle.update(sp, flags['offboard_active'])
        records.append(dict(t=t, sp=sp, state=executor.state,
                            mode=executor.mode.name if executor.mode else None,
                            pos=vehicle.pos, yaw=vehicle.yaw))
    return records


def armed_then_offboard(arm_at=1.0, offboard_at=2.0, disarm_at=None):
    def schedule(t):
        armed = t >= arm_at and (disarm_at is None or t < disarm_at)
        return dict(armed=armed,
                    offboard_active=armed and t >= offboard_at)
    return schedule


class FullMission(unittest.TestCase):

    def setUp(self):
        self.params = mc.Params(yaw_spin_enabled=True)
        self.logs = []
        self.ex = mc.MissionExecutor(
            self.params, lambda level, msg: self.logs.append((level, msg)))
        self.veh = SimVehicle(pos=(0.4, -0.2, 0.0), yaw=0.3)
        self.rec = run(self.ex, self.veh, armed_then_offboard(), 150.0)

    def test_sequence_matches_workshop_executor(self):
        self.assertEqual(
            self.ex.transitions,
            ['WAIT_FOR_OFFBOARD', 'TAKEOFF', 'WAYPOINTS', 'YAW_SPIN',
             'DESCEND', 'HOLD'])

    def test_every_waypoint_visited(self):
        start = (0.4, -0.2, 0.0)
        for wx, wy, wz in self.params.waypoints_enu_m:
            target = (start[0] + wx, start[1] + wy, start[2] + wz)
            closest = min(math.dist(r['pos'], target) for r in self.rec)
            self.assertLessEqual(closest, self.params.arrival_radius_m)

    def test_ends_holding_at_handoff_height(self):
        self.assertEqual(self.rec[-1]['state'], mc.HOLD)
        self.assertAlmostEqual(self.rec[-1]['sp'].position[2], 0.5, places=6)
        self.assertAlmostEqual(self.rec[-1]['pos'][2], 0.5, delta=0.05)
        self.assertIsNone(self.rec[-1]['sp'].yaw)

    def test_setpoints_never_jump(self):
        # Pose setpoints are not speed-limited by the interface, so ours
        # must be: consecutive commands differ by at most the fastest
        # configured speed times dt.
        fastest = max(self.params.climb_speed_mps,
                      self.params.cruise_speed_mps,
                      self.params.descend_speed_mps)
        flying = [r for r in self.rec if r['mode'] is not None]
        for a, b in zip(flying, flying[1:]):
            self.assertLessEqual(
                math.dist(a['sp'].position, b['sp'].position),
                fastest * DT + 1e-9)

    def test_carrot_stays_close_to_vehicle(self):
        p = self.params
        for r in self.rec:
            if r['mode'] is not None:
                self.assertLessEqual(math.dist(r['sp'].position, r['pos']),
                                     p.max_lead_m + 0.5 * DT + 1e-6)

    def test_yaw_spin_turns_a_full_circle(self):
        # Only the yaw spin commands a heading; every other step sends None.
        yaws = [r['sp'].yaw for r in self.rec
                if r['sp'] is not None and r['sp'].yaw is not None]
        turned = sum(mc.wrap_pi(b - a) for a, b in zip(yaws, yaws[1:]))
        self.assertAlmostEqual(turned, 2 * math.pi, delta=math.radians(2))
        rate = max(abs(mc.wrap_pi(b - a)) for a, b in zip(yaws, yaws[1:]))
        self.assertLessEqual(rate, math.radians(10.0) * DT + 1e-9)

    def test_leaves_ground_well_before_px4_preflight_disarm(self):
        # COM_DISARM_PRFLT disarms 10 s after arming if not airborne.
        airborne = next(r['t'] for r in self.rec if r['pos'][2] > 0.1)
        self.assertLess(airborne - 1.0, 4.0)

    def test_disarm_resets_for_next_flight(self):
        veh = SimVehicle()
        ex = mc.MissionExecutor(mc.Params())
        run(ex, veh, armed_then_offboard(disarm_at=200.0), 210.0)
        self.assertEqual(ex.state, mc.WAIT_FOR_OFFBOARD)
        self.assertEqual(ex.transitions[-1], 'WAIT_FOR_OFFBOARD')


class Safety(unittest.TestCase):

    def test_nothing_published_before_state_or_odometry(self):
        ex = mc.MissionExecutor(mc.Params())
        no_state = mc.Inputs(now_s=0.0, position=(0.0, 0.0, 0.0), yaw=0.0,
                             odom_age_s=0.0, state_received=False)
        self.assertIsNone(ex.step(no_state))
        no_odom = mc.Inputs(now_s=0.1, position=None, yaw=None,
                            odom_age_s=math.inf, state_received=True)
        self.assertIsNone(ex.step(no_odom))

    def test_waiting_commands_current_position(self):
        ex = mc.MissionExecutor(mc.Params())
        veh = SimVehicle(pos=(1.0, 2.0, 0.0))
        rec = run(ex, veh, lambda t: {}, 2.0)
        self.assertTrue(all(r['sp'].position == (1.0, 2.0, 0.0) for r in rec))
        self.assertEqual(ex.state, mc.WAIT_FOR_OFFBOARD)

    def test_offboard_lost_stops_the_mission(self):
        ex = mc.MissionExecutor(mc.Params())
        veh = SimVehicle()

        def schedule(t):
            return dict(armed=t >= 1.0,
                        offboard_active=2.0 <= t < 20.0)
        rec = run(ex, veh, schedule, 40.0)
        self.assertEqual(ex.state, mc.PAUSED)
        self.assertNotIn('YAW_SPIN', ex.transitions)
        after = [r for r in rec if r['t'] >= 20.0]
        self.assertTrue(all(r['mode'] is None for r in after))
        # Paused: we only echo the vehicle's own position (PX4 ignores it).
        self.assertTrue(all(r['sp'].position == r['pos'] for r in after[1:]))

    def test_rearm_after_pilot_takeover_starts_a_fresh_mission(self):
        ex = mc.MissionExecutor(mc.Params())
        veh = SimVehicle()

        def schedule(t):
            if t < 20.0:
                return dict(armed=t >= 1.0, offboard_active=2.0 <= t < 15.0)
            if t < 25.0:
                return dict(armed=False)
            return dict(armed=True, offboard_active=t >= 26.0)
        run(ex, veh, schedule, 30.0)
        self.assertEqual(ex.state, mc.RUNNING)
        self.assertEqual(ex.transitions.count('TAKEOFF'), 2)

    def test_node_started_mid_flight_does_not_take_off_again(self):
        ex = mc.MissionExecutor(mc.Params())
        veh = SimVehicle(pos=(3.0, 1.0, 1.2))
        rec = run(ex, veh, lambda t: dict(armed=True, offboard_active=True),
                  5.0)
        self.assertNotIn('TAKEOFF', ex.transitions)
        self.assertEqual(ex.state, mc.HOLD)
        self.assertTrue(all(r['sp'].position == (3.0, 1.0, 1.2) for r in rec))

    def test_stale_odometry_freezes_the_mission(self):
        ex = mc.MissionExecutor(mc.Params())
        veh = SimVehicle()

        def schedule(t):
            flags = armed_then_offboard()(t)
            if 12.0 <= t < 15.0:
                flags['odom_age_s'] = 1.0
            return flags
        rec = run(ex, veh, schedule, 20.0)
        frozen = [r for r in rec if 12.0 <= r['t'] < 15.0]
        self.assertEqual(len({r['sp'].position for r in frozen}), 1)
        self.assertEqual({r['mode'] for r in frozen}, {'WAYPOINTS'})
        resumed = [r['sp'].position for r in rec if r['t'] >= 15.0]
        self.assertGreater(len(set(resumed)), 1)

    def test_unreachable_waypoint_times_out_into_hold(self):
        params = mc.Params(step_timeout_s=5.0)
        ex = mc.MissionExecutor(params, lambda level, msg: None)
        veh = SimVehicle(max_x=1.0)   # first waypoint is at x = 2
        rec = run(ex, veh, armed_then_offboard(), 60.0)
        self.assertEqual(ex.transitions,
                         ['WAIT_FOR_OFFBOARD', 'TAKEOFF', 'WAYPOINTS', 'HOLD'])
        held = [r['sp'].position for r in rec if r['state'] == mc.HOLD]
        self.assertEqual(len(set(held)), 1)
        # Held where the vehicle got stuck, not 1 m further into the wall.
        self.assertAlmostEqual(held[0][0], 1.0, delta=0.05)

    def test_yaw_spin_off_by_default(self):
        ex = mc.MissionExecutor(mc.Params())
        run(ex, SimVehicle(), armed_then_offboard(), 120.0)
        self.assertEqual(ex.transitions,
                         ['WAIT_FOR_OFFBOARD', 'TAKEOFF', 'WAYPOINTS',
                          'DESCEND', 'HOLD'])

    def _spin(self, **vehicle_kwargs):
        ex = mc.MissionExecutor(
            mc.Params(yaw_spin_enabled=True, step_timeout_s=20.0))
        veh = SimVehicle(**vehicle_kwargs)
        rec = run(ex, veh, armed_then_offboard(), 200.0)
        return ex, rec

    def test_yaw_spin_tolerates_a_small_offset(self):
        ex, _ = self._spin(yaw_offset_deg=10.0)
        self.assertIn('DESCEND', ex.transitions)

    def test_yaw_spin_with_a_large_offset_stalls_into_hold(self):
        ex, rec = self._spin(yaw_offset_deg=40.0)
        self.assertEqual(ex.transitions[-2:], ['YAW_SPIN', 'HOLD'])
        self.assertNotIn('DESCEND', ex.transitions)
        # It never ran ahead of the vehicle while stalled.
        commanded = [r['sp'].yaw for r in rec
                     if r['sp'] is not None and r['sp'].yaw is not None]
        # It may creep until the lag reaches yaw_max_lead_deg, then stops.
        self.assertLess(max(abs(mc.wrap_pi(c - commanded[0]))
                            for c in commanded), math.radians(25.0))

    def test_yaw_spin_waits_for_a_slow_vehicle(self):
        ex, rec = self._spin(yaw_rate_dps=4.0)   # slower than the 10 deg/s ask
        self.assertIn('DESCEND', ex.transitions)
        for r in rec:
            if r['sp'] is not None and r['sp'].yaw is not None:
                lag = abs(mc.wrap_pi(r['sp'].yaw - r['yaw']))
                self.assertLess(lag, math.radians(25.0 + 1.0))

    def test_nan_odometry_is_never_commanded(self):
        ex = mc.MissionExecutor(mc.Params())
        bad = mc.Inputs(now_s=0.0, position=(float('nan'), 0.0, 0.0),
                        yaw=0.0, odom_age_s=0.0, state_received=True,
                        px4_link=True)
        self.assertIsNone(ex.step(bad))

    def test_stale_odometry_at_the_switch_does_not_start_a_mission(self):
        ex = mc.MissionExecutor(mc.Params())
        veh = SimVehicle(pos=(1.0, 1.0, 0.0))

        def schedule(t):
            flags = armed_then_offboard()(t)
            if t < 4.0:
                flags['odom_age_s'] = 2.0       # VIO dropped before the switch
            return flags
        rec = run(ex, veh, schedule, 5.0)
        stale = [r for r in rec if r['t'] < 4.0]
        self.assertTrue(all(r['sp'] is None for r in stale))
        self.assertEqual(ex.state, mc.RUNNING)
        self.assertEqual(ex.ctx.start, (1.0, 1.0, 0.0))

    def test_stale_state_freezes_without_publishing(self):
        ex = mc.MissionExecutor(mc.Params())
        veh = SimVehicle()

        def schedule(t):
            flags = armed_then_offboard()(t)
            if 10.0 <= t < 12.0:
                flags['state_received'] = False   # flight_interface silent
            return flags
        rec = run(ex, veh, schedule, 14.0)
        silent = [r for r in rec if 10.0 <= r['t'] < 12.0]
        self.assertTrue(all(r['sp'] is None for r in silent))
        self.assertEqual(ex.state, mc.RUNNING)

    def test_offboard_without_armed_does_not_start(self):
        ex = mc.MissionExecutor(mc.Params())
        veh = SimVehicle()

        def schedule(t):
            return dict(armed=False, offboard_active=t >= 1.0)
        run(ex, veh, schedule, 3.0)
        self.assertEqual(ex.state, mc.WAIT_FOR_OFFBOARD)
        self.assertNotIn('TAKEOFF', ex.transitions)


class Helpers(unittest.TestCase):

    def test_parse_waypoints(self):
        self.assertEqual(mc.parse_waypoints([1, 2, 3, 4, 5, 6]),
                         [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
        for bad in ([], [1.0, 2.0], [1.0, 2.0, float('nan')]):
            with self.assertRaises(ValueError):
                mc.parse_waypoints(bad)

    def test_validate(self):
        self.assertEqual(mc.validate(mc.Params()), [])
        self.assertEqual(mc.validate(mc.Params(yaw_spin_enabled=True)), [])
        bad = mc.Params(cruise_speed_mps=0.0, yaw_rate_dps=90.0,
                        waypoints_enu_m=[(1.0, 0.0, 0.0)])
        errors = '\n'.join(mc.validate(bad))
        self.assertIn('cruise_speed_mps', errors)
        self.assertIn('yaw_rate_dps', errors)
        self.assertIn('waypoint 1', errors)

    def test_validate_rejects_far_or_fast_setpoints(self):
        cases = dict(max_lead_m=20.0, cruise_speed_mps=5.0,
                     climb_speed_mps=2.0, handoff_altitude_m=0.1,
                     hover_altitude_m=4.0, odom_timeout_s=float('nan'))
        for name, value in cases.items():
            errors = mc.validate(mc.Params(**{name: value}))
            self.assertTrue(any(name in e for e in errors), (name, errors))
        high = mc.Params(waypoints_enu_m=[(0.0, 0.0, 3.5)])
        self.assertTrue(any('waypoint 1' in e for e in mc.validate(high)))

    def test_yaw_quaternion_round_trip(self):
        # The node encodes yaw as (0, 0, sin(y/2), cos(y/2)).
        for deg in (-170, -90, 0, 45, 179):
            y = math.radians(deg)
            got = mc.yaw_from_xyzw(0.0, 0.0, math.sin(y / 2), math.cos(y / 2))
            self.assertAlmostEqual(got, y, places=9)

    def test_carrot_waits_for_the_vehicle(self):
        carrot = (1.0, 0.0, 0.0)
        same = mc.advance_carrot(carrot, (5.0, 0.0, 0.0), (0.0, 0.0, 0.0),
                                 1.0, 0.1, max_lead=1.0)
        self.assertEqual(same, carrot)
        moved = mc.advance_carrot(carrot, (5.0, 0.0, 0.0), (0.5, 0.0, 0.0),
                                  1.0, 0.1, max_lead=1.0)
        self.assertAlmostEqual(moved[0], 1.1)
        landed = mc.advance_carrot((4.95, 0.0, 0.0), (5.0, 0.0, 0.0),
                                   (4.9, 0.0, 0.0), 1.0, 0.1, max_lead=1.0)
        self.assertEqual(landed, (5.0, 0.0, 0.0))
        nan_vehicle = mc.advance_carrot(carrot, (5.0, 0.0, 0.0),
                                        (float('nan'), 0.0, 0.0), 1.0, 0.1,
                                        max_lead=1.0)
        self.assertEqual(nan_vehicle, carrot)


if __name__ == '__main__':
    unittest.main()
