"""Timeline of a skybeta flight bag: VIO vs PX4 vs setpoints, resets, mode changes.
usage: python3 - <bag_dir_or_mcap> <t0_unix_s> [t_start_rel] [t_end_rel]   (script on stdin)
Times printed relative to t0 (Offboard start). Read-only.
"""
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, t0 = sys.argv[1], float(sys.argv[2])
t_lo = float(sys.argv[3]) if len(sys.argv) > 3 else -5.0
t_hi = float(sys.argv[4]) if len(sys.argv) > 4 else 1e9
topics = ['/visual_slam/tracking/odometry', '/visual_slam/status', '/fmu/out/vehicle_local_position',
          '/fmu/in/vehicle_visual_odometry', '/fmu/in/trajectory_setpoint', '/skybeta/setpoint/pose',
          '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_land_detected']
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=topics))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
S = {}; last = {}; next_row = None
path = 0.0; prev_v = None; vmax = 0.0; start_xy = None
def f(v, n=2): return 'nan' if v is None or (isinstance(v, float) and math.isnan(v)) else f'{v:.{n}f}'
while r.has_next():
    topic, data, ts = r.read_next()
    t = ts * 1e-9 - t0
    if t < t_lo: continue
    if t > t_hi: break
    m = deserialize_message(data, get_message(types[topic]))
    if topic == '/visual_slam/tracking/odometry':
        p = m.pose.pose.position; S['odom'] = (p.x, p.y, p.z)
    elif topic == '/visual_slam/status':
        if m.vo_state != last.get('vo'):
            print(f'{t:7.2f} EVENT vo_state {last.get("vo")} -> {m.vo_state}'); last['vo'] = m.vo_state
    elif topic == '/fmu/out/vehicle_local_position':
        S['px4'] = (m.y, m.x, -m.z)  # ENU
        S['vel'] = math.hypot(m.vx, m.vy)
        S['db'] = (m.dist_bottom, m.dist_bottom_valid)
        rc = (m.xy_reset_counter, m.z_reset_counter)
        if rc != last.get('rc'):
            if 'rc' in last:
                print(f'{t:7.2f} EVENT PX4 reset xy {last["rc"][0]}->{rc[0]} z {last["rc"][1]}->{rc[1]}  delta_xy ({f(m.delta_xy[0])}, {f(m.delta_xy[1])}) delta_z {f(m.delta_z)}  px4 now ENU ({f(m.y)}, {f(m.x)}, {f(-m.z)})')
            last['rc'] = rc
        if t >= 0:
            if prev_v is not None:
                dt = t - prev_v[0]
                if 0 < dt < 0.5:
                    path += math.hypot(m.vx, m.vy) * dt
            prev_v = (t, m.vx, m.vy); vmax = max(vmax, math.hypot(m.vx, m.vy))
    elif topic == '/fmu/in/vehicle_visual_odometry':
        if m.reset_counter != last.get('evrc'):
            if 'evrc' in last:
                print(f'{t:7.2f} EVENT vision reset_counter {last["evrc"]} -> {m.reset_counter}  (bridge pos NED {f(m.position[0])}, {f(m.position[1])}, {f(m.position[2])})')
            last['evrc'] = m.reset_counter
    elif topic == '/fmu/in/trajectory_setpoint':
        S['tsp'] = (m.position[1], m.position[0], -m.position[2])  # ENU
    elif topic == '/skybeta/setpoint/pose':
        p = m.pose.position; S['sp'] = (p.x, p.y, p.z)
    elif topic == '/fmu/out/vehicle_status_v1':
        k = (m.nav_state, m.arming_state)
        if k != last.get('st'):
            print(f'{t:7.2f} EVENT nav_state {m.nav_state} arming {m.arming_state}'); last['st'] = k
    elif topic == '/fmu/out/vehicle_land_detected':
        if m.landed != last.get('ld'):
            print(f'{t:7.2f} EVENT landed {m.landed}'); last['ld'] = m.landed
    if next_row is None: next_row = t
    if t >= next_row and 'odom' in S and 'px4' in S:
        next_row = t + 0.5
        o, p = S['odom'], S['px4']
        tsp = S.get('tsp'); sp = S.get('sp')
        gap = '' if tsp is None else f' PX4target-PX4pos {f(math.hypot(tsp[0]-p[0], tsp[1]-p[1]))}m'
        print(f'{t:7.2f} odom ({f(o[0])},{f(o[1])},{f(o[2])}) px4 ({f(p[0])},{f(p[1])},{f(p[2])}) v_xy {f(S["vel"])} '
              f'sp {None if sp is None else tuple(round(c, 2) for c in sp)} px4target {None if tsp is None else tuple(round(c, 2) for c in tsp)}{gap} '
              f'range {f(S["db"][0])}/{int(S["db"][1])} path {f(path, 1)}m')
print(f'SUMMARY horizontal path after Offboard {path:.1f} m, max horizontal speed {vmax:.2f} m/s')
