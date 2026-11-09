"""Compare VIO height from the live flight vs replays (IMU off / on) against PX4's height.
usage: python3 compare_ab.py <flight.mcap> <t0_unix> <replay_off_dir> <replay_on_dir> [t_from t_to]
Heights are relative to each source's value at t_from (drone at rest), metres, ENU up."""
import bisect, math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def read(uri, topics, use_header):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {t: [] for t in topics}
    while r.has_next():
        topic, data, ts = r.read_next()
        m = deserialize_message(data, get_message(types[topic]))
        if topic.endswith('odometry'):
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 if use_header else ts * 1e-9
            p = m.pose.pose.position; out[topic].append((st, (p.x, p.y, p.z)))
        elif topic.endswith('local_position'):
            out[topic].append((ts * 1e-9, (m.y, m.x, -m.z, m.dist_bottom)))
        elif topic.endswith('status'):
            out[topic].append((ts * 1e-9, m.vo_state))
    return out

flight, t0, off_dir, on_dir = sys.argv[1], float(sys.argv[2]), sys.argv[3], sys.argv[4]
t_from = float(sys.argv[5]) if len(sys.argv) > 5 else -4.0
t_to = float(sys.argv[6]) if len(sys.argv) > 6 else 7.2
live = read(flight, ['/visual_slam/tracking/odometry', '/fmu/out/vehicle_local_position'], True)
srcs = {'live(no IMU)': live['/visual_slam/tracking/odometry'], 'PX4': live['/fmu/out/vehicle_local_position']}
for name, d in (('replay IMU off', off_dir), ('replay IMU on', on_dir)):
    rr = read(d, ['/replay/odometry', '/replay/status'], True)
    srcs[name] = rr['/replay/odometry']
    lost = [s for s in rr['/replay/status'] if s[1] != 1]
    print(f'{name}: {len(rr["/replay/odometry"])} odometry msgs, {len(lost)} status msgs not tracking')

def at(series, t):
    ts = [s[0] for s in series]
    i = bisect.bisect_left(ts, t)
    if not series or i >= len(series) or abs(series[i][0] - t) > 0.2:
        return None
    return series[i][1]

base = {k: at(v, t0 + t_from) for k, v in srcs.items()}
print('\n   t  ' + ''.join(f'{k:>16}' for k in srcs) + '   rangefinder')
t = t_from
while t <= t_to + 1e-9:
    row = f'{t:5.1f} '
    for k, v in srcs.items():
        cur = at(v, t0 + t)
        row += f'{(cur[2] - base[k][2]) if cur and base[k] else float("nan"):16.2f}'
    px = at(srcs['PX4'], t0 + t)
    row += f'   {px[3]:.2f}' if px else ''
    print(row)
    t += 0.5
