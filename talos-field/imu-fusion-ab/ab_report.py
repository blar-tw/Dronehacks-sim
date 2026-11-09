"""IMU off vs on replay report for the 22:43 flight: rest drift, flight smoothness, wall-contact jumps.
usage: python3 ab_report.py <flight.mcap> <out_off> <out_on>   (times printed as epoch - 1794120000)"""
import sys, glob
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BASE = 1794120000.0

def read(uri, odom_topic, status_topic, t_end=None):
    files = sorted(glob.glob(uri + '/*.mcap')) if not uri.endswith('.mcap') else [uri]
    odom, bad = [], 0
    for f in files:
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=f, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in (odom_topic, status_topic) if t in types]))
        while r.has_next():
            topic, data, ts = r.read_next()
            if t_end and ts * 1e-9 > t_end: break
            m = deserialize_message(data, get_message(types[topic]))
            if topic == odom_topic:
                st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                p = m.pose.pose.position; odom.append((st, p.x, p.y, p.z))
            elif m.vo_state != 1:
                bad += 1
    a = np.array(sorted(odom)); return a, bad

flight, off_dir, on_dir = sys.argv[1:4]
live, live_bad = read(flight, '/visual_slam/tracking/odometry', '/visual_slam/status', t_end=BASE + 230)
srcs = {'live (no IMU)': (live, live_bad)}
for name, d in (('replay IMU off', off_dir), ('replay IMU on', on_dir)):
    srcs[name] = read(d, '/replay/odometry', '/replay/status')

def window(a, t0, t1):
    return a[(a[:, 0] >= BASE + t0) & (a[:, 0] <= BASE + t1)]

def at(a, t):
    i = np.searchsorted(a[:, 0], BASE + t)
    return a[min(i, len(a) - 1), 1:] if len(a) else None

print('%-16s %6s %8s | %-26s | %-30s | %-34s' % ('source', 'msgs', 'notTrk', 'REST 201-215.5 max drift m', 'FLIGHT 218.3-225.4 max speed', 'WALL 225.4-226.2 max jump (m/s)'))
for name, (a, bad) in srcs.items():
    if len(a) < 5:
        print('%-16s %6d  (too few messages)' % (name, len(a))); continue
    rest = window(a, 201.0, 215.5)
    drift = np.linalg.norm(rest[:, 1:] - rest[0, 1:], axis=1).max() if len(rest) > 2 else float('nan')
    def maxspeed(w):
        if len(w) < 3: return float('nan')
        dt = np.diff(w[:, 0]); dp = np.linalg.norm(np.diff(w[:, 1:], axis=0), axis=1)
        ok = dt > 1e-3
        return (dp[ok] / dt[ok]).max()
    fl = window(a, 218.3, 225.4); wl = window(a, 225.4, 226.2)
    print('%-16s %6d %8d | %26.3f | %30.2f | %34.2f' % (name, len(a), bad, drift, maxspeed(fl), maxspeed(wl)))

print('\nposition relative to t=215.5 (x y z), every 0.5 s')
print('  t     ' + ''.join('%-27s' % k for k in srcs))
for t in np.arange(215.5, 227.01, 0.5):
    row = '%6.1f  ' % t
    for name, (a, _) in srcs.items():
        if len(a) < 5: row += '%-27s' % '-'; continue
        p, p0 = at(a, t), at(a, 215.5)
        row += '(%6.2f %6.2f %5.2f)       ' % tuple(p - p0)
    print(row)
