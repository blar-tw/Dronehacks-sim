"""Crash-window timeline for a team13 bag: VIO vs PX4 vs IMU jolt, plus a left-camera contact sheet.
usage: python3 - BAG T0 T1 OUTDIR   (T0/T1 are absolute epoch seconds)"""
import sys, math, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, T0, T1, outdir = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
BASE = 1794120000.0  # print times as ts-BASE so they match launch.log (e.g. 225.64)
os.makedirs(outdir, exist_ok=True)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/visual_slam/tracking/odometry', '/visual_slam/status', '/oak/imu/data',
        '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_attitude',
        '/fmu/in/vehicle_visual_odometry', '/skybeta/setpoint/pose', '/oak/left/image_raw']
r.set_filter(rosbag2_py.StorageFilter(topics=[w for w in want if w in types]))
try:
    r.seek(int(T0 * 1e9))
except Exception as e:
    print('seek unavailable (%s); scanning from start' % e)
msg_t = {k: get_message(types[k]) for k in want if k in types}

def rpy(w, x, y, z):
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw

BIN = 0.1
bins = {}
def B(t):
    k = int((t - T0) / BIN)
    return bins.setdefault(k, {'amax': 0.0, 'gmax': 0.0})
events = []
prev_odom = None; last_vo = None; last_reset = None; last_evreset = None
frames = []; next_frame = T0 + 3.0  # frames every 0.25 s from T0+3
FRAME_DT = 0.25
last_odom_pos = None

while r.has_next():
    topic, data, ts = r.read_next(); t = ts * 1e-9
    if t < T0: continue
    if t > T1: break
    b = B(t); tp = t - BASE
    if topic == '/oak/left/image_raw':
        if t >= next_frame and len(frames) < 24:
            m = deserialize_message(data, msg_t[topic])
            a = np.frombuffer(m.data, dtype=np.uint8)
            ch = {'mono8': 1, 'bgr8': 3, 'rgb8': 3}.get(m.encoding, None)
            if ch:
                img = a.reshape(m.height, m.step // 1)[:, :m.width * ch].reshape(m.height, m.width, ch) if ch > 1 else a.reshape(m.height, m.step)[:, :m.width]
                frames.append((tp, img.copy(), last_odom_pos))
            else:
                events.append('%.2f image encoding %s not handled' % (tp, m.encoding))
            next_frame += FRAME_DT
        continue
    m = deserialize_message(data, msg_t[topic])
    if topic == '/visual_slam/tracking/odometry':
        p = m.pose.pose.position; q = m.pose.pose.orientation
        pos = np.array([p.x, p.y, p.z]); last_odom_pos = pos
        b['odom'] = pos; b['oyaw'] = rpy(q.w, q.x, q.y, q.z)[2]
        if prev_odom is not None:
            dt = t - prev_odom[0]; d = np.linalg.norm(pos - prev_odom[1])
            if dt > 0 and d / dt > 2.5:
                events.append('%.3f VIO JUMP %.2f m in %.3f s (%.1f m/s): (%.2f %.2f %.2f) -> (%.2f %.2f %.2f)' % (tp, d, dt, d / dt, *prev_odom[1], *pos))
            if dt > 0.15:
                events.append('%.3f VIO GAP %.3f s' % (tp, dt))
        prev_odom = (t, pos)
    elif topic == '/visual_slam/status':
        if m.vo_state != last_vo:
            events.append('%.3f vo_state %s -> %d' % (tp, last_vo, m.vo_state)); last_vo = m.vo_state
        b['trk_ms'] = max(b.get('trk_ms', 0), float(m.track_execution_time) * (1000 if m.track_execution_time < 1 else 1))
    elif topic == '/oak/imu/data':
        a = m.linear_acceleration; g = m.angular_velocity
        b['amax'] = max(b['amax'], math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2) / 9.81)
        b['gmax'] = max(b['gmax'], math.degrees(math.sqrt(g.x ** 2 + g.y ** 2 + g.z ** 2)))
    elif topic == '/fmu/out/vehicle_local_position':
        b['px4'] = np.array([m.y, m.x, -m.z]); b['px4v'] = math.hypot(m.vx, m.vy)
        b['db'] = m.dist_bottom if m.dist_bottom_valid else float('nan')
        rc = (m.xy_reset_counter, m.z_reset_counter, m.heading_reset_counter)
        if rc != last_reset:
            if last_reset is not None:
                events.append('%.3f EKF reset counters %s -> %s  delta_xy (%.2f %.2f) delta_z %.2f delta_heading %.1f deg' % (tp, last_reset, rc, m.delta_xy[0], m.delta_xy[1], m.delta_z, math.degrees(m.delta_heading)))
            last_reset = rc
    elif topic == '/fmu/out/vehicle_attitude':
        w, x, y, z = m.q; rr, pp, yy = rpy(w, x, y, z)
        b['att'] = (rr, pp, yy)
        b['tilt'] = max(b.get('tilt', 0), math.hypot(rr, pp))
    elif topic == '/fmu/in/vehicle_visual_odometry':
        if m.reset_counter != last_evreset:
            events.append('%.3f EV reset_counter -> %d (quality %d)' % (tp, m.reset_counter, m.quality)); last_evreset = m.reset_counter
        b['evq'] = m.quality
    elif topic == '/skybeta/setpoint/pose':
        p = m.pose.position; b['sp'] = np.array([p.x, p.y, p.z])

print('t      | VIO odom x y z        yaw  | PX4 local(ENU) x y z  spd | tilt  imu|a|max gyro max | rng  | setpoint x y z      | trk ms')
last = {}
for k in sorted(bins):
    b = bins[k]
    for key in ('odom', 'oyaw', 'px4', 'px4v', 'db', 'sp', 'att'):
        if key in b: last[key] = b[key]
    tp = T0 + k * BIN - BASE
    f3 = lambda v: '(%6.2f %6.2f %5.2f)' % tuple(v) if v is not None else '(   -      -     -  )'
    print('%.1f | %s %6.1f | %s %4.2f | %4.1f  %4.2f g  %5.0f d/s | %4.2f | %s | %s' % (
        tp, f3(last.get('odom')), last.get('oyaw', float('nan')), f3(last.get('px4')), last.get('px4v', float('nan')),
        b.get('tilt', float('nan')), b['amax'], b['gmax'], last.get('db', float('nan')), f3(last.get('sp')),
        ('%.0f' % b['trk_ms']) if 'trk_ms' in b else '-'))
print('\nEVENTS')
for e in events: print(e)

# contact sheet
try:
    import cv2
    tiles = []
    for tp, img, op in frames:
        if img.ndim == 3: img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        tl = cv2.resize(img, (320, 200))
        tl = cv2.cvtColor(tl, cv2.COLOR_GRAY2BGR)
        lab = 't=%.2f' % tp + ('  vio x=%.2f' % op[0] if op is not None else '')
        cv2.rectangle(tl, (0, 0), (320, 22), (0, 0, 0), -1)
        cv2.putText(tl, lab, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        tiles.append(tl)
    while len(tiles) % 4: tiles.append(np.zeros_like(tiles[0]))
    rows = [np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]
    cv2.imwrite(os.path.join(outdir, 'sheet.jpg'), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 80])
    print('\nsheet: %d frames -> %s' % (len(frames), os.path.join(outdir, 'sheet.jpg')))
except Exception as e:
    print('sheet failed:', e)
