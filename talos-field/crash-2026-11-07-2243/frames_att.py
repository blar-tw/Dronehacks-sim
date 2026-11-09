"""Save left-camera frames near given times and print roll/pitch. Read-only on the bag; writes PNGs to /tmp/crash_frames."""
import math, os, sys
import numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, t0 = sys.argv[1], float(sys.argv[2])
want_t = [float(x) for x in sys.argv[3].split(',')]
os.makedirs('/tmp/crash_frames', exist_ok=True)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/oak/left/image_raw', '/fmu/out/vehicle_attitude', '/fmu/out/vehicle_local_position']))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
pending = list(want_t); last_att = -9; vz = None
while r.has_next():
    topic, data, ts = r.read_next(); t = ts * 1e-9 - t0
    if t < min(want_t) - 1: continue
    if t > max(want_t) + 1.5: break
    m = deserialize_message(data, get_message(types[topic]))
    if topic.endswith('local_position'):
        vz = (m.vz, m.az if hasattr(m, 'az') else float('nan'))
    elif topic.endswith('attitude') and t - last_att >= 0.25:
        last_att = t; w, x, y, z = m.q
        roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))); pitch = math.degrees(math.asin(max(-1, min(1, 2*(w*y-z*x)))))
        print(f'{t:6.2f} roll {roll:6.1f} pitch {pitch:6.1f}  px4 vz_ned {vz[0] if vz else float("nan"):6.2f} az_ned {vz[1] if vz else float("nan"):6.2f}')
    elif topic.endswith('image_raw') and pending and t >= pending[0]:
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
        if m.encoding in ('bgr8', 'rgb8'): img = img.reshape(m.height, m.width, 3)
        small = cv2.resize(img, (512, 320))
        cv2.putText(small, f't={t:.2f}s', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imwrite(f'/tmp/crash_frames/left_{t:05.2f}.png', small); print(f'{t:6.2f} saved frame ({m.encoding} {m.width}x{m.height})')
        pending.pop(0)
