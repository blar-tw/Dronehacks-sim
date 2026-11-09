#!/usr/bin/env python3
"""Stereo depth accuracy check for skybeta-05's OAK-D-S2. Team-side and read-only:
it only subscribes, so the VIO -> PX4 path is untouched.

The driver undistorts both mono cameras on-device (left/right.i_undistorted = true), so
camera_info carries D = 0 and the K that matches the images. It does NOT publish a usable
stereo rectification (R = I, and the baseline term P[3] sits on the LEFT info, where ROS
stereo tools do not look). This tool therefore rectifies with the factory left->right
extrinsics from the EEPROM backup, runs SGBM, and reports:

  centre   median stereo depth in a centre box (for tape-measure checks against a wall)
  board    if the team's 6x6 tag36h11 AprilGrid (20 mm tags, 6 mm gaps) is in view:
           its distance from PnP on the left image (independent of the stereo baseline)
           vs the stereo depth at the same tag corners -> depth error in %

Run inside the skybeta container (ROS sourced), with the board held still in view:
    python3 depth_check.py --seconds 30
"""
import argparse, json, math, time
from collections import deque
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

ap = argparse.ArgumentParser()
ap.add_argument('--calib', default='/root/team_ws/oak-calibration/factory-calibration-backup.json')
ap.add_argument('--left', default='/oak/left/image_mono')
ap.add_argument('--right', default='/oak/right/image_mono')
ap.add_argument('--scale', type=float, default=0.5, help='SGBM resolution relative to 1024x640')
ap.add_argument('--min-depth', type=float, default=0.25)
ap.add_argument('--hz', type=float, default=2.0)
ap.add_argument('--seconds', type=float, default=30.0)
ap.add_argument('--box', type=float, default=0.2, help='centre box size as a fraction of width/height')
ap.add_argument('--out', default='', help='optional JSON-lines output')
args = ap.parse_args()

# ---- factory stereo extrinsics (left = socket 1 / CAM_B, right = socket 2 / CAM_C) ----
cal = json.load(open(args.calib))
cams = {int(s): v for s, v in cal['cameraData']}
ext = cams[1]['extrinsics']
assert int(ext['toCameraSocket']) == 2, ext
R_lr = np.array(ext['rotationMatrix'], dtype=np.float64)
T_lr = np.array([ext['translation'][k] for k in 'xyz'], dtype=np.float64) / 100.0  # cm -> m

# AprilGrid layout as verified by the team (oak-kalibr/audit_visual_poses.py)
PITCH, TAG = 0.026, 0.020
def board_corners(tag):
    x, y = (tag % 6) * PITCH, (tag // 6) * PITCH
    return [[x + TAG, y, 0], [x, y, 0], [x, y + TAG, 0], [x + TAG, y + TAG, 0]]
aparams = cv2.aruco.DetectorParameters_create()
aparams.markerBorderBits = 2
aparams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def to_gray(m):
    a = np.frombuffer(m.data, dtype=np.uint8)
    if m.encoding in ('mono8', '8UC1'):
        return a.reshape(m.height, m.step)[:, :m.width]
    img = a.reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY if m.encoding == 'bgr8' else cv2.COLOR_RGB2GRAY)


class DepthCheck(Node):
    def __init__(self):
        super().__init__('team13_depth_check')
        self.info = {}; self.img = {'left': deque(maxlen=8), 'right': deque(maxlen=8)}; self.rect = None; self.rows = []
        self.count = {'left': 0, 'right': 0}; self.dts = []
        for side, topic in (('left', args.left), ('right', args.right)):
            self.create_subscription(Image, topic, lambda m, s=side: self.on_img(s, m), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, f'/oak/{side}/camera_info', lambda m, s=side: self.info.setdefault(s, m), qos_profile_sensor_data)
        self.create_timer(1.0 / args.hz, self.tick)
        self.t_end = time.monotonic() + args.seconds
        self.out = open(args.out, 'a') if args.out else None

    def on_img(self, side, m):
        self.img[side].append(m); self.count[side] += 1

    def best_pair(self):
        st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        best = None
        for l in self.img['left']:
            for r in self.img['right']:
                d = abs(st(l) - st(r))
                if best is None or d < best[0]:
                    best = (d, l, r)
        return best

    def setup(self):
        li, ri = self.info['left'], self.info['right']
        W, H = li.width, li.height
        K1 = np.array(li.k).reshape(3, 3); K2 = np.array(ri.k).reshape(3, 3)
        if any(abs(x) > 1e-9 for x in list(li.d) + list(ri.d)):
            self.get_logger().warn('camera_info D is non-zero: images may not be undistorted; results will be off')
        s = args.scale; ws, hs = int(round(W * s)), int(round(H * s))
        S = np.diag([s, s, 1.0]); K1s, K2s = S @ K1, S @ K2
        z5 = np.zeros(5)
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1s, z5, K2s, z5, (ws, hs), R_lr, T_lr,
                                                   flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        f = P1[0, 0]; B = -P2[0, 3] / P2[0, 0]
        ndisp = int(16 * math.ceil(f * B / args.min_depth / 16))
        self.rect = dict(W=W, H=H, ws=ws, hs=hs, K1=K1, R1=R1, P1=P1, f=f, B=B,
                         m1=cv2.initUndistortRectifyMap(K1s, z5, R1, P1, (ws, hs), cv2.CV_32FC1),
                         m2=cv2.initUndistortRectifyMap(K2s, z5, R2, P2, (ws, hs), cv2.CV_32FC1))
        self.sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=ndisp, blockSize=5,
                                          P1=8 * 25, P2=32 * 25, disp12MaxDiff=1, uniquenessRatio=10,
                                          speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        pub_B = -(li.p[3] if abs(li.p[3]) > abs(ri.p[3]) else ri.p[3]) / li.p[0]
        print(f'rectified {ws}x{hs}: f={f:.1f}px  baseline={B*1000:.2f} mm (camera_info implies {abs(pub_B)*1000:.2f} mm)  '
              f'disparities={ndisp} -> min depth {f*B/ndisp:.2f} m', flush=True)

    def tick(self):
        if time.monotonic() > self.t_end:
            self.summary(); raise SystemExit
        if 'left' not in self.info or 'right' not in self.info or not self.img['left'] or not self.img['right']:
            return
        if self.rect is None:
            self.setup()
        dt, L, Rm = self.best_pair()
        self.dts.append(dt)
        if dt > 0.010:   # OAK stereo is hardware-synced; >10 ms means the pair is not from one exposure
            return
        tl = L.header.stamp.sec + L.header.stamp.nanosec * 1e-9
        r = self.rect
        gl, gr = to_gray(L), to_gray(Rm)
        sl = cv2.resize(gl, (r['ws'], r['hs']), interpolation=cv2.INTER_AREA)
        sr = cv2.resize(gr, (r['ws'], r['hs']), interpolation=cv2.INTER_AREA)
        rl = cv2.remap(sl, *r['m1'], cv2.INTER_LINEAR); rr = cv2.remap(sr, *r['m2'], cv2.INTER_LINEAR)
        disp = self.sgbm.compute(rl, rr).astype(np.float32) / 16.0
        valid = disp > 0.5
        depth = np.where(valid, r['f'] * r['B'] / np.maximum(disp, 1e-3), np.nan)
        bw, bh = int(r['ws'] * args.box / 2), int(r['hs'] * args.box / 2)
        cx, cy = r['ws'] // 2, r['hs'] // 2
        box = depth[cy - bh:cy + bh, cx - bw:cx + bw]
        centre = float(np.nanmedian(box)) if np.isfinite(box).any() else float('nan')
        row = {'t': tl, 'centre_m': centre, 'centre_valid': float(np.isfinite(box).mean())}
        line = f'centre {centre:5.3f} m ({row["centre_valid"]*100:3.0f}% valid)'

        corners, ids, _ = cv2.aruco.detectMarkers(gl, adict, parameters=aparams)
        if ids is not None:
            obj, pix = [], []
            for c, tag in zip(corners, ids.ravel()):
                if 0 <= tag < 36:
                    obj += board_corners(int(tag)); pix += c.reshape(4, 2).tolist()
            if len(obj) >= 24:
                obj = np.array(obj, float); pix = np.array(pix, float)
                ok, rv, tv, inl = cv2.solvePnPRansac(obj, pix, r['K1'], np.zeros(5), reprojectionError=2.5, iterationsCount=100)
                if ok and inl is not None and len(inl) >= 24:
                    k = inl.ravel()
                    rv, tv = cv2.solvePnPRefineLM(obj[k], pix[k], r['K1'], np.zeros(5), rv, tv)
                    rep = np.linalg.norm(cv2.projectPoints(obj[k], rv, tv, r['K1'], np.zeros(5))[0].reshape(-1, 2) - pix[k], axis=1)
                    Pc = (cv2.Rodrigues(rv)[0] @ obj[k].T + tv.reshape(3, 1))   # board corners, left camera frame
                    Pr = r['R1'] @ Pc                                           # rectified left frame
                    uvw = r['P1'] @ np.vstack([Pr, np.ones((1, Pr.shape[1]))])
                    u, v = uvw[0] / uvw[2], uvw[1] / uvw[2]
                    sz = []
                    for ui, vi in zip(u, v):
                        x, y = int(round(ui)), int(round(vi))
                        if 2 <= x < r['ws'] - 2 and 2 <= y < r['hs'] - 2:
                            win = disp[y - 2:y + 3, x - 2:x + 3]; win = win[win > 0.5]
                            sz.append(r['f'] * r['B'] / np.median(win) if len(win) >= 5 else np.nan)
                        else:
                            sz.append(np.nan)
                    sz = np.array(sz); pz = Pr[2]
                    good = np.isfinite(sz)
                    if good.sum() >= 12:
                        err = float(np.median(sz[good] / pz[good] - 1.0) * 100)
                        row.update(board_pnp_m=float(np.median(pz)), board_stereo_m=float(np.median(sz[good])),
                                   board_err_pct=err, board_tags=len(obj) // 4, reproj_px=float(np.median(rep)))
                        line += (f' | board {len(obj)//4:2d} tags, PnP {row["board_pnp_m"]:.3f} m, stereo {row["board_stereo_m"]:.3f} m, '
                                 f'error {err:+.2f}%  (reproj {row["reproj_px"]:.2f} px)')
        print(line, flush=True)
        self.rows.append(row)
        if self.out: self.out.write(json.dumps(row) + '\n'); self.out.flush()

    def summary(self):
        b = [x for x in self.rows if 'board_err_pct' in x]
        print(f'\nreceived left {self.count["left"]} / right {self.count["right"]} images; '
              f'best-pair stamp gap median {np.median(self.dts)*1000 if self.dts else float("nan"):.1f} ms')
        print(f'{len(self.rows)} frames, {len(b)} with the board.')
        if b:
            e = np.array([x['board_err_pct'] for x in b]); d = np.array([x['board_pnp_m'] for x in b])
            print(f'board at {np.median(d):.2f} m (range {d.min():.2f}-{d.max():.2f}): stereo depth error median {np.median(e):+.2f}%, '
                  f'spread (p10..p90) {np.percentile(e,10):+.2f}..{np.percentile(e,90):+.2f}%')


rclpy.init()
node = DepthCheck()
try:
    rclpy.spin(node)
except SystemExit:
    pass
finally:
    node.destroy_node(); rclpy.try_shutdown()
