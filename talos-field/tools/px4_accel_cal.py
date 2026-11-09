#!/usr/bin/env python3
"""Guided six-side PX4 accelerometer calibration with a pose-accuracy guard (skybeta-05).

Runs on the Mac with the Pixhawk 6C USB-C in /dev/cu.usbmodem01 (pymavlink from ~/mavproxy-env).
Only one program may hold the port; stop any forwarder first.

Modes
  backup   read every parameter (read-only) and save JSON
  guide    1) REHEARSAL (read-only): live tilt readout per pose, records "before" gravity norms
           2) CALIBRATION: PX4 six-side accel cal; cancels it if any side is captured tilted
              more than --cancel-deg (PX4 itself accepts ~30 deg, which ruined attempt003)
           3) VERIFICATION: six poses again with the new fit; |g| must be within --tol-pct
  verify   step 3 alone
  restore  write the 12 CAL_ACC{0,1} offset/scale fields back from a backup, exact-echo checked

PX4 body frame (FRD) poses, as PX4 names them:
  down = level right side up, up = upside down, front = nose down, back = nose up (tail down),
  left = lying on its left side, right = lying on its right side.
Never arms, never touches motors. Props must be off.
"""
import argparse
import json
import math
import os
import re
import select
import statistics
import struct
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

from pymavlink import mavutil

G = 9.80665
EXPECT_SYSID = 5
EXPECT_UID = 3473490359910740788
POSES = {
    "down": (0.0, 0.0, -1.0),
    "up": (0.0, 0.0, 1.0),
    "front": (-1.0, 0.0, 0.0),
    "back": (1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
}
HOWTO = {
    "down": "level, right side up, on its landing gear",
    "up": "upside down",
    "front": "nose straight DOWN (front/camera end on the floor)",
    "back": "nose straight UP (tail end on the floor)",
    "left": "lying on its LEFT side",
    "right": "lying on its RIGHT side",
}
ORDER = ["down", "up", "front", "back", "left", "right"]
CAL_FIELDS = [f"CAL_ACC{i}_{f}" for i in (0, 1) for f in ("XOFF", "YOFF", "ZOFF", "XSCALE", "YSCALE", "ZSCALE")]
IMUS = ("SCALED_IMU", "SCALED_IMU2")
STILL_SD = 0.15  # m/s^2 per-axis std over 1 s counts as "still" (bench noise at rest is ~0.03-0.08)

GREEN, YELLOW, RED, BOLD, RESET = "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[1m", "\x1b[0m"


# ---------------------------------------------------------------- vector helpers
def vnorm(v):
    return math.sqrt(sum(c * c for c in v))


def tilt_deg(v, axis):
    n = vnorm(v)
    if n < 1e-6:
        return float("nan")
    d = sum(a * b for a, b in zip(v, axis)) / n
    return math.degrees(math.acos(max(-1.0, min(1.0, d))))


def nearest_pose(v):
    return min(POSES, key=lambda p: tilt_deg(v, POSES[p]))


def angle_between(a, b):
    na, nb = vnorm(a), vnorm(b)
    d = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, d))))


def f32hex(x):
    return struct.pack("<f", x).hex()


# ---------------------------------------------------------------- MAVLink link
class Link:
    def __init__(self, port, log_path=None):
        self.m = mavutil.mavlink_connection(port, baud=115200, source_system=255, source_component=190)
        self.imu = {n: deque(maxlen=3000) for n in IMUS}
        self.texts = deque()
        self.param_msgs = deque()
        self.params = {}
        self.acks = deque()
        self.hb = None
        self.t_hb_rx = 0.0
        self.t_hb_tx = 0.0
        self.sysid = None
        self.version = None
        self.log = open(log_path, "a") if log_path else None

    def _tx_heartbeat(self):
        if time.time() - self.t_hb_tx > 0.5:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.t_hb_tx = time.time()

    def _handle(self, msg):
        t = msg.get_type()
        if t == "BAD_DATA":
            return
        if self.sysid is not None and msg.get_srcSystem() != self.sysid:
            return
        if msg.get_srcComponent() != 1:
            return
        now = time.time()
        if t == "HEARTBEAT":
            if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self.hb, self.t_hb_rx = msg, now
        elif t in self.imu:
            if msg.xacc or msg.yacc or msg.zacc:
                self.imu[t].append((now, (msg.xacc * G / 1000.0, msg.yacc * G / 1000.0, msg.zacc * G / 1000.0)))
        elif t == "STATUSTEXT":
            self.texts.append((now, msg.text))
            if self.log:
                self.log.write(f"{time.strftime('%H:%M:%S')} STATUSTEXT {msg.severity} {msg.text}\n")
                self.log.flush()
        elif t == "PARAM_VALUE":
            self.param_msgs.append(msg)
            self.params[msg.param_id] = msg
        elif t == "COMMAND_ACK":
            self.acks.append(msg)
        elif t == "AUTOPILOT_VERSION":
            self.version = msg

    def pump(self, dur=0.0):
        end = time.time() + dur
        n = 0
        while True:
            self._tx_heartbeat()
            remaining = end - time.time()
            if remaining > 0:
                msg = self.m.recv_match(blocking=True, timeout=min(0.05, remaining))
            else:
                msg = self.m.recv_match(blocking=False)
            if msg is None:
                if time.time() >= end:
                    return
                continue
            self._handle(msg)
            n += 1
            if remaining <= 0 and n > 3000:
                return

    def connect(self):
        t0 = time.time()
        while time.time() - t0 < 15 and self.hb is None:
            self.pump(0.2)
        if self.hb is None:
            raise SystemExit("No PX4 heartbeat. Is the USB-C cable in and nothing else holding the port?")
        # _handle accepted any sysid until now; lock to the FC that answered
        self.sysid = self._last_hb_sysid()
        if self.sysid != EXPECT_SYSID:
            raise SystemExit(f"Refusing: sysid {self.sysid} is not skybeta-05 (expected {EXPECT_SYSID}).")
        for _ in range(3):
            self.cmd(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 148)
            self.pump(1.0)
            if self.version:
                break
        if not self.version or self.version.uid != EXPECT_UID:
            raise SystemExit(f"Refusing: UID {getattr(self.version, 'uid', None)} is not skybeta-05's Pixhawk.")
        for msgid in (26, 116):  # SCALED_IMU, SCALED_IMU2 at 50 Hz (not persistent)
            self.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msgid, 20000)
        self.pump(1.0)

    def _last_hb_sysid(self):
        return self.hb.get_srcSystem()

    def armed(self):
        return bool(self.hb and self.hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def cmd(self, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        self.m.mav.command_long_send(self.sysid, 1, command, 0, p1, p2, p3, p4, p5, p6, p7)

    def window(self, name, t0, t1=None):
        t1 = t1 or time.time()
        return [v for t, v in self.imu[name] if t0 <= t <= t1]

    def stats(self, name, seconds=1.0, t_end=None):
        t_end = t_end or time.time()
        vs = self.window(name, t_end - seconds, t_end)
        if len(vs) < 5:
            return None
        mean = tuple(statistics.fmean(c) for c in zip(*vs))
        sd = tuple(statistics.pstdev(c) for c in zip(*vs))
        return {"mean": mean, "sd": sd, "n": len(vs)}

    def read_param(self, name, tries=3):
        for _ in range(tries):
            self.params.pop(name, None)
            self.m.mav.param_request_read_send(self.sysid, 1, name.encode(), -1)
            t0 = time.time()
            while time.time() - t0 < 1.5:
                self.pump(0.1)
                if name in self.params:
                    return self.params[name]
        raise SystemExit(f"No reply reading {name}")

    def set_param_exact(self, name, hexbytes, ptype):
        want = bytes.fromhex(hexbytes)
        value = struct.unpack("<f", want)[0]
        for _ in range(3):
            self.params.pop(name, None)
            self.m.mav.param_set_send(self.sysid, 1, name.encode(), value, ptype)
            t0 = time.time()
            while time.time() - t0 < 2.0:
                self.pump(0.1)
                if name in self.params:
                    got = struct.pack("<f", self.params[name].param_value)
                    if got == want:
                        return True
                    break
        return False


# ---------------------------------------------------------------- terminal helpers
def say(line=""):
    sys.stdout.write("\r\x1b[2K" + line + "\n")
    sys.stdout.flush()


VOICE = {"on": True}


def speak(text):
    """Spoken prompt (macOS `say`) so the user can watch the drone, not the screen."""
    if VOICE["on"]:
        try:
            subprocess.Popen(["say", "-r", "210", text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            pass


def status(line):
    sys.stdout.write("\r\x1b[2K" + line)
    sys.stdout.flush()


def stdin_line():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower()
    return None


def live_state(link):
    a = link.stats("SCALED_IMU", 1.0)
    b = link.stats("SCALED_IMU2", 1.0)
    if not a or not b:
        return None
    pose = nearest_pose(a["mean"])
    ta, tb = tilt_deg(a["mean"], POSES[pose]), tilt_deg(b["mean"], POSES[pose])
    still = max(a["sd"]) < STILL_SD and max(b["sd"]) < STILL_SD
    return {"pose": pose, "tilt": (ta, tb), "still": still,
            "err_pct": ((vnorm(a["mean"]) / G - 1) * 100, (vnorm(b["mean"]) / G - 1) * 100)}


def fmt_live(s, ok_deg, show_norm=False):
    if not s:
        return "waiting for IMU data..."
    t = max(s["tilt"])
    col = GREEN if t <= ok_deg else (YELLOW if t <= 2 * ok_deg else RED)
    verdict = "OK" if t <= ok_deg else "ADJUST"
    still = "still" if s["still"] else "moving"
    extra = f"  |g| err IMU0 {s['err_pct'][0]:+.2f}%  IMU1 {s['err_pct'][1]:+.2f}%" if show_norm else ""
    return (f"pose {BOLD}{s['pose']:>5}{RESET}  tilt IMU0 {s['tilt'][0]:4.1f} deg  IMU1 {s['tilt'][1]:4.1f} deg  "
            f"{col}{BOLD}{verdict}{RESET}  ({still}){extra}")


def record_pose(link, seconds):
    out = {}
    for n in IMUS:
        st = link.stats(n, seconds)
        mean = st["mean"]
        out[n] = {"mean": mean, "sd": st["sd"], "n": st["n"], "norm": vnorm(mean),
                  "err_pct": (vnorm(mean) / G - 1) * 100}
    return out


def save(path, obj):
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(json.dumps(obj, indent=2))
    tmp.replace(path)


# ---------------------------------------------------------------- phases
def pose_session(link, title, ok_deg, hold_s, rec_key, rec, path, show_norm):
    """Collect one still recording per pose. Enter finishes early, q quits."""
    got = {}
    say(f"{BOLD}{title}{RESET}")
    for p in ORDER:
        say(f"   {p:>5}: {HOWTO[p]}")
    say("   Hold each pose still ~4 s. A pose is recorded once it has been still for "
        f"{hold_s:.0f} s. Press Enter when done (q + Enter to quit).")
    cand, t_cand = None, 0.0
    t_draw = 0.0
    while len(got) < 6:
        link.pump(0.05)
        line = stdin_line()
        if line is not None:
            if line == "q":
                raise KeyboardInterrupt
            break
        s = live_state(link)
        now = time.time()
        if s and s["still"]:
            if cand != s["pose"]:
                cand, t_cand = s["pose"], now
            elif now - t_cand >= hold_s and cand not in got:
                r = record_pose(link, hold_s - 0.5)
                tilts = [tilt_deg(r[n]["mean"], POSES[cand]) for n in IMUS]
                r["tilt_deg"] = tilts
                got[cand] = r
                col = GREEN if max(tilts) <= ok_deg else YELLOW
                say(f"   recorded {BOLD}{cand:>5}{RESET}: tilt {col}{max(tilts):.1f} deg{RESET}, "
                    f"|g| err IMU0 {r['SCALED_IMU']['err_pct']:+.2f}%  IMU1 {r['SCALED_IMU2']['err_pct']:+.2f}%   "
                    f"[{len(got)}/6; left: {' '.join(p for p in ORDER if p not in got)}]")
                rec[rec_key] = got
                save(path, rec)
        else:
            cand = None
        if now - t_draw > 0.2:
            status("   " + fmt_live(s, ok_deg, show_norm))
            t_draw = now
    say()
    rec[rec_key] = got
    save(path, rec)
    return got


def calibrate(link, args, rec, path):
    if link.armed():
        raise SystemExit("Refusing: vehicle is armed.")
    say(f"{BOLD}CALIBRATION{RESET}  PX4 is now collecting. Put the drone in any pending pose using your supports,")
    say("   get the readout green, then let it sit still. PX4 measures for ~2 s after it detects the pose.")
    say(f"   This guide cancels the whole run (nothing saved) if a side is captured more than {args.cancel_deg:.1f} deg off.")
    say("   Type c + Enter to cancel yourself.")
    say(f"   {YELLOW}{BOLD}LIFT IT OFF as soon as you hear 'done': resting still on a finished side 5 times "
        f"(~15 s) makes PX4 abort. Held in your hands and moving is safe for ~90 s.{RESET}")
    say(f"{BOLD}Put the drone in your FIRST pose (any of the six), then press Enter to start PX4 calibration{RESET} "
        "(q + Enter to quit).")
    t_draw = 0.0
    while True:
        link.pump(0.05)
        line = stdin_line()
        if line == "q":
            raise KeyboardInterrupt
        if line is not None:
            break
        if time.time() - t_draw > 0.2:
            status("   " + fmt_live(live_state(link), args.ok_deg))
            t_draw = time.time()
    say()
    if link.armed():
        raise SystemExit("Refusing: vehicle is armed.")
    link.texts.clear()
    link.acks.clear()
    link.cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0)
    rec["attempt_count"] = rec.get("attempt_count", 0) + 1
    cal = {"attempt": rec["attempt_count"], "sides": {}, "statustext": [], "started": time.time()}
    retries = 0
    announced = set()
    cancel_sent = cleared_old = clearing = False
    rec["calibration"] = cal
    rec.setdefault("attempts", []).append(cal)
    speak("Calibration started")
    measuring, t_meas = None, 0.0
    deadline = time.time() + 900
    t_draw = 0.0

    def finalize(side, px4_vec=None):
        nonlocal measuring
        if side in cal["sides"]:
            return None
        t1 = time.time()
        t0 = t_meas if measuring == side else t1 - 1.5
        entry = {"px4_reported": px4_vec}
        worst = 0.0
        for n in IMUS:
            vs = link.window(n, t0, t1)
            if len(vs) < 10:
                vs = link.window(n, t1 - 1.5, t1)
            mean = tuple(statistics.fmean(c) for c in zip(*vs))
            tl = tilt_deg(mean, POSES[side])
            entry[n] = {"mean": mean, "n": len(vs), "tilt_deg": tl}
            worst = max(worst, tl)
        if px4_vec:
            tl = tilt_deg(px4_vec, POSES[side])
            entry["px4_tilt_deg"] = tl
            worst = max(worst, tl)
        entry["worst_tilt_deg"] = worst
        cal["sides"][side] = entry
        measuring = None
        save(path, rec)
        col = GREEN if worst <= args.ok_deg else (YELLOW if worst <= args.cancel_deg else RED)
        say(f"   captured {BOLD}{side:>5}{RESET}: worst tilt {col}{worst:.1f} deg{RESET}   "
            f"[{len(cal['sides'])}/6]")
        return worst

    while time.time() < deadline:
        link.pump(0.05)
        newest = link.imu["SCALED_IMU"][-1][0] if link.imu["SCALED_IMU"] else cal["started"]
        if time.time() - newest > 6.0:
            say(f"   {RED}{BOLD}USB LINK LOST: no data from the Pixhawk for 6 s. Check the cable. On USB power the "
                f"Pixhawk restarts and the unfinished calibration is dropped (nothing saved). Relaunch the guide.{RESET}")
            speak("USB link lost. Check the cable.")
            cal["result"] = "link_lost"
            rec["link_lost"] = True
            save(path, rec)
            return "link_lost"
        while link.acks:
            a = link.acks.popleft()
            if a.command != mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION:
                continue
            cal.setdefault("acks", []).append(a.result)
            # A cancel is answered twice: ACCEPTED by the calibration thread and TEMPORARILY_REJECTED by
            # commander (its worker is busy). Neither means the start was refused.
            if a.result in (0, 5) or cancel_sent or clearing:
                continue
            if a.result == 1 and not cleared_old and not cal["sides"]:
                say(f"   {YELLOW}PX4 is still busy with an earlier, unfinished calibration: cancelling it first, "
                    f"then starting again from side 1.{RESET}")
                cleared_old = clearing = True
                link.cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 0, 0)
                continue
            say(f"   {RED}PX4 refused the calibration command (MAV_RESULT {a.result}).{RESET}")
            cal["result"] = f"refused_{a.result}"
            save(path, rec)
            return "refused"
        while link.texts:
            _, text = link.texts.popleft()
            cal["statustext"].append([time.time(), text])
            if "[cal]" in text or "cal" in text.lower() or "accel" in text.lower():
                say(f"   PX4: {text}")
            if clearing and ("calibration cancel" in text or "calibration failed" in text):
                clearing = False
                link.pump(1.0)
                link.texts.clear()
                link.acks.clear()
                link.cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 1, 0, 0)
                say(f"   {GREEN}old calibration cleared; new calibration started from side 1{RESET}")
                break
            m = re.search(r"measuring (\w+) side", text)
            if m:
                measuring, t_meas = m.group(1), time.time()
                retries = 0
                speak(f"Measuring {m.group(1)}. Hold still.")
            m = re.search(r"(\w+) side already completed", text)
            if m:
                retries += 1
                speak(f"Move it. {m.group(1)} is done.")
                pending = [p for p in ORDER if p not in cal["sides"]]
                say(f"   {RED}{BOLD}MOVE IT: {m.group(1)} is already done ({retries}/5 before PX4 aborts). "
                    f"Pending: {' '.join(pending)}{RESET}")
            m = re.search(r"(\w+) side done", text)
            if m and m.group(1) not in announced:
                announced.add(m.group(1))
                pending = [p for p in ORDER if p not in cal["sides"]]
                if pending:
                    say(f"   {YELLOW}{BOLD}>>> LIFT IT and move NOW to one of: {' '.join(pending)}{RESET}")
                    speak(f"{m.group(1)} done. Lift it now. {len(pending)} to go.")
            m = re.search(r"(\w+) side result: \[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)", text)
            side = None
            px4_vec = None
            if m:
                side, px4_vec = m.group(1), tuple(float(x) for x in m.group(2, 3, 4))
            else:
                m = re.search(r"(\w+) side done", text)
                if m:
                    side = m.group(1)
            if side in POSES:
                worst = finalize(side, px4_vec)
                if worst is not None and worst > args.cancel_deg:
                    if len(cal["sides"]) < 6:
                        cancel_sent = True
                        link.cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 0, 0)
                        say(f"   {RED}{BOLD}{side} side was {worst:.1f} deg off: cancelling, nothing will be saved.{RESET}")
                        speak(f"Cancelled. {side} was tilted {worst:.0f} degrees.")
                        cal["guard_cancel"] = {"side": side, "tilt_deg": worst}
                    else:
                        say(f"   {RED}{BOLD}last side ({side}) was {worst:.1f} deg off and PX4 saves immediately: "
                            f"this fit must be restored from the backup.{RESET}")
                        cal["bad_last_side"] = {"side": side, "tilt_deg": worst}
            if "calibration done" in text:
                cal["result"] = "done"
                save(path, rec)
                speak("Calibration done. Starting verification.")
                return "done"
            if "calibration failed" in text:
                cal["result"] = "failed: " + text
                save(path, rec)
                speak("Calibration failed. Nothing saved.")
                return "failed"
            if "calibration cancel" in text:
                cal["result"] = "cancelled"
                save(path, rec)
                return "cancelled"
        line = stdin_line()
        if line == "c":
            cancel_sent = True
            link.cmd(mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0, 0, 0, 0, 0, 0, 0)
            say("   cancel sent")
            cal["user_cancel"] = True
        if time.time() - t_draw > 0.2:
            status("   " + fmt_live(live_state(link), args.ok_deg))
            t_draw = time.time()
    cal["result"] = "timeout"
    save(path, rec)
    return "timeout"


def evaluate(got, tol):
    res = {"tol_pct": tol, "poses": sorted(got), "complete": len(got) == 6}
    worst = 0.0
    for n in IMUS:
        errs = {p: got[p][n]["err_pct"] for p in got}
        res[n] = {"err_pct": errs,
                  "max_abs_err_pct": max(abs(e) for e in errs.values()) if errs else None,
                  "spread_pct": (max(errs.values()) - min(errs.values())) if errs else None}
        worst = max(worst, res[n]["max_abs_err_pct"] or 0.0)
        pairs = {}
        for a, b in (("front", "back"), ("left", "right"), ("up", "down")):
            if a in got and b in got:
                pairs[f"{a}/{b}"] = angle_between(got[a][n]["mean"], got[b][n]["mean"])
        res[n]["opposing_angle_deg"] = pairs
    res["worst_abs_err_pct"] = worst
    res["pass"] = res["complete"] and worst <= tol
    return res


def print_eval(title, ev):
    say(f"{BOLD}{title}{RESET}  (limit {ev['tol_pct']:.2f}% of g, poses {len(ev['poses'])}/6)")
    for n in IMUS:
        e = ev[n]["err_pct"]
        row = "  ".join(f"{p} {e[p]:+.2f}%" for p in ORDER if p in e)
        say(f"   {n:<11} {row}")
        if ev[n]["opposing_angle_deg"]:
            say("   " + " " * 11 + " opposing poses: " +
                "  ".join(f"{k} {v:.1f} deg" for k, v in ev[n]["opposing_angle_deg"].items()))
    verdict = f"{GREEN}{BOLD}PASS{RESET}" if ev["pass"] else f"{RED}{BOLD}NOT ACCEPTED{RESET}"
    say(f"   worst |g| error {ev['worst_abs_err_pct']:.2f}%  ->  {verdict}")


def read_cal_params(link):
    out = {}
    for n in [f"CAL_ACC{i}_ID" for i in (0, 1)] + CAL_FIELDS:
        p = link.read_param(n)
        out[n] = {"value": p.param_value, "hex": f32hex(p.param_value), "type": p.param_type}
    return out


def do_backup(link, path):
    link.param_msgs.clear()
    link.m.mav.param_request_list_send(link.sysid, 1)
    got, count, t_last = {}, None, time.time()
    while True:
        link.pump(0.2)
        while link.param_msgs:
            m = link.param_msgs.popleft()
            if m.param_index < m.param_count:
                got[m.param_index], count, t_last = m, m.param_count, time.time()
        if count and len(got) >= count:
            break
        if time.time() - t_last > 4:
            break
    for _ in range(5):
        missing = [i for i in range(count or 0) if i not in got]
        if not missing:
            break
        for i in missing:
            link.m.mav.param_request_read_send(link.sysid, 1, b"", i)
            link.pump(0.02)
        t_end = time.time() + 3
        while time.time() < t_end:
            link.pump(0.2)
            while link.param_msgs:
                m = link.param_msgs.popleft()
                if m.param_index < m.param_count:
                    got[m.param_index] = m
    params = [{"id": m.param_id, "index": i, "type": m.param_type, "value_bytes_le_hex": f32hex(m.param_value)}
              for i, m in sorted(got.items())]
    obj = {"host_unix_s": time.time(), "sysid": link.sysid, "uid": link.version.uid, "expected_count": count,
           "complete": count is not None and len(params) == count, "parameters": params}
    save(path, obj)
    return obj


def do_restore(link, backup_path):
    b = json.loads(Path(backup_path).read_text())
    by = {p["id"]: p for p in b["parameters"]}
    for i in (0, 1):
        live = link.read_param(f"CAL_ACC{i}_ID")
        if f32hex(live.param_value) != by[f"CAL_ACC{i}_ID"]["value_bytes_le_hex"]:
            raise SystemExit(f"Refusing: CAL_ACC{i}_ID differs from the backup (different sensor).")
    changed, ok = [], True
    for n in CAL_FIELDS:
        want = by[n]["value_bytes_le_hex"]
        cur = f32hex(link.read_param(n).param_value)
        if cur == want:
            print(f"  {n}: already {want}")
            continue
        good = link.set_param_exact(n, want, by[n]["type"])
        changed.append(n)
        ok &= good
        print(f"  {n}: {cur} -> {want}  {'exact echo' if good else 'ECHO MISMATCH'}")
    final = {n: f32hex(link.read_param(n).param_value) for n in CAL_FIELDS}
    match = all(final[n] == by[n]["value_bytes_le_hex"] for n in CAL_FIELDS)
    print(f"restored {len(changed)} field(s); all 12 now match backup: {match}")
    return ok and match


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=("backup", "guide", "verify", "restore"))
    ap.add_argument("--port", default="/dev/cu.usbmodem01")
    ap.add_argument("--out", type=Path, default=Path("/Users/ds/Desktop/TALOS/pixhawk-accel-cal-2026-11-07"))
    ap.add_argument("--backup", type=Path, help="backup JSON for restore")
    ap.add_argument("--ok-deg", type=float, default=2.0)
    ap.add_argument("--cancel-deg", type=float, default=3.0)
    ap.add_argument("--tol-pct", type=float, default=0.5)
    ap.add_argument("--hold-s", type=float, default=3.0)
    ap.add_argument("--skip-rehearsal", action="store_true")
    ap.add_argument("--no-voice", action="store_true")
    args = ap.parse_args()
    VOICE["on"] = not args.no_voice

    args.out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%H%M%S")
    link = Link(args.port, args.out / f"{args.mode}-{stamp}-statustext.log")
    link.connect()
    say(f"connected: sysid {link.sysid}, UID {link.version.uid}, armed={link.armed()}")

    if args.mode == "backup":
        obj = do_backup(link, args.out / f"parameters-{stamp}.json")
        print(f"backup: {len(obj['parameters'])}/{obj['expected_count']} params, complete={obj['complete']} "
              f"-> {args.out / f'parameters-{stamp}.json'}")
        return 0 if obj["complete"] else 1
    if args.mode == "restore":
        if not args.backup:
            raise SystemExit("--backup is required")
        return 0 if do_restore(link, args.backup) else 1

    path = args.out / f"{args.mode}-{stamp}.json"
    rec = {"mode": args.mode, "started": time.time(), "uid": link.version.uid,
           "limits": {"ok_deg": args.ok_deg, "cancel_deg": args.cancel_deg, "tol_pct": args.tol_pct}}
    rec["params_before"] = read_cal_params(link)
    save(path, rec)
    try:
        if args.mode == "verify":
            got = pose_session(link, "VERIFICATION", args.ok_deg, args.hold_s, "verify", rec, path, True)
            rec["verify_eval"] = evaluate(got, args.tol_pct)
            print_eval("VERIFICATION RESULT", rec["verify_eval"])
            save(path, rec)
            return 0 if rec["verify_eval"]["pass"] else 3

        if not args.skip_rehearsal:
            got = pose_session(link, "REHEARSAL (read-only, nothing is written)", args.ok_deg, args.hold_s,
                               "rehearsal", rec, path, True)
            if got:
                rec["rehearsal_eval"] = evaluate(got, args.tol_pct)
                print_eval("BEFORE (current calibration)", rec["rehearsal_eval"])
                save(path, rec)
            say(f"{BOLD}Press Enter to START the PX4 accelerometer calibration{RESET} (q + Enter to quit).")
            while True:
                link.pump(0.1)
                line = stdin_line()
                if line == "q":
                    raise KeyboardInterrupt
                if line is not None:
                    break
        while True:
            result = calibrate(link, args, rec, path)
            say(f"calibration result: {BOLD}{result}{RESET}")
            if result == "done":
                break
            if result == "link_lost":
                return 4
            link.pump(2.0)
            chk = read_cal_params(link)
            same = all(chk[n]["hex"] == rec["params_before"][n]["hex"] for n in CAL_FIELDS)
            rec["calibration"]["params_unchanged_after"] = same
            save(path, rec)
            say(f"   all 12 calibration values unchanged: {same}")
            if not same:
                say(f"{RED}Values changed without 'calibration done': stopping so they can be checked.{RESET}")
                rec["params_after"] = chk
                save(path, rec)
                return 2
            say("   Retrying in this same tab.")
        link.pump(3.0)
        rec["params_after"] = read_cal_params(link)
        save(path, rec)
        for n in CAL_FIELDS:
            b, a = rec["params_before"][n]["value"], rec["params_after"][n]["value"]
            say(f"   {n:<15} {b:+.6f} -> {a:+.6f}")
        if rec["calibration"].get("bad_last_side"):
            say(f"{RED}Last side was off-axis: restore with  restore --backup <file>{RESET}")
        got = pose_session(link, "VERIFICATION (new calibration active)", args.ok_deg, args.hold_s,
                           "verify", rec, path, True)
        rec["verify_eval"] = evaluate(got, args.tol_pct)
        print_eval("AFTER (new calibration)", rec["verify_eval"])
        save(path, rec)
        say(f"saved {path}")
        return 0 if rec["verify_eval"]["pass"] else 3
    except KeyboardInterrupt:
        say("stopped by user")
        rec["stopped_by_user"] = True
        save(path, rec)
        return 130


if __name__ == "__main__":
    sys.exit(main())
