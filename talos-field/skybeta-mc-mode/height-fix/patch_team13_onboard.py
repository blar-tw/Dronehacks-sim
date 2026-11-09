#!/usr/bin/env python3
"""team13_onboard.sh: keep axis / wall on their own height correction once team13_nav anchors setpoints.

tools/axis_test.py and tools/wall_hop.py add their own live, low-passed (PX4 local - VIO odom) offset to
every setpoint (when every axis is under 1 m), and have no switch to turn that off. After
patch_team13_nav_launch.py, team13_nav/launch/team13.launch.py defaults to anchor_setpoints:=true, which
corrects the same offset inside flight_interface - so for axis / wall the offset would be corrected twice.

What this changes in team13_onboard.sh (nothing else):
  * axis and wall launch team13.launch.py with anchor_setpoints:=false interface_start_delay_s:=0, i.e.
    exactly as before (their validated behaviour, no 60 s wait);
  * the `fly` and `use blar` messages tell the pilot to wait for 'envelope origin captured' (~60 s)
    before arming, because those paths now start flight_interface 60 s after launch;
  * a header comment explaining the split.
Refuses to write unless the file is byte-identical to the 2026-11-07 23:15 version (sha256 below).

usage: python3 -I patch_team13_onboard.py <path to team13_onboard.sh>
"""
import hashlib
import sys

ORIGINAL_SHA256 = "a44a1fdbfcb9b2e49332891dde1a011c3bd3999714efe09fb2f9989e45c3732b"
WAIT = "wait ~60 s for 'envelope origin captured' in window algo before arming (anchored setpoints)"

EDITS = [
    (
        "# organizers' example. drone.env is never edited. Env: PRIOR, SPEED (default 0.4), IMAGES=1.\n",
        "# organizers' example. drone.env is never edited. Env: PRIOR, SPEED (default 0.4), IMAGES=1.\n"
        "# Height (2026-11-08): fly / use blar run flight_interface with anchor_setpoints:=true and a 60 s start delay\n"
        "# (arm only after 'envelope origin captured'). axis / wall keep their tools' own live PX4-minus-VIO offset, so\n"
        "# they pass anchor_setpoints:=false interface_start_delay_s:=0 - never both corrections at once.\n",
    ),
    (
        "name:=axis_$(date +%H%M%S) mission:=false stereo:=false\"\n",
        "name:=axis_$(date +%H%M%S) mission:=false stereo:=false anchor_setpoints:=false interface_start_delay_s:=0\"\n",
    ),
    (
        "name:=wall_$(date +%H%M%S) mission:=false stereo:=false\"\n",
        "name:=wall_$(date +%H%M%S) mission:=false stereo:=false anchor_setpoints:=false interface_start_delay_s:=0\"\n",
    ),
    (
        "Pilot: ALTITUDE + throttle low -> ARM -> POSITION within seconds.\"\n",
        "Pilot: " + WAIT + ", then ALTITUDE + throttle low -> ARM -> POSITION.\"\n",
    ),
    (
        "(bag, launch.log, preflight.txt, code.tar.gz)\" ;;\n",
        "(bag, launch.log, preflight.txt, code.tar.gz)\"\n"
        "    echo \"  Pilot: " + WAIT + ".\" ;;\n",
    ),
]


def patch(text):
    if "anchor_setpoints" in text:
        raise SystemExit("already patched (anchor_setpoints present) - nothing to do")
    sha = hashlib.sha256(text.encode()).hexdigest()
    if sha != ORIGINAL_SHA256:
        raise SystemExit(f"file differs from the version this was written for (sha256 {sha[:16]}...) - not writing")
    for old, new in EDITS:
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"expected 1 match, found {count}, for:\n{old}\nnot writing")
        text = text.replace(old, new)
    return text


def main():
    path = sys.argv[1]
    with open(path) as f:
        original = f.read()
    patched = patch(original)
    with open(path, "w") as f:
        f.write(patched)
    print(f"patched {path}")


if __name__ == "__main__":
    main()
