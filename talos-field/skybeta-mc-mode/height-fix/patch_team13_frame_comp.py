#!/usr/bin/env python3
"""Turn off team13_mission's own frame compensation now that the flight interface anchors setpoints.

Why: two corrections for the same offset were on at once.
  * team13 node.py `frame_comp` (added 2026-11-07 21:46, default true) adds the live, low-passed
    (PX4 local - odom) offset to every setpoint it publishes, but only while every axis is < 1.0 m.
  * `anchor_setpoints:=true` (patch_team13_launch.py, applied 23:17) makes flight_interface send
    every setpoint relative to PX4's own position, captured with the odometry origin.
With both on, any offset under 1 m is corrected twice and the drone flies off by that offset in the
other direction. Decision (2026-11-08): keep the anchor, drop frame_comp. The anchor flies heights on
PX4's estimate (rangefinder-fused) instead of the drifting camera height and has no 1 m cut-off.

What this changes in launch/team13_mission.launch.py:
  * new launch argument frame_comp (default false), passed to the team13_mission node.
Turn it back on for a run with `frame_comp:=true anchor_setpoints:=false`, never both true.

Must be applied AFTER patch_team13_launch.py. Exact-string edits: if the file is not the version this
was written against, it stops without writing anything.

usage: python3 -I patch_team13_frame_comp.py <path to team13_mission.launch.py>
"""
import ast
import sys

EDITS = [
    # (old, new) -- each `old` must occur exactly once
    (
        "interface_start_delay_s after launch so that capture happens after VIO and EKF2 settle.\n"
        "\"\"\"",
        "interface_start_delay_s after launch so that capture happens after VIO and EKF2 settle.\n"
        "\n"
        "frame_comp (team13's own live PX4-minus-odom setpoint offset) is off: the anchor already\n"
        "corrects that offset, and with both on it is corrected twice.\n"
        "\"\"\"",
    ),
    (
        "    ('mission', 'true', 'start the mission node (false: flight interface only)'),\n",
        "    ('frame_comp', 'false', 'team13 live (PX4 local - odom) setpoint offset; keep false while '\n"
        "     'anchor_setpoints is true, or the offset is corrected twice'),\n"
        "    ('mission', 'true', 'start the mission node (false: flight interface only)'),\n",
    ),
    (
        "            'depth_info_topic': ParameterValue(L('depth_info_topic'), value_type=str),\n",
        "            'depth_info_topic': ParameterValue(L('depth_info_topic'), value_type=str),\n"
        "            'frame_comp': ParameterValue(L('frame_comp'), value_type=bool),\n",
    ),
]


def patch(text):
    if "anchor_setpoints" not in text:
        raise SystemExit("anchor_setpoints missing: apply patch_team13_launch.py first - not writing")
    if "frame_comp" in text:
        raise SystemExit("already patched (frame_comp present) - nothing to do")
    for old, new in EDITS:
        count = text.count(old)
        if count != 1:
            raise SystemExit(f"expected 1 match, found {count}, for:\n{old}\nfile differs - not writing")
        text = text.replace(old, new)
    ast.parse(text)
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
