#!/usr/bin/env python3
"""Same height fix for team13_nav's launch (team13_onboard.sh hover / axis / wall / ... modes).

team13_mission.launch.py (the `fly` path) already runs flight_interface with anchor_setpoints:=true,
a 60 s start delay, and team13's own live frame compensation off. team13_nav/launch/team13.launch.py
did not: anchor off, and its mission node's skybeta_io `sky_frame_comp` defaulted to true. That is the
live (PX4 local - odom) correction that, on the 2026-11-07 22:43 flight, followed the camera's height
drift and flew the drone down to floor level.

What this changes in team13_nav/launch/team13.launch.py:
  * new launch argument anchor_setpoints (default true), passed to flight_interface;
  * new launch argument interface_start_delay_s (default 60): flight_interface and every team13
    action (bag, stereo, mission) start that long after launch, so the anchor is captured after VIO
    and EKF2 settle;
  * new launch argument sky_frame_comp (default false), passed to the official_mission node.
Turn the old behaviour back on for a run with `anchor_setpoints:=false sky_frame_comp:=true
interface_start_delay_s:=0`. Never set anchor_setpoints and sky_frame_comp both true.

Refuses to write unless the file is byte-identical to the 2026-11-07 14:44 version (sha256 below).

usage: python3 -I patch_team13_nav_launch.py <path to team13_nav/launch/team13.launch.py>
"""
import ast
import hashlib
import sys

ORIGINAL_SHA256 = "7264bee6b4cfe5de9f35c3e798e7ab8074c1a222239625434c8912c06978537d"

EDITS = [
    (
        "bag/ (mcap; record_images:=true adds the rectified left image + depth).\n\"\"\"",
        "bag/ (mcap; record_images:=true adds the rectified left image + depth).\n"
        "\n"
        "Height fix (2026-11-08, same as team13_mission.launch.py): anchor_setpoints is on, so flight_interface sends\n"
        "every setpoint relative to PX4's own position captured with the odometry origin; skybeta_io's live\n"
        "sky_frame_comp is off (with both on the offset is corrected twice); and everything starts\n"
        "interface_start_delay_s after launch so the anchor is captured after VIO and EKF2 settle. Wait for\n"
        "'envelope origin captured' in the log before arming.\n"
        "\"\"\"",
    ),
    (
        "from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction\n",
        "from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction\n",
    ),
    (
        "    ('odometry_topic', '/visual_slam/tracking/odometry',\n"
        "     'source of the ENU pose the altitude band is measured against'),\n",
        "    ('odometry_topic', '/visual_slam/tracking/odometry',\n"
        "     'source of the ENU pose the altitude band is measured against'),\n"
        "    ('anchor_setpoints', 'true', 'send setpoints relative to PX4 own position (fixes the camera/PX4 '\n"
        "     'height offset); never true together with sky_frame_comp'),\n"
        "    ('interface_start_delay_s', '60.0', 'start everything this long after launch, so the anchor is '\n"
        "     'captured after VIO and EKF2 settle'),\n",
    ),
    (
        "    ('mission', 'true', 'start the team13 mission node (io:=skybeta)'),\n",
        "    ('mission', 'true', 'start the team13 mission node (io:=skybeta)'),\n"
        "    ('sky_frame_comp', 'false', 'skybeta_io live (PX4 local - odom) setpoint offset; keep false while '\n"
        "     'anchor_setpoints is true, or the offset is corrected twice'),\n",
    ),
    (
        "            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),\n",
        "            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),\n"
        "            'anchor_setpoints': LaunchConfiguration('anchor_setpoints'),\n",
    ),
    (
        "                            parameters=[{'io': 'skybeta', 'prior': a['prior'],\n",
        "                            parameters=[{'io': 'skybeta', 'prior': a['prior'],\n"
        "                                         'sky_frame_comp': a['sky_frame_comp'].lower() == 'true',\n",
    ),
    (
        "    return LaunchDescription(declared + [flight_interface, OpaqueFunction(function=_team13)])\n",
        "    def delayed(context, *args, **kwargs):\n"
        "        # Resolved to a float here, so this works whatever TimerAction accepts.\n"
        "        delay = max(0.0, float(LaunchConfiguration('interface_start_delay_s').perform(context)))\n"
        "        note = LogInfo(msg=f'team13: flight_interface starts in {delay:.0f} s - wait for \"envelope origin '\n"
        "                           f'captured\" before arming')\n"
        "        return [note, TimerAction(period=delay, actions=[flight_interface, OpaqueFunction(function=_team13)])]\n"
        "\n"
        "    return LaunchDescription(declared + [OpaqueFunction(function=delayed)])\n",
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
