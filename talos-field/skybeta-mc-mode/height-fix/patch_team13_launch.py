#!/usr/bin/env python3
"""Turn on the flight interface's height/position anchoring in team13_mission's launch file.

Problem (measured on skybeta-05, 2026-11-07): with the drone on the ground, PX4's height estimate
and the camera odometry differed by 1.15-1.33 m. The flight interface passes odometry-frame
setpoints to PX4 unchanged unless anchor_setpoints is true, so every commanded height was off by
that much (0.6 m flight -> ~1.9 m; the landing push stalled ~1 m up).

What this patch changes in launch/team13_mission.launch.py:
  * new launch argument anchor_setpoints (default true), passed to flight_interface;
  * new launch argument interface_start_delay_s (default 60): both nodes start that long after the
    launch. The interface captures PX4's position once, at its first odometry message; delaying it
    lets VIO and EKF2 settle after a boot or stack restart, so the captured pair is consistent.
Nothing else changes. Exact-string edits: if the file is not the version this was written
against, it stops without writing anything.

usage: python3 -I patch_team13_launch.py <path to team13_mission.launch.py>
"""
import ast
import sys

EDITS = [
    # (old, new) -- each `old` must occur exactly once
    (
        "takeoff floor (there is no land command), and PX4's land detector then disarms.\n\"\"\"",
        "takeoff floor (there is no land command), and PX4's land detector then disarms.\n"
        "\n"
        "anchor_setpoints is on: on skybeta-05 the camera odometry and PX4 disagreed on height by\n"
        "1.1-1.3 m (measured 2026-11-07), so the interface re-anchors every setpoint on PX4's own\n"
        "position, captured together with the odometry origin. Both nodes start\n"
        "interface_start_delay_s after launch so that capture happens after VIO and EKF2 settle.\n"
        "\"\"\"",
    ),
    (
        "from launch.actions import DeclareLaunchArgument\n",
        "from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction\n",
    ),
    (
        "    ('odometry_topic', '/visual_slam/tracking/odometry', 'ENU odometry'),\n",
        "    ('odometry_topic', '/visual_slam/tracking/odometry', 'ENU odometry'),\n"
        "    ('anchor_setpoints', 'true', 'send setpoints relative to PX4 own position '\n"
        "     '(fixes the camera/PX4 height offset)'),\n"
        "    ('interface_start_delay_s', '60.0', 'start both nodes this long after launch, '\n"
        "     'so the anchor is captured after VIO and EKF2 settle'),\n",
    ),
    (
        "            'auto_offboard_on_arm': ParameterValue(L('auto_offboard_on_arm'), value_type=bool),\n",
        "            'auto_offboard_on_arm': ParameterValue(L('auto_offboard_on_arm'), value_type=bool),\n"
        "            'anchor_setpoints': ParameterValue(L('anchor_setpoints'), value_type=bool),\n",
    ),
    (
        "    return LaunchDescription(declared + [flight_interface, mission])\n",
        "    def delayed(context, *args, **kwargs):\n"
        "        # Resolved to a float here, so this works whatever TimerAction accepts.\n"
        "        delay = max(0.0, float(L('interface_start_delay_s').perform(context)))\n"
        "        return [TimerAction(period=delay, actions=[flight_interface, mission])]\n"
        "\n"
        "    return LaunchDescription(declared + [OpaqueFunction(function=delayed)])\n",
    ),
]


def patch(text):
    if "anchor_setpoints" in text:
        raise SystemExit("already patched (anchor_setpoints present) - nothing to do")
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
