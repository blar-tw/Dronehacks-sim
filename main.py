"""Companion entrypoint: the map-based course mission in nav/ (ARCHITECTURE: VIO -> PX4, map
localization shifts targets only). `python3 main.py` runs the ROS node; extra arguments are ROS
arguments (e.g. `-p height_m:=0.6`). main_route.py is the hard-coded refnav route kept as a fallback."""
import os
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT))

if __name__ == "__main__":
    from nav import node
    args = ["nav", "--ros-args", "--params-file", str(PROJECT / "flight.yaml")]
    deployment = Path(os.environ.get("DRONE_PARAMS_FILE", "/etc/flyx/mission.yaml"))
    if deployment.is_file():
        args += ["--params-file", str(deployment)]
    node.main(args + sys.argv[1:])
