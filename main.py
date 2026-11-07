"""Course mission entrypoint (ARCHITECTURE.md): map-aligned navigation through every checkpoint,
pad landing, then the return leg. The code is in nav/; settings in nav/config/nav.yaml.

`python3 main.py` runs the ROS node (nav/node.py). Extra ROS arguments pass through, e.g.
python3 main.py -p overrides:='{"mission.chicane": "chicane_gate_b"}'.
The previous refnav takeoff/hover program is kept as main_refnav.py (reference).
"""
import os
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent

if __name__ == "__main__":
    sys.path.insert(0, str(PROJECT))
    from nav import node
    args = ["nav", "--ros-args"]
    deployment = Path(os.environ.get("DRONE_PARAMS_FILE", "/etc/flyx/mission.yaml"))
    if deployment.is_file():
        args += ["--params-file", str(deployment)]     # use_sim_time etc. from the deployment
    user = sys.argv[1:]
    if user and user[0] == "--ros-args":
        user = user[1:]
    print(f"Flight source: {Path(node.__file__).parent} (this run's project)", flush=True)
    node.main(args + user)
