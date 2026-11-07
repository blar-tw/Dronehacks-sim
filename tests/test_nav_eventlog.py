"""nav.eventlog: JSON-lines event log."""
import json
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nav.eventlog import EventLog  # noqa: E402


def test_writes_flushed_json_lines(tmp_path):
    d = tmp_path / "logs"
    with EventLog(d, "mission") as log:
        assert re.fullmatch(r"mission_\d{8}-\d{6}\.jsonl", Path(log.path).name)
        log.event(1.5, "phase", phase="TAKEOFF", xy=(1.0, 2.0), n=np.int64(3), a=np.array([0.5]))
        first = json.loads(Path(log.path).read_text().splitlines()[0])   # flushed before close
        assert first == {"t": 1.5, "kind": "phase", "phase": "TAKEOFF", "xy": [1.0, 2.0],
                         "n": 3, "a": [0.5]}
        log.event(2.0, "land")
    assert len(Path(log.path).read_text().splitlines()) == 2
