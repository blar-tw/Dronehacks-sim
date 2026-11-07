"""Per-node event log, one JSON object per line: {"t": t, "kind": kind, ...fields}.

The ONLY nav module that writes files (the logger component). For development analysis only
(rule 6.4: flight code never reads logs or bags back).
"""
import json
import os
import time

import numpy as np


def _np(v):
    if isinstance(v, np.generic):
        return v.item()
    if isinstance(v, np.ndarray):
        return v.tolist()
    raise TypeError(f"not JSON serializable: {type(v).__name__}")


class EventLog:
    def __init__(self, directory, node_name):
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(directory, f"{node_name}_{time.strftime('%Y%m%d-%H%M%S')}.jsonl")
        self._f = open(self.path, "a", encoding="utf-8")

    def event(self, t, kind, **fields) -> None:
        self._f.write(json.dumps({"t": t, "kind": kind, **fields}, default=_np) + "\n")
        self._f.flush()

    def close(self) -> None:
        self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
