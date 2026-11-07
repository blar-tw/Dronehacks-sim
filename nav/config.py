"""nav.config - nav/config/nav.yaml -> the parameter dataclasses, and the one place that wires the
modules together. PURE (yaml + dataclasses).

nav.yaml has one section per module (planner, follower, landmarks, aligner, mission).
A key that a dataclass does not have is an error, so a typo never silently keeps a default.
"""
from __future__ import annotations

import dataclasses
from pathlib import Path

SECTIONS = ("planner", "follower", "landmarks", "aligner", "mission")


def default_config_path() -> Path:
    return Path(__file__).resolve().parent / "config" / "nav.yaml"


def _params_classes():
    from . import aligner, follower, landmarks, mission, planner
    return {"planner": planner.PlannerParams, "follower": follower.FollowerParams,
            "landmarks": landmarks.LandmarkParams, "aligner": aligner.AlignerParams,
            "mission": mission.MissionParams}


def fill(cls, values: dict):
    names = {f.name: f for f in dataclasses.fields(cls)}
    bad = sorted(set(values) - set(names))
    if bad:
        raise ValueError(f"{cls.__name__}: unknown keys {bad}")
    obj = cls()
    for k, v in values.items():
        cur = getattr(obj, k)
        if isinstance(cur, bool):
            v = bool(v)
        elif isinstance(cur, float):
            v = float(v)
        elif isinstance(cur, int):
            v = int(v)
        setattr(obj, k, v)
    return obj


def load(path=None, overrides: dict | None = None) -> dict:
    """{section: params dataclass}. overrides: {"mission.chicane": "chicane_gate_b", ...}."""
    import yaml
    path = Path(path) if path else default_config_path()
    doc = yaml.safe_load(path.read_text()) if path.exists() else {}
    doc = doc or {}
    bad = sorted(set(doc) - set(SECTIONS))
    if bad:
        raise ValueError(f"{path}: unknown sections {bad}")
    for key, val in (overrides or {}).items():
        sec, _, name = key.partition(".")
        doc.setdefault(sec, {})[name] = val
    classes = _params_classes()
    return {sec: fill(classes[sec], doc.get(sec) or {}) for sec in SECTIONS}


def build(course, params: dict):
    """The mission with its real planner, follower and aligner."""
    from .aligner import Aligner
    from .follower import Follower
    from .mission import Mission
    from .planner import Planner
    return Mission(course, Planner(course, params["planner"]), Follower(params["follower"]),
                   Aligner(course, params["aligner"]), params["mission"])
