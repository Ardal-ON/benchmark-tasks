"""Checks scoring outcomes independently of the simulator."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np

spec = importlib.util.spec_from_file_location("sorting_task", Path(__file__).with_name("task.py"))
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)


def block(x, y, bottom, resting=True):
    return SimpleNamespace(pos=np.array([x, y, bottom + 0.02]), low=np.array([x - 0.02, y - 0.02, bottom]),
                           high=np.array([x + 0.02, y + 0.02, bottom + 0.04]), resting=lambda: resting)


def test_empty_and_placed():
    states = {name: block(0.34, y, task.BELT_TOP) for name, y in task.STARTS}
    assert task.score(states) == 0
    for name, _ in task.STARTS:
        states[name] = block(*task.BINS[name.split("_")[0]], task.TOP + 0.01)
    assert task.success(states)
    assert task.score(states) == 1
    states["red_1"] = block(*task.BINS["blue"], task.TOP + 0.01)
    assert not task.success(states)
    assert abs(task.score(states) - 0.8) < 1e-9


def test_lifting_and_progress_vary_continuously():
    states = {name: block(0.34, y, task.BELT_TOP) for name, y in task.STARTS}
    states["red_1"] = block(0.34, 0.24, task.BELT_TOP + 0.02)
    lifted = task.score(states)
    states["red_1"] = block(0.25, 0.22, task.BELT_TOP + 0.02)
    nearer = task.score(states)
    states["red_1"] = block(0.25, 0.22, task.BELT_TOP + 0.04)
    higher = task.score(states)
    assert 0 < lifted < nearer < higher < 0.2
