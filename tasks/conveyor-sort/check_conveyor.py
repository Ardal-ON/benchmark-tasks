"""Run a five-second, hands-off check of the conveyor in Isaac Sim."""
from pathlib import Path

import numpy as np
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY

from conveyor_sort.embodiment import ConveyorEmbodiment

names = ("red_1", "red_2", "red_3", "blue_1", "blue_2")
emb = ConveyorEmbodiment(task=str(Path(__file__).with_name("task.py")), device="cpu", object_jitter=0,
                         control_hz=20, cam_width=320, cam_height=176)
emb.reset(Scene(id="belt_check", instruction="Hold both arms still."))
from yam_arena.scene import object_states  # Isaac must be started first
start = object_states(emb._env, names)
action = Action(data=np.array([*READY, 1.0, *READY, 1.0]))
for _ in range(100):
    emb.step(action)
end = object_states(emb._env, names)
for name in names:
    displacement = float(end[name].pos[1] - start[name].pos[1])
    height_change = float(end[name].pos[2] - start[name].pos[2])
    print(f"{name}: moved {displacement:.4f} m, height change {height_change:.4f} m")
    assert 0.055 <= displacement <= 0.065, name
    assert abs(height_change) < 0.005, name
emb.close()
