"""Physics diagnostic only: start all parts at the proposed assembled poses."""
from dataclasses import replace
import importlib.util
from pathlib import Path
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
spec=importlib.util.spec_from_file_location('square_task',Path(__file__).resolve().parents[1]/'task.py')
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
SCENE=t.SCENE
OBJECTS=[replace(obj,xy=goal,yaw=0.0) for obj,goal in zip(t.OBJECTS[:4],t.GOAL)] + t.OBJECTS[4:]
score,success,distance=t.score,t.success,t.distance
def demo(): return {'left':[Waypoint(5.0,(.12,.30,t.TOP+.12),FINGER_OPEN,'wait clear')]}
