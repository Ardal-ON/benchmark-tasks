"""A manual press with a flanged bushing started square in a chamfered steel pivot housing."""

from dataclasses import dataclass
from importlib.machinery import PathFinder
import json
from math import cos, pi, sin
import os
from pathlib import Path
import sys

import numpy as np

from yam_arena import demo as yam_demo
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
PRESS = (0.420, -0.125, TOP)
ROTATION = -pi / 2
HUB_Z = 0.273
HOUSING_TOP = 0.075
TRAVEL = 0.015
PITCH_RADIUS = 0.020
FIT_SLIP = 35.0 / 280_000.0
PLATEN_GAP = 0.012
ASSETS = Path(__file__).resolve().parent / "assets"
OBJECTS = [
    Box("arbor_press", PRESS, size=(0.20, 0.20, 0.30), yaw=ROTATION),
    Box("bearing_fit", (PRESS[0] - 0.018, PRESS[1], TOP + HOUSING_TOP), size=(0.052, 0.052, 0.0555)),
]


@dataclass
class Reading:
    ram: float
    handle: float
    insertion: float
    insertion_speed: float
    hands_clear: bool
    driven_advance: float


reading: Reading | None = None
fit_reference = 0.0
driven_advance = 0.0


def grade(sample):
    if sample is None or not np.isfinite(
        (sample.ram, sample.handle, sample.insertion, sample.insertion_speed, sample.driven_advance)
    ).all():
        return 0.0
    if abs(sample.insertion_speed) > 0.001 or sample.ram > 0.004 or not sample.hands_clear:
        return 0.0
    if sample.driven_advance < sample.insertion - 0.00075:
        return 0.0
    if abs(sample.insertion - TRAVEL) <= 0.00020:
        return 1.0
    return 0.5 if TRAVEL / 2 <= sample.insertion < TRAVEL else 0.0


def advance_under_platen(proven, previous_depth, ram, depth):
    """Count gradual movement that occurs while the platen follows the sleeve."""
    if abs(ram - PLATEN_GAP - depth) < 0.001 and 0.0 < depth - proven < 0.002:
        if depth > previous_depth + 0.00001:
            return depth
    return proven


def score(objects):
    return grade(reading)


def success(objects):
    return grade(reading) == 1.0


def knob(angle):
    return (
        PRESS[0] + 0.015 - 0.160 * cos(angle),
        PRESS[1] - 0.063,
        TOP + HUB_Z - 0.160 * sin(angle),
    )


@dataclass
class PressWaypoint(Waypoint):
    pitch: float = -pi / 4


def press_target(plan, t):
    previous_pos = plan.start_pos
    previous_rot = plan.start_rot
    previous_finger = plan.finger_start
    for waypoint in plan.waypoints:
        position = np.asarray(waypoint.pos)
        rotation = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), waypoint.pitch) @ grip_rotation()
        if t < waypoint.duration:
            factor = ik.smoothstep(t / waypoint.duration)
            return (
                previous_pos + (position - previous_pos) * factor,
                ik.slerp_matrix(previous_rot, rotation, factor),
                previous_finger + (waypoint.finger - previous_finger) * factor,
                waypoint.label,
            )
        t -= waypoint.duration
        previous_pos, previous_rot, previous_finger = position, rotation, waypoint.finger
    return previous_pos, previous_rot, previous_finger, "done"


def _enable_press_waypoints():
    previous = ArmPlan.target

    def target(plan, t):
        if plan.waypoints and isinstance(plan.waypoints[0], PressWaypoint):
            return press_target(plan, t)
        return previous(plan, t)

    yam_demo.ArmPlan.target = target


def demo():
    _enable_press_waypoints()
    x, y, z = knob(0.0)
    arc = np.linspace(0.0, 1.42, 17)
    points = [
        PressWaypoint(2.0, (x - 0.050, y, z + 0.025), FINGER_OPEN, "before press grip"),
        PressWaypoint(1.0, (x, y, z + 0.005), FINGER_OPEN, "approach grip"),
        PressWaypoint(0.8, (x, y, z + 0.005), FINGER_CLOSED, "close on grip"),
    ]
    for angle in (*arc[1:], *reversed(arc[:-1])):
        gx, gy, gz = knob(float(angle))
        points.append(PressWaypoint(0.22, (gx, gy, gz + 0.005), FINGER_CLOSED, "turn press lever"))
    points.extend([
        PressWaypoint(0.8, (x, y, z + 0.005), FINGER_OPEN, "release raised lever"),
        PressWaypoint(1.2, (x - 0.06, y, z + 0.015), FINGER_OPEN, "clear lever"),
    ])
    return {"right": points}


def _make_articulation(spec):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim import UsdFileCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    press = spec.name == "arbor_press"
    path = ASSETS / ("press.usda" if press else "fit.usda")

    class PressObject(Object):
        def _generate_articulation_cfg(self):
            actuators = (
                {
                    "lever": ImplicitActuatorCfg(
                        joint_names_expr=["lever_pinion"], stiffness=0.86, damping=0.12, effort_limit_sim=1.0,
                    ),
                    "ram": ImplicitActuatorCfg(joint_names_expr=["ram_slide"], stiffness=0.0, damping=3.0),
                }
                if press else
                {
                    "interference": ImplicitActuatorCfg(
                        joint_names_expr=["fit_slide"], stiffness=280_000.0, damping=20.0, effort_limit_sim=35.0,
                    )
                }
            )
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return PressObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=UsdFileCfg(usd_path=str(path)),
        initial_pose=Pose(
            position_xyz=spec.pos, rotation_xyzw=(0.0, 0.0, sin(spec.yaw / 2), cos(spec.yaw / 2)),
        ),
    )


def _patch_scene(module):
    base_object = module.task_object
    base_states = module.object_states

    def task_object(spec):
        if spec.name in ("arbor_press", "bearing_fit"):
            return _make_articulation(spec)
        return base_object(spec)

    def object_states(env, names):
        global reading, fit_reference, driven_advance
        if set(names) == {"arbor_press", "bearing_fit"}:
            press = env.unwrapped.scene["arbor_press"]
            fit = env.unwrapped.scene["bearing_fit"]
            ram, hinge = (press.joint_names.index(j) for j in ("ram_slide", "lever_pinion"))
            q = _torch(press.data.joint_pos)
            fit_q, fit_qd = _torch(fit.data.joint_pos), _torch(fit.data.joint_vel)
            angle = float(q[0, hinge])
            lever_grip = np.asarray(knob(angle))
            press_throat = np.asarray(OBJECTS[1].pos)
            hands_clear = True
            for side in ("left", "right"):
                arm = env.unwrapped.scene[f"{side}_robot"]
                tcp, _ = ik.tcp_pose(_torch(arm.data.joint_pos)[0, :6].detach().cpu().numpy(), station.BASES[side])
                hands_clear &= min(np.linalg.norm(tcp - lever_grip), np.linalg.norm(tcp - press_throat)) > 0.032
            ram_depth = float(q[0, ram])
            sleeve_depth = -float(fit_q[0, 0])
            previous_depth = reading.insertion if reading is not None else 0.0
            driven_advance = advance_under_platen(driven_advance, previous_depth, ram_depth, sleeve_depth)
            reading = Reading(ram_depth, angle, sleeve_depth, -float(fit_qd[0, 0]), bool(hands_clear), driven_advance)
            # Yielding reference models sliding interference without a force that advances the bushing.
            fit_reference = min(fit_reference, -reading.insertion + FIT_SLIP)
            target = fit_q.clone()
            target.fill_(fit_reference)
            fit.set_joint_position_target(target)
            if path := os.environ.get("ROBOCURVE_PRESS_TRACE"):
                with open(path, "a", encoding="utf-8") as output:
                    output.write(json.dumps(vars(reading)) + "\n")
            return {}
        return base_states(env, names)

    module.task_object, module.object_states = task_object, object_states


def _torch(value):
    return getattr(value, "torch", value)


def _patch_reset(module):
    previous = module.YamArenaEmbodiment.reset

    def reset(self, *args, **kwargs):
        global reading, fit_reference, driven_advance
        reading, fit_reference, driven_advance = None, 0.0, 0.0
        return previous(self, *args, **kwargs)

    module.YamArenaEmbodiment.reset = reset


class _SceneLoader:
    def __init__(self, loader):
        self.loader = loader

    def create_module(self, spec):
        return self.loader.create_module(spec)

    def exec_module(self, module):
        self.loader.exec_module(module)
        _patch_scene(module)


class _SceneFinder:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "yam_arena.scene":
            spec = PathFinder.find_spec(fullname, path)
            if spec is not None and spec.loader is not None:
                spec.loader = _SceneLoader(spec.loader)
            return spec
        return None


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
else:
    sys.meta_path.insert(0, _SceneFinder())
if "yam_arena.inspect_embodiment" in sys.modules:
    _patch_reset(sys.modules["yam_arena.inspect_embodiment"])
