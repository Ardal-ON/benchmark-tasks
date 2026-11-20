"""Hold a self-inking stamp slightly down, eject its old pad and seat the replacement."""

from dataclasses import dataclass
from importlib.machinery import PathFinder
import json
from math import cos, sin, pi
import os
from pathlib import Path
import sys

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
TABLE = TABLE_TOP[SCENE]
TOP = TABLE + .105
X, Y = 0.32, 0.04
PAD_Z = TOP + 0.046
PAD_SIZE = (0.049, 0.042, 0.008)
YAW = -pi / 2
OBJECTS = [
    Box("stamp", (X, Y, TOP), size=(0.080, 0.056, 0.091), yaw=YAW),
    Box("old_pad", (X, Y, PAD_Z), size=PAD_SIZE, mass=0.009, color=(0.18, 0.19, 0.18), yaw=YAW),
    Box("new_pad", (X - .075, Y, PAD_Z + .0012), size=(.0685, .049, .034), color=(.16, .40, .57)),
    Box("bench_fixture", (X + .003, Y, TABLE + .0525), size=(.290, .130, .105),
        color=(.43, .43, .42)),
    Box("incoming_support", (X - .069, Y, TOP + 0.011), size=(0.072, 0.052, 0.022),
        color=(0.54, 0.52, 0.49)),
    Box("incoming_runner_left", (X - .069, Y + .022, TOP + .032), size=(.072, .008, .020),
        color=(.34, .35, .36)),
    Box("incoming_runner_right", (X - .069, Y - .022, TOP + .032), size=(.072, .008, .020),
        color=(.34, .35, .36)),
    Box("incoming_guide_left", (X - .061, Y + .0276, TOP + .044), size=(.056, .004, .008),
        color=(.34, .35, .36)),
    Box("incoming_guide_right", (X - .061, Y - .0276, TOP + .044), size=(.056, .004, .008),
        color=(.34, .35, .36)),
    Box("catch_tray_riser", (X + .075, Y, TOP + .0145), size=(.105, .092, .029),
        color=(.43, .43, .42)),
    Box("used_pad_tray", (X + .075, Y, TOP + .035), size=(0.110, 0.100, 0.012),
        color=(0.54, 0.52, 0.49)),
]

cap_position = None
pad_motion = None


def score(objects):
    old = objects["old_pad"]
    on_tray = old.resting() and X + .019 <= old.low[0] and old.high[0] <= X + .131 and abs(old.low[2] - TOP - .041) < .002
    on_tray = on_tray and Y - .05 <= old.low[1] and old.high[1] <= Y + .05
    if not on_tray:
        return 0.0
    installed = pad_motion is not None and abs(pad_motion[0] - .075) < .0025 and abs(pad_motion[1]) < .005
    return 1.0 if installed and cap_position is not None and -.002 <= cap_position <= .001 else 0.5


def success(objects):
    return score(objects) == 1.0


@dataclass
class CapWaypoint(Waypoint):
    lean: float = -0.80
    pitch: float = 0.0


def enable_cap_plan():
    earlier = ArmPlan.target

    def target(plan, t):
        if not plan.waypoints or not isinstance(plan.waypoints[0], CapWaypoint):
            return earlier(plan, t)
        p0, r0, f0 = plan.start_pos, plan.start_rot, plan.finger_start
        for w in plan.waypoints:
            p1 = np.asarray(w.pos)
            lean = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), w.lean)
            pitch = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), w.pitch)
            r1 = lean @ pitch @ grip_rotation()
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return p0 + s * (p1 - p0), ik.slerp_matrix(r0, r1, s), f0 + s * (w.finger - f0), w.label
            t -= w.duration
            p0, r0, f0 = p1, r1, w.finger
        return p0, r0, f0, "done"

    ArmPlan.target = target


def demo():
    enable_cap_plan()
    hover, contact = TOP + .124, PAD_Z + .011
    closed = FINGER_CLOSED
    left = [
        CapWaypoint(2, (X + .004, Y + .018, TOP + .139), closed, "hover above cap", lean=-.35),
        CapWaypoint(2, (X + .004, Y + .020, TOP + .0715), closed, "depress stamp", lean=-.35),
        CapWaypoint(9.6, (X + .004, Y + .020, TOP + .0715), closed, "hold cartridge path open", lean=-.35),
        CapWaypoint(1.0, (X + .004, Y + .018, TOP + .139), FINGER_OPEN, "release cap", lean=-.35),
    ]
    def push_waypoint(duration, x, z, label, lateral=0, finger=closed):
        return CapWaypoint(duration, (X + x, Y + lateral, z), finger, label, lean=0, pitch=-.52)

    right = [
        push_waypoint(2.0, -.149, hover, "hover behind the tab"),
        push_waypoint(2.5, -.149, hover, "wait for cartridge path"),
        push_waypoint(1.7, -.149, contact, "align with the tab"),
        push_waypoint(3.8, -.044, contact, "slide fresh pad in and eject used pad"),
        push_waypoint(1.0, -.044, contact, "settle the cartridge"),
        push_waypoint(1.0, -.152, contact, "withdraw from the cartridge"),
        push_waypoint(1.0, -.120, hover, "clear stamp", lateral=-.07, finger=FINGER_OPEN),
    ]
    return {"left": left, "right": right}


def patch_scene(module):
    previous_object = module.task_object
    previous_states = module.object_states

    def object_config(spec):
        if spec.name in ("stamp", "new_pad"):
            from isaaclab.actuators import ImplicitActuatorCfg
            from isaaclab.assets import ArticulationCfg
            from isaaclab.sim import UsdFileCfg
            from isaaclab_arena.assets.object import Object
            from isaaclab_arena.assets.object_type import ObjectType
            from isaaclab_arena.utils.pose import Pose

            class GuidedPart(Object):
                def _generate_articulation_cfg(self):
                    cfg = ArticulationCfg(
                        prim_path=self.prim_path, spawn=self.spawner_cfg, actuators={"guide": drive},
                    )
                    return self._add_initial_pose_to_cfg(cfg)

            is_stamp = spec.name == "stamp"
            filename = "stamp.usda" if is_stamp else "fresh-pad.usda"
            drive = ImplicitActuatorCfg(
                joint_names_expr=["cap_slide" if is_stamp else "pad_slide"],
                stiffness=550.0 if is_stamp else 0.0,
                damping=8.0 if is_stamp else 0.6,
                effort_limit_sim=30.0,
            )
            return GuidedPart(
                name=spec.name,
                object_type=ObjectType.ARTICULATION,
                spawner_cfg=UsdFileCfg(usd_path=str(Path(__file__).resolve().with_name(filename))),
                initial_pose=Pose(position_xyz=spec.pos, rotation_xyzw=(0, 0, sin(spec.yaw / 2), cos(spec.yaw / 2))),
            )
        result = previous_object(spec)
        if spec.name.startswith("incoming_"):
            result.spawner_cfg.collision_props.collision_enabled = False
        if spec.name == "old_pad":
            result.spawner_cfg.collision_props.contact_offset = .0007
            result.spawner_cfg.physics_material.static_friction = .30
            result.spawner_cfg.physics_material.dynamic_friction = .25
        return result

    def observe(environment, names):
        global cap_position, pad_motion
        if "stamp" in names:
            joint = environment.unwrapped.scene["stamp"].data.joint_pos
            cap_position = float(getattr(joint, "torch", joint)[0, 0])
        if "new_pad" in names:
            pad = environment.unwrapped.scene["new_pad"].data
            q, v = pad.joint_pos, pad.joint_vel
            pad_motion = (float(getattr(q, "torch", q)[0, 0]), float(getattr(v, "torch", v)[0, 0]))
        names = [n for n in names if n not in ("stamp", "new_pad")]
        result = previous_states(environment, names)
        if "old_pad" in result and (trace := os.environ.get("ROBOCURVE_STAMP_TRACE")):
            from yam_arena import station

            values = {"cap": cap_position, "old": result["old_pad"].pos.tolist(), "pad_slide": pad_motion[0]}
            for arm in ("left", "right"):
                q = environment.unwrapped.scene[f"{arm}_robot"].data.joint_pos
                q = getattr(q, "torch", q)[0, :6].detach().cpu().numpy()
                values[f"{arm}_tcp"] = ik.tcp_pose(q, station.BASES[arm])[0].tolist()
            with open(trace, "a", encoding="utf-8") as stream:
                stream.write(json.dumps(values) + "\n")
        return result

    module.task_object = object_config
    module.object_states = observe


class _Loader:
    def __init__(self, loader):
        self.loader = loader

    def create_module(self, spec):
        return self.loader.create_module(spec)

    def exec_module(self, module):
        self.loader.exec_module(module)
        patch_scene(module)


class _Finder:
    def find_spec(self, fullname, path=None, target=None):
        if fullname == "yam_arena.scene":
            spec = PathFinder.find_spec(fullname, path)
            if spec and spec.loader:
                spec.loader = _Loader(spec.loader)
            return spec
        return None


if "yam_arena.scene" in sys.modules:
    patch_scene(sys.modules["yam_arena.scene"])
else:
    sys.meta_path.insert(0, _Finder())
