"""Seat a vacuum brush bar and lock the end cap with a quarter turn of a coin."""

from dataclasses import dataclass
from importlib.machinery import PathFinder
import json
from math import pi
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
TOP = TABLE_TOP[SCENE]
HEAD = (.407, -.185, TOP + .158)
COIN = (.257, -.331, TOP + .030)
STROKE = .100
OBJECTS = [
    Box("head", HEAD, size=(.330, .15, .08)),
    Box("cleaner_housing", HEAD, size=(.232, .128, .08)),
    Box("coin", COIN, size=(.02426, .00175, .02426), mass=.00567),
    Box("head_support", (HEAD[0] + .121, HEAD[1] + .015, TOP + .060), size=(.176, .110, .120),
        color=(.50, .49, .48)),
    Box("coin_base", (COIN[0], COIN[1], TOP + .0089), size=(.040, .028, .0178),
        color=(.42, .43, .43)),
    Box("coin_rail_left", (COIN[0], COIN[1] - .004, TOP + .0190), size=(.028, .0054, .0035),
        color=(.42, .43, .43)),
    Box("coin_rail_right", (COIN[0], COIN[1] + .004, TOP + .0190), size=(.028, .0054, .0035),
        color=(.42, .43, .43)),
]

measurement = None
last_turn = 0.0
coin_rotation = 0.0


def score(objects):
    if measurement is None:
        return 0.0
    slide, turn, slide_speed, turn_speed = measurement
    if not all(np.isfinite(measurement)) or abs(slide_speed) > .004 or abs(turn_speed) > .1:
        return 0.0
    if abs(slide - STROKE) > .003:
        return 0.25 if slide >= .050 else 0.0
    withdrawn = float(objects["coin"].high[0]) < HEAD[0] - .027
    if turn >= np.deg2rad(85) and coin_rotation >= np.deg2rad(55) and withdrawn:
        return 1.0
    return 0.5


def success(objects):
    return score(objects) == 1.0


def measure_coin_rotation(coin):
    global last_turn, coin_rotation
    slide, turn, _, _ = measurement
    if slide < .010:
        coin_rotation = 0.0
    cap_x = HEAD[0] + slide - STROKE
    in_slot = cap_x - .0105 <= coin.high[0] <= cap_x - .003
    centered = np.linalg.norm(coin.pos[1:] - HEAD[1:]) < .006
    if slide >= STROKE - .003 and in_slot and centered:
        coin_rotation = max(0.0, coin_rotation + turn - last_turn)
    last_turn = turn


@dataclass
class ToolWaypoint(Waypoint):
    pitch: float = -pi / 2
    twist: float = 0.0


def enable_tool_plan():
    previous = ArmPlan.target

    def target(plan, t):
        if not plan.waypoints or not isinstance(plan.waypoints[0], ToolWaypoint):
            return previous(plan, t)
        p0, r0, f0 = plan.start_pos, plan.start_rot, plan.finger_start
        for waypoint in plan.waypoints:
            p1 = np.asarray(waypoint.pos)
            pitch = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), waypoint.pitch)
            twist = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), waypoint.twist)
            r1 = twist @ pitch @ grip_rotation()
            if t < waypoint.duration:
                s = ik.smoothstep(t / waypoint.duration)
                return p0 + s * (p1 - p0), ik.slerp_matrix(r0, r1, s), f0 + s * (waypoint.finger - f0), waypoint.label
            t -= waypoint.duration
            p0, r0, f0 = p1, r1, waypoint.finger
        return p0, r0, f0, "done"

    ArmPlan.target = target


def demo():
    enable_tool_plan()
    x, y, z = HEAD
    cx, cy, cz = COIN
    open_, closed = FINGER_OPEN, FINGER_CLOSED
    lock_twist = np.deg2rad(110)
    poses = [
        ToolWaypoint(2.0, (x - .151, y - .015, z), closed, "behind brush bar"),
        ToolWaypoint(3.2, (x - .010, y - .015, z), closed, "press bar to its seat"),
        ToolWaypoint(1.50, (x - .010, y - .015, z), closed, "settle at the physical stop"),
        ToolWaypoint(1.0, (x - .092, y - .015, z), closed, "withdraw"),
        ToolWaypoint(1.2, (cx, cy, TOP + .132), open_, "above the coin", pitch=0.0),
        ToolWaypoint(1.2, (cx, cy, cz + .0065), open_, "flank the coin", pitch=0.0),
        ToolWaypoint(1.2, (cx, cy, cz + .0065), open_, "settle over the coin", pitch=0.0),
        ToolWaypoint(1.0, (cx, cy, cz + .0065), closed, "grasp coin by trailing edge", pitch=0.0),
        ToolWaypoint(1.0, (cx, cy, TOP + .116), closed, "lift coin", pitch=0.0),
        ToolWaypoint(1.5, (x - .085, y, z + .035), closed, "orient coin for slot"),
        ToolWaypoint(.8, (x - .055, y + .0018, z + .0007), closed, "line up with slot"),
        ToolWaypoint(1.4, (x - .018, y + .0018, z + .0007), closed, "insert coin edge"),
        ToolWaypoint(2.5, (x - .017, y + .0018, z + .0007), closed, "turn cap to lock", twist=lock_twist),
        ToolWaypoint(.5, (x - .017, y + .0018, z + .0007), closed, "settle against lock stop", twist=lock_twist),
        ToolWaypoint(1.3, (x - .090, y + .0018, z + .0007), closed, "withdraw coin", twist=lock_twist),
    ]
    return {"right": poses}


def patch_scene(module):
    previous_object = module.task_object
    previous_states = module.object_states

    def object_config(spec):
        if spec.name not in ("head", "coin", "cleaner_housing"):
            return previous_object(spec)

        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.assets import ArticulationCfg
        from isaaclab.sim import MassPropertiesCfg, RigidBodyMaterialCfg, UsdFileCfg
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

        class Head(Object):
            def _generate_articulation_cfg(self):
                actuators = {
                    "slide": ImplicitActuatorCfg(joint_names_expr=["roller_slide"], stiffness=0.0, damping=6.0),
                    "lock": ImplicitActuatorCfg(
                        joint_names_expr=["cap_turn"], stiffness=0.0, damping=.05, friction=.08,
                    ),
                }
                cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
                return self._add_initial_pose_to_cfg(cfg)

        folder = Path(__file__).resolve().parent
        if spec.name == "cleaner_housing":
            return Object(
                name=spec.name, object_type=ObjectType.BASE,
                spawner_cfg=UsdFileCfg(usd_path=str(folder / "housing.usda")),
                initial_pose=Pose(position_xyz=spec.pos),
            )
        if spec.name == "head":
            return Head(
                name=spec.name, object_type=ObjectType.ARTICULATION,
                spawner_cfg=UsdFileCfg(usd_path=str(folder / "head.usda")),
                initial_pose=Pose(position_xyz=spec.pos),
            )
        return Object(
            name=spec.name, object_type=ObjectType.RIGID,
            spawner_cfg=UsdFileCfg(
                usd_path=str(folder / "coin.usda"),
                rigid_props=PhysxRigidBodyPropertiesCfg(),
                mass_props=MassPropertiesCfg(mass=.00567),
                physics_material=RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=.85),
            ),
            initial_pose=Pose(position_xyz=spec.pos),
        )

    def observe(environment, names):
        global measurement
        if "head" in names:
            asset = environment.unwrapped.scene["head"]
            pose, velocity = asset.data.joint_pos, asset.data.joint_vel
            pose, velocity = getattr(pose, "torch", pose), getattr(velocity, "torch", velocity)
            slide, pivot = (asset.joint_names.index(n) for n in ("roller_slide", "cap_turn"))
            measurement = (float(pose[0, slide]), float(pose[0, pivot]), float(velocity[0, slide]),
                           float(velocity[0, pivot]))
            names = [n for n in names if n not in ("head", "cleaner_housing")]
        result = previous_states(environment, names)
        if "coin" in result and measurement is not None:
            measure_coin_rotation(result["coin"])
        if "coin" in result and (trace := os.environ.get("ROBOCURVE_VACUUM_TRACE")):
            from yam_arena import station

            q = environment.unwrapped.scene["right_robot"].data.joint_pos
            q = getattr(q, "torch", q)[0].detach().cpu().numpy()
            tcp, orientation = ik.tcp_pose(q[:6], station.BASES["right"])
            values = {"slide": measurement[0], "turn": measurement[1], "coin_rotation": coin_rotation,
                      "coin": result["coin"].pos.tolist(),
                      "coin_quat": result["coin"].quat_xyzw.tolist(), "tcp": tcp.tolist(),
                      "approach": orientation[:, 2].tolist(), "finger": q[6:].tolist()}
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
