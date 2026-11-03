"""YAM station with a PhysX contact-velocity conveyor and visual stripes."""
from yam_arena.inspect_embodiment import YamArenaEmbodiment

BELT_SPEED = 0.012  # metres per second in world +y


def spawn_belt(prim_path, cfg, **kwargs):
    from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
    from pxr import Gf, PhysxSchema

    prim = spawn_cuboid(prim_path, cfg, **kwargs)
    surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
    surface.CreateSurfaceVelocityEnabledAttr().Set(True)
    surface.CreateSurfaceVelocityLocalSpaceAttr().Set(False)
    surface.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, BELT_SPEED, 0.0))
    return prim


class ConveyorEmbodiment(YamArenaEmbodiment):
    def __init__(self, **kwargs):
        kwargs.setdefault("device", "cpu")
        if kwargs["device"] != "cpu":
            raise ValueError("Use CPU PhysX: GPU physics did not maintain conveyor contacts in validation")
        super().__init__(**kwargs)

    def _ensure_sim(self, goal):
        # Task-file shapes normally make fixed static colliders. Replace just
        # the belt with a kinematic collider, and make the stripes visual-only.
        import atexit
        import os
        from yam_arena import inspect_embodiment as base_module
        if base_module._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            kwargs = {"headless": self.headless, "device": self.device, "enable_cameras": True}
            if not self.headless:
                kwargs["visualizer"] = ["kit"]
                os.environ["DISPLAY"] = base_module.window_display(os.environ)
            base_module._APP = AppLauncher(**kwargs).app
            atexit.register(base_module._close_app_at_exit)
        from yam_arena import scene
        from isaaclab.sim import PhysxRigidBodyPropertiesCfg

        original = scene.task_object

        def make_object(spec):
            obj = original(spec)
            if spec.name == "belt":
                obj.spawner_cfg.rigid_props = PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
                obj.spawner_cfg.func = spawn_belt
            elif spec.name.startswith("stripe_"):
                obj.spawner_cfg.collision_props = None
            if spec.name == "belt" or spec.name.startswith("stripe_"):
                obj.object_cfg = obj._init_object_cfg()
            return obj

        scene.task_object = make_object
        try:
            return super()._ensure_sim(goal)
        finally:
            scene.task_object = original

    def step(self, action):
        from pxr import Gf, UsdGeom
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        for i in range(10):
            prim = stage.GetPrimAtPath(f"/World/envs/env_0/stripe_{i}")
            if not prim.IsValid():
                raise RuntimeError(f"Missing conveyor stripe {i}")
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    y = ((-0.54+i*0.12 + self._t*BELT_SPEED + 0.60) % 1.20) - 0.60
                    op.Set(Gf.Vec3d(0.34, y, 0.7678))
                    break
        return super().step(action)


def make_embodiment(**kwargs):
    return ConveyorEmbodiment(**kwargs)
