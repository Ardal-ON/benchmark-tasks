"""Native YAM cameras and actions with authored open-bore USD assets."""
from pathlib import Path
from dataclasses import replace
import hashlib
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from ratchet_task.config import *
ROOT=Path(__file__).resolve().parents[1]
FEEDBACK={}
def array(v):
    v=v.torch if hasattr(v,'torch') else v
    return v.detach().cpu().numpy() if hasattr(v,'detach') else np.asarray(v)
class RatchetEmbodiment(YamArenaEmbodiment):
    def __init__(self,walls=False,physics_hz=PHYSICS_HZ,**kwargs):
        kwargs.setdefault('object_jitter',0.)
        super().__init__(**kwargs)
        if self.object_jitter:raise ValueError('Joint-consistent layout requires object_jitter=0')
        self.walls=bool(walls);self.physics_hz=int(physics_hz)
        h=hashlib.sha256()
        for p in sorted(ROOT.rglob('*')):
            if p.is_file() and p.suffix in ('.py','.usda','.toml'):h.update(p.relative_to(ROOT).as_posix().encode()+b'\0'+p.read_bytes())
        self.info=replace(self.info,name='yam_confined_ratchet',environment_revision=h.hexdigest())
    def _ensure_sim(self,goal):
        if self._env is not None:return
        from yam_arena import inspect_embodiment as base
        if base._APP is None:
            import atexit
            from isaaclab.app import AppLauncher
            base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        import isaaclab.sim as sim
        old_object,old_callback=scene.task_object,scene.env_cfg_callback
        def spawn(spec):
            if spec.name not in self.task.names:return old_object(spec)
            props=dict(collision_props=sim.CollisionPropertiesCfg(contact_offset=.0001,rest_offset=0.),physics_material=sim.RigidBodyMaterialCfg(static_friction=.55,dynamic_friction=.4))
            if spec.free:props['rigid_props']=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=64,solver_velocity_iteration_count=16,max_depenetration_velocity=.2,linear_damping=.02,angular_damping=.02)
            return Object(name=spec.name,usd_path=str(ROOT/'assets'/f'{spec.name}.usda'),object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,initial_pose=Pose(position_xyz=tuple(map(float,spec.pos))),spawn_cfg_addon=props)
        def callback(hz=None):
            original=old_callback(hz)
            def configure(cfg):
                cfg=original(cfg);cfg.sim.dt=1/self.physics_hz;cfg.decimation=round(self.physics_hz/self.info.control_hz);cfg.sim.render_interval=cfg.decimation;cfg.sim.physics.enable_ccd=True
                return cfg
            return configure
        scene.task_object,scene.env_cfg_callback=spawn,callback
        try:super()._ensure_sim(goal)
        finally:scene.task_object,scene.env_cfg_callback=old_object,old_callback
        from pxr import UsdPhysics,Sdf
        self.drive_joint=UsdPhysics.RevoluteJoint.Define(self._base.sim.stage,'/World/envs/env_0/RatchetDriveJoint')
        self.drive_joint.CreateBody0Rel().SetTargets([Sdf.Path('/World/envs/env_0/handle')])
        self.drive_joint.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/drive')])
        self.drive_joint.CreateAxisAttr('Z');self.drive_joint.CreateCollisionEnabledAttr(False)
        self.drive_joint.CreateExcludeFromArticulationAttr(True)
    def _read(self):
        from yam_arena import ik,station
        from yam_arena.urdf import quat_xyzw_to_matrix
        poses={};rotations={}
        for n in self.task.names:
            v=array(self._base.arena_world.get_pose_w(n))[0];poses[n]=v[:3];rotations[n]=quat_xyzw_to_matrix(v[3:])
        hands={}
        for side,key in [('left','left_robot'),('right','right_robot')]:
            arm,fingers=self._ids[key];q=array(self._base.scene[key].data.joint_pos)[0];p,r=ik.tcp_pose(q[arm],station.BASES[side]);hands[side]=dict(tcp=p,rotation=r,joints=q[arm],fingers=q[fingers])
        FEEDBACK.update(poses=poses,rotations=rotations,hands=hands)
    def reset(self,scene,*,seed=None):
        obs=super().reset(scene,seed=seed);self._read();return obs
    def _task_step(self):
        self._read();return super()._task_step()
def make_embodiment(**kwargs):return RatchetEmbodiment(**kwargs)
