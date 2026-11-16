import os
"""Rigid two-lever nutcracker with a native breakable walnut seam."""
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import math,hashlib
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from nutcracker_task.config import *
ROOT=Path(__file__).resolve().parents[1]
FEEDBACK={}
NAMES=('upper','lower','shell_a','shell_b')
def arr(x):
 x=x.torch if hasattr(x,'torch') else x
 return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x)
class NutcrackerEmbodiment(YamArenaEmbodiment):
 def __init__(self,**kwargs):
  for k,v in dict(task=str(ROOT/'task.py'),device='cpu',control_hz=20,cam_width=640,cam_height=360,object_jitter=0.).items():kwargs.setdefault(k,v)
  super().__init__(**kwargs);h=hashlib.sha256()
  for p in sorted(ROOT.rglob('*')):
   if p.is_file() and p.suffix in ('.py','.usda','.toml'):h.update(p.relative_to(ROOT).as_posix().encode()+b'\0'+p.read_bytes())
  self.info=replace(self.info,name='yam_nutcracker',environment_revision=h.hexdigest());self.history=SimpleNamespace(score=0.)
 def _ensure_sim(self,goal):
  if self._env is not None:return
  from yam_arena import inspect_embodiment as base
  if base._APP is None:
   import atexit
   from isaaclab.app import AppLauncher
   base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app;atexit.register(base._close_app_at_exit)
  from yam_arena import scene,embodiment as robots
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  import isaaclab.sim as sim
  original,old_callback,old_robot=scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment
  def spawn(spec):
   return Object(name=spec.name,usd_path=str(ROOT/'assets'/f'{spec.name}.usda'),object_type=ObjectType.BASE if spec.name=='fixture' else ObjectType.RIGID,initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=((math.sqrt(.5),0.,0.,math.sqrt(.5)) if spec.name.startswith('shell_') else (0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2)))),spawn_cfg_addon=dict(physics_material=sim.RigidBodyMaterialCfg(static_friction=1.2,dynamic_friction=1.0),collision_props=sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),rigid_props=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=64,solver_velocity_iteration_count=16,max_depenetration_velocity=.2)))
  def callback(hz=None):
   base=old_callback(hz)
   def configure(cfg):
    cfg=base(cfg);cfg.sim.dt=1/PHYSICS_HZ;cfg.decimation=24;cfg.sim.render_interval=24;cfg.sim.physics.enable_ccd=True;cfg.sim.enable_scene_query_support=True;return cfg
   return configure
  class NativeRobot(old_robot):
   def __init__(robot,**opts):opts.update(cam_width=640,cam_height=360,robot_contact_offset=.0002);super().__init__(**opts)
  scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=spawn,callback,NativeRobot
  try:super()._ensure_sim(goal)
  finally:scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=original,old_callback,old_robot
  import omni.physx
  self.physics=omni.physx.get_physx_simulation_interface();self.native=omni.physx.get_physx_interface();self._base.sim.set_setting('/physics/disableContactProcessing',False)
  self.subscription=self.physics.subscribe_full_contact_report_events(lambda *args:None)
  from omni.physx.bindings._physx import SimulationEvent
  self.event_subscription=self.native.get_simulation_event_stream_v2().create_subscription_to_pop_by_type(int(SimulationEvent.JOINT_BREAK),self._on_break)
  self.active=False
  raw=self._base.sim.step
  def guarded_step(*args,**kwargs):
   result=raw(*args,**kwargs)
   if self.active:self._contact_check()
   return result
  self._base.sim.step=guarded_step
  from pxr import PhysxSchema
  for n in NAMES:PhysxSchema.PhysxContactReportAPI.Apply(self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+n)).CreateThresholdAttr(0.)
 def _on_break(self,event):
  from pxr import PhysicsSchemaTools
  path=PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath'])
  if str(path).endswith('/WalnutSeam') and self.active:
   headers,contacts,_=self.physics.get_full_contact_report()
   print('BREAK_CONTACTS',[(str(PhysicsSchemaTools.intToSdfPath(int(h.collider0))),str(PhysicsSchemaTools.intToSdfPath(int(h.collider1))),str(PhysicsSchemaTools.intToSdfPath(int(h.actor0))),str(PhysicsSchemaTools.intToSdfPath(int(h.actor1))),h.num_contact_data) for h in headers if h.num_contact_data],flush=True)
   self._contact_check()
   self.history.broken=True;self.remove_broken_joint=True;self.history.valid_break=all(self.ticks-self.contact_tick[n]<=120 for n in ('upper','lower'));print('JOINT_BREAK',str(path),'valid tool contacts',self.history.valid_break,'ages',self.ticks,{n:self.ticks-v for n,v in self.contact_tick.items()},'peaks',self.contact_peak,flush=True)
 def _contact_check(self):
  from pxr import PhysicsSchemaTools
  self.ticks+=1
  headers,contacts,_=self.physics.get_full_contact_report()
  for h in headers:
   if not h.num_contact_data:continue
   paths=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.actor0,h.actor1)]
   names=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.collider0,h.collider1)]
   if any('RightArm' in p for p in paths) and any('/upper' in p or '/lower' in p for p in paths):self.robot_contacts.add(tuple(names))
   for n in ('upper','lower'):
    if any(p.endswith('/'+n) for p in paths) and any(p.endswith('/shell_a') or p.endswith('/shell_b') for p in paths):
     self.contact_tick[n]=self.ticks
     force=sum(np.linalg.norm(c.impulse) for c in contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data])*PHYSICS_HZ
     self.contact_peak[n]=max(self.contact_peak[n],float(force))
 def _joints(self):
  from pxr import UsdPhysics,Gf,Sdf
  s=self._base.sim.stage;base='/World/envs/env_0/'
  mount=UsdPhysics.FixedJoint.Define(s,base+'MountPin');mount.CreateBody1Rel().SetTargets([Sdf.Path(base+'upper')]);mount.CreateLocalPos0Attr(Gf.Vec3f(*map(float,HINGE)));mount.CreateLocalRot0Attr(Gf.Quatf(math.cos(FIGURE_YAW/2),0,0,math.sin(FIGURE_YAW/2)));mount.CreateExcludeFromArticulationAttr(True)
  joint=UsdPhysics.RevoluteJoint.Define(s,base+'NutcrackerHinge');joint.CreateBody0Rel().SetTargets([Sdf.Path(base+'upper')]);joint.CreateBody1Rel().SetTargets([Sdf.Path(base+'lower')]);joint.CreateAxisAttr('X');joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,JAW_PIVOT)));joint.CreateLocalPos1Attr(Gf.Vec3f(*map(float,JAW_PIVOT)));joint.CreateLowerLimitAttr(-45.);joint.CreateUpperLimitAttr(3.);joint.CreateCollisionEnabledAttr(False);joint.CreateExcludeFromArticulationAttr(True)
  UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath(base+'upper')).CreateFilteredPairsRel().SetTargets([Sdf.Path(base+'lower')])
  self._joints_seam_only()
 def _joints_seam_only(self):
  from pxr import UsdPhysics,Gf,Sdf
  s=self._base.sim.stage;base='/World/envs/env_0/'
  seam=UsdPhysics.FixedJoint.Define(s,base+'WalnutSeam');seam.CreateBody0Rel().SetTargets([Sdf.Path(base+'shell_a')]);seam.CreateBody1Rel().SetTargets([Sdf.Path(base+'shell_b')]);seam.CreateLocalPos0Attr(Gf.Vec3f(0,-.008,0));seam.CreateLocalPos1Attr(Gf.Vec3f(0,.008,0));seam.CreateBreakForceAttr(float(os.environ.get('NUT_SEAM_FORCE','200.')));seam.CreateBreakTorqueAttr(float(os.environ.get('NUT_SEAM_TORQUE','5.')));seam.CreateExcludeFromArticulationAttr(True);seam.CreateCollisionEnabledAttr(True)
 def reset(self,scene,*,seed=None):
  self._ensure_sim(None);self.active=False
  for n in ('NutcrackerHinge','WalnutSeam','MountPin','BenchPin','BenchMotor'):self._base.sim.stage.RemovePrim('/World/envs/env_0/'+n)
  self._instruction=scene.instruction;self._held=0;self._best_score=0.;self._env.reset()
  # Reset all four free roots before creating physical joints; no pose writes during a trial.
  for spec in self.task.objects:
   if spec.name=='fixture':continue
   obj=self._base.scene[spec.name];pose=obj.data.default_root_pose.torch.clone();pose[:,:3]=self._torch.tensor(spec.pos,dtype=pose.dtype,device=pose.device);pose[:,3:]=self._torch.tensor(([math.sqrt(.5),0.,0.,math.sqrt(.5)] if spec.name.startswith('shell_') else [0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2)]),dtype=pose.dtype,device=pose.device);obj.write_root_pose_to_sim_index(root_pose=pose);obj.write_root_velocity_to_sim_index(root_velocity=self._torch.zeros_like(obj.data.default_root_vel.torch))
  self.history=SimpleNamespace(score=0.,broken=False,valid_break=False,hold=0.,placement_hold=0.);self.remove_broken_joint=False;self.task.module.HISTORY=self.history;self.ticks=0;self.contact_tick={'upper':-10000,'lower':-10000};self.contact_peak={'upper':0.,'lower':0.};self.robot_contacts=set();self._joints();self.active=True
  self._read();print('RESET_BEFORE_SETTLE',{n:v.tolist() for n,v in FEEDBACK['poses'].items()},flush=True)
  for i in range(10):
   self._env.step(self._ready)
   if i in (0,1,9):
    self._read();print('RESET_STEP',i,{n:v.tolist() for n,v in FEEDBACK['poses'].items()},flush=True)
  for _ in range(4):self._base.sim.render()
  self._t=0.;FEEDBACK.clear();self._read();return self._observation()
 def _read(self):
  from yam_arena.urdf import quat_xyzw_to_matrix
  from yam_arena import ik,station
  poses={};rotations={}
  for n in NAMES:
   v=self.native.get_rigidbody_transformation('/World/envs/env_0/'+n);poses[n]=np.asarray(v['position']);rotations[n]=quat_xyzw_to_matrix(v['rotation'])
  hands={}
  for side,key in [('left','left_robot'),('right','right_robot')]:
   arm,fingers=self._ids[key];q=arr(self._base.scene[key].data.joint_pos)[0];p,r=ik.tcp_pose(q[arm],station.BASES[side]);hands[side]=dict(tcp=p,rotation=r,joints=q[arm],fingers=q[fingers])
  FEEDBACK.update(poses=poses,rotations=rotations,hands=hands);return poses,rotations
 def _task_step(self):
  if self.remove_broken_joint:
   self._base.sim.stage.RemovePrim('/World/envs/env_0/WalnutSeam');self.remove_broken_joint=False
  poses,rot=self._read();d=np.linalg.norm(poses['shell_a']-poses['shell_b']);h=self.history
  # Only a native break event with recent contact from both jaws qualifies.
  points=list(poses.values())+[poses['lower']+rot['lower']@LEVER_LOCAL]
  clear=all(-float(np.sum(v['fingers']))>=.070 and min(np.linalg.norm(v['tcp']-p) for p in points)>=.08 for v in FEEDBACK['hands'].values())
  on_table=all(TABLE-.005<=poses[n][2]<=TABLE+.22 for n in ('shell_a','shell_b')) and all(np.linalg.norm(poses[n]-HINGE)<.01 for n in ('upper','lower'))
  # Matching shell-rim landmarks separate under opening or sliding fracture.
  # At reset their world coordinates coincide; this measures visible rim offset.
  a,b=poses['shell_a'],poses['shell_b'];ra,rb=rot['shell_a'],rot['shell_b']
  offsets=[np.linalg.norm((a+ra@np.array([x,-.008,z]))-(b+rb@np.array([x,.008,z]))) for x,z in ((.016,0),(-.016,0),(0,.016),(0,-.016))]
  rim_offset=float(max(offsets))
  h.hold=h.hold+1 if h.broken and h.valid_break and rim_offset>=.003 and clear and on_table else 0
  center=(poses['shell_a']+poses['shell_b'])/2
  placed=not h.broken and abs(center[0]-NUT_BENCH[0])<=.018 and abs(center[1]-NUT_BENCH[1])<=.020 and abs(center[2]-NUT_BENCH[2])<=.025
  h.placement_hold=h.placement_hold+1 if placed and clear and on_table else 0
  h.score=1. if h.hold>=40 else .25 if h.placement_hold>=40 else 0.
  FEEDBACK.update(robot_contacts=sorted(self.robot_contacts),contact_peak=dict(self.contact_peak),rim_offset=rim_offset,placed=placed,separation=float(d),broken=h.broken,valid_break=h.valid_break,clear=clear,on_table=on_table,hold=h.hold,score=h.score)
  from inspect_robots.types import StepResult
  result=super()._task_step()
  return StepResult(observation=result.observation,terminated=False,info=result.info)
def make_embodiment(**kwargs):return NutcrackerEmbodiment(**kwargs)
