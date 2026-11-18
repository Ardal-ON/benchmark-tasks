import os
"""Rigid two-lever nutcracker with a native breakable walnut seam."""
from pathlib import Path
from dataclasses import replace,fields
from types import SimpleNamespace
from collections import deque
import math,hashlib
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from nutcracker_task.config import *
from nutcracker_task.scoring import Rubric
from nutcracker_task.geometry import rim_opening,fixture_stable,table_bounds_ok,supported,envelopes,grippers_clear
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
   if p.is_file() and p.suffix in ('.py','.usda','.toml','.csv'):h.update(p.relative_to(ROOT).as_posix().encode()+b'\0'+p.read_bytes())
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
    cfg=base(cfg);cfg.sim.dt=1/PHYSICS_HZ;cfg.decimation=24;cfg.sim.render_interval=24;cfg.sim.physics.enable_ccd=True;cfg.sim.enable_scene_query_support=True
    from isaaclab.sensors import CameraCfg
    from isaaclab_arena.utils.configclass import make_configclass
    from yam_arena.ik import matrix_to_quat_xyzw
    eye=np.array([.130,-.140,.875]);target=np.array([.280,-.040,.8775])
    back=eye-target;back/=np.linalg.norm(back);right=np.cross([0.,0.,1.],back);right/=np.linalg.norm(right);up=np.cross(back,right)
    quat=tuple(map(float,matrix_to_quat_xyzw(np.column_stack((right,up,back)))))
    witness=CameraCfg(prim_path='{ENV_REGEX_NS}/WitnessCam',height=540,width=960,
        data_types=['rgb'],update_period=.05,spawn=sim.PinholeCameraCfg(focal_length=24.,horizontal_aperture=20.955,clipping_range=(.005,20.)),
        offset=CameraCfg.OffsetCfg(pos=tuple(map(float,eye)),rot=quat,convention='opengl'),renderer_cfg=robots.RENDERER)
    values={f.name:getattr(cfg.scene,f.name) for f in fields(cfg.scene) if f.init}
    cls=make_configclass('NutcrackerWitnessSceneCfg',[('witness_cam',CameraCfg,witness)],bases=(type(cfg.scene),))
    cfg.scene=cls(**values)
    return cfg
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
   if self.active:
    if self.remove_broken_joint:
     self._remove_seam()
    self._contact_check()
   return result
  self._base.sim.step=guarded_step
  from pxr import PhysxSchema
  for n in NAMES:PhysxSchema.PhysxContactReportAPI.Apply(self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+n)).CreateThresholdAttr(0.)
 def _remove_seam(self):
  from pxr import UsdPhysics
  stage=self._base.sim.stage
  stage.RemovePrim('/World/envs/env_0/WalnutSeam')
  UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/World/envs/env_0/shell_a')).CreateFilteredPairsRel().SetTargets([])
  self.remove_broken_joint=False
 def _on_break(self,event):
  from pxr import PhysicsSchemaTools
  path=PhysicsSchemaTools.decodeSdfPath(*event.payload['jointPath'])
  if str(path).endswith('/WalnutSeam') and self.active:
   headers,contacts,_=self.physics.get_full_contact_report()
   print('BREAK_CONTACTS',[(str(PhysicsSchemaTools.intToSdfPath(int(h.collider0))),str(PhysicsSchemaTools.intToSdfPath(int(h.collider1))),str(PhysicsSchemaTools.intToSdfPath(int(h.actor0))),str(PhysicsSchemaTools.intToSdfPath(int(h.actor1))),h.num_contact_data) for h in headers if h.num_contact_data],flush=True)
   self.history.broken=True;self.remove_broken_joint=True;self.history.valid_break=all(self.ticks-self.contact_tick[n]<=120 for n in ('upper','lower'));print('JOINT_BREAK',str(path),'valid tool contacts',self.history.valid_break,'ages',self.ticks,{n:self.ticks-v for n,v in self.contact_tick.items()},'peaks',self.contact_peak,flush=True)
 def _contact_check(self):
  from pxr import PhysicsSchemaTools
  self.ticks+=1
  headers,contacts,_=self.physics.get_full_contact_report()
  self.contact_force={'upper':0.,'lower':0.}
  pair_count=0;pair_separation=None;foreign_now=False
  jaw_contacts=set()
  for h in headers:
   if not h.num_contact_data:continue
   paths=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.actor0,h.actor1)]
   names=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.collider0,h.collider1)]
   if any('RightArm' in p for p in paths) and any('/upper' in p or '/lower' in p for p in paths):self.robot_contacts.add(tuple(names))
   if any('Arm' in p for p in paths) and any(p.endswith('/shell_a') or p.endswith('/shell_b') for p in paths):self.contact_tick['robot_shell']=self.ticks
   shell_names={p.rsplit('/',1)[-1] for p in paths}&{'shell_a','shell_b'}
   data=contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]
   if shell_names and len(shell_names)!=2:
    other_names=[name.rsplit('/',1)[-1] for name in names if '/shell_' not in name]
    allowed={'UpperLip','LowerJaw','JawTooth0','JawTooth1','JawTooth2','FrontToothL','FrontToothR','JawGuideL','JawGuideR','HingeBoss'}
    # The head's underside overlaps the jaw plate in this rigid envelope.
    # Only its mouth-roof contacts are designated jaw assembly contacts.
    roof=all(-.040<=c.position[2]-HINGE[2]<=.039 and .005<=HINGE[0]-c.position[0]<=.090 and abs(c.position[1]-HINGE[1])<=.035 for c in data)
    if any(n not in allowed and not(n=='Head' and roof) for n in other_names):foreign_now=True
   # Normal load on the lower jaw assembly or table can support through
   # normal force or friction. Final stable-pose checks reject transient
   # grazing and airborne contacts; an upper-lip-only touch is insufficient.
   lower_surface=any(p.rsplit('/',1)[-1] in ('LowerJaw','JawTooth0','JawTooth1','JawTooth2','FrontToothL','FrontToothR','JawGuideL','JawGuideR','HingeBoss') for p in names)
   table_surface=any('table' in p.lower() for p in names)
   if len(shell_names)==2:
    pair_count+=len(data)
    minimum=min(float(c.separation) for c in data)
    pair_separation=minimum if pair_separation is None else min(pair_separation,minimum)
    if any(np.linalg.norm(c.impulse)*PHYSICS_HZ>.005 for c in data):self.shell_pair_tick=self.ticks
   if table_surface or lower_surface:
    for shell in shell_names:
     center=self.native.get_rigidbody_transformation('/World/envs/env_0/'+shell)['position']
     if any(np.linalg.norm(c.impulse)*PHYSICS_HZ>.005 for c in data):
      self.support_tick[shell]=self.ticks
      if lower_surface:self.lower_support_tick=self.ticks
   for n in ('upper','lower'):
    jaw_names=('UpperLip',) if n=='upper' else ('LowerJaw','JawTooth0','JawTooth1','JawTooth2','FrontToothL','FrontToothR')
    if any(p.rsplit('/',1)[-1] in jaw_names for p in names) and any(p.endswith('/'+n) for p in paths) and any(p.endswith('/shell_a') or p.endswith('/shell_b') for p in paths):
     self.contact_tick[n]=self.ticks; jaw_contacts.add(n)
     force=sum(np.linalg.norm(c.impulse) for c in contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data])*PHYSICS_HZ
     self.contact_peak[n]=max(self.contact_peak[n],float(force));self.contact_force[n]+=float(force)
  both=jaw_contacts=={'upper','lower'}
  if both:self.opposing_tick=self.ticks
  if foreign_now:self.foreign_tick=self.ticks
  from yam_arena.urdf import quat_xyzw_to_matrix
  rotation=quat_xyzw_to_matrix(self.native.get_rigidbody_transformation('/World/envs/env_0/lower')['rotation'])
  theta=math.atan2(rotation[2,1],-rotation[0,1])
  if both and self.last_both and not foreign_now and self.ticks-self.foreign_tick>120:
   self.closing_contact_angle=max(0.,self.closing_contact_angle+theta-self.last_hinge_angle)
  else:self.closing_contact_angle=0.
  if self.closing_contact_angle>=math.radians(1.):self.compression_tick=self.ticks
  self.last_hinge_angle=theta;self.last_both=both
  shell_states={n:self.native.get_rigidbody_transformation('/World/envs/env_0/'+n) for n in ('shell_a','shell_b')}
  c,s=math.cos(FIGURE_YAW),math.sin(FIGURE_YAW);yaw=np.array([[c,-s,0],[s,c,0],[0,0,1]])
  local_centers=[yaw.T@(np.array(v['position'])-HINGE) for v in shell_states.values()]
  custody=all(abs(v[0])<=.045 and -.005<=v[1]<=.105 and -.055<=v[2]<=.055 for v in local_centers)
  opening=rim_opening(np.array(shell_states['shell_a']['position']),quat_xyzw_to_matrix(shell_states['shell_a']['rotation']),np.array(shell_states['shell_b']['position']),quat_xyzw_to_matrix(shell_states['shell_b']['rotation']))
  self.rubric.observe(opening_m=opening,opposing_jaw_contact=both,
      robot_shell_contact=self.ticks-self.contact_tick['robot_shell']<=120,
      compression=self.closing_contact_angle>=math.radians(1.),mouth_custody=custody,
      foreign_contact=self.ticks-self.foreign_tick<=120)
  row={'tick':self.ticks,'broken':self.history.broken,'pair_contacts':pair_count,
      'pair_min_separation_m':pair_separation,'jaw_force_N':dict(self.contact_force),
      'opening_m':opening,'mouth_custody':custody,'compression_witness':self.rubric.witness_valid,'fracture_qualified':self.rubric.fracture_qualified,'visible_crack_qualified':self.rubric.visible_crack_qualified,
      'seam_prim_present':bool(self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/WalnutSeam')),
      'positions':{n:list(self.native.get_rigidbody_transformation('/World/envs/env_0/'+n)['position']) for n in ('shell_a','shell_b')}}
  if not self.history.broken:self.prefracture.append(row)
  elif self.postbreak_ticks<240:
   if not self.fracture_trace:self.fracture_trace=list(self.prefracture)
   self.fracture_trace.append(row);self.postbreak_ticks+=1
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
  seam=UsdPhysics.FixedJoint.Define(s,base+'WalnutSeam');seam.CreateBody0Rel().SetTargets([Sdf.Path(base+'shell_a')]);seam.CreateBody1Rel().SetTargets([Sdf.Path(base+'shell_b')]);seam.CreateLocalPos0Attr(Gf.Vec3f(0,-.008,0));seam.CreateLocalPos1Attr(Gf.Vec3f(0,.008,0));seam.CreateBreakForceAttr(float(os.environ.get('NUT_SEAM_FORCE','200.')));seam.CreateBreakTorqueAttr(float(os.environ.get('NUT_SEAM_TORQUE','5.')));seam.CreateExcludeFromArticulationAttr(True);seam.CreateCollisionEnabledAttr(False)
  UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath(base+'shell_a')).CreateFilteredPairsRel().SetTargets([Sdf.Path(base+'shell_b')])
 def reset(self,scene,*,seed=None):
  self._ensure_sim(None);self.active=False
  for n in ('NutcrackerHinge','WalnutSeam','MountPin','BenchPin','BenchMotor'):self._base.sim.stage.RemovePrim('/World/envs/env_0/'+n)
  self._instruction=scene.instruction;self._held=0;self._best_score=0.;self._env.reset()
  # Reset all four free roots before creating physical joints; no pose writes during a trial.
  for spec in self.task.objects:
   if spec.name=='fixture':continue
   obj=self._base.scene[spec.name];pose=obj.data.default_root_pose.torch.clone();pose[:,:3]=self._torch.tensor(spec.pos,dtype=pose.dtype,device=pose.device);pose[:,3:]=self._torch.tensor(([math.sqrt(.5),0.,0.,math.sqrt(.5)] if spec.name.startswith('shell_') else [0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2)]),dtype=pose.dtype,device=pose.device);obj.write_root_pose_to_sim_index(root_pose=pose);obj.write_root_velocity_to_sim_index(root_velocity=self._torch.zeros_like(obj.data.default_root_vel.torch))
  self.prefracture=deque(maxlen=120);self.fracture_trace=[];self.postbreak_ticks=0
  self.history=SimpleNamespace(score=0.,broken=False,valid_break=False,hold=0.,placement_hold=0.);self.remove_broken_joint=False;self.task.module.HISTORY=self.history;self.rubric=Rubric();self.rest_reference=None;self.boxes=envelopes();self.support_tick={'shell_a':-10000,'shell_b':-10000};self.shell_pair_tick=-10000;self.lower_support_tick=-10000;self.opposing_tick=-10000;self.foreign_tick=-10000;self.compression_tick=-10000;self.closing_contact_angle=0.;self.last_hinge_angle=0.;self.last_both=False;self.ticks=0;self.contact_tick={'upper':-10000,'lower':-10000,'robot_shell':-10000};self.contact_peak={'upper':0.,'lower':0.};self.contact_force={'upper':0.,'lower':0.};self.robot_contacts=set();self._joints();self.active=True
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
   arm,fingers=self._ids[key];q=arr(self._base.scene[key].data.joint_pos)[0];p,r=ik.tcp_pose(q[arm],station.BASES[side]);effort=getattr(self._base.scene[key].data,'applied_torque',None)
   hands[side]=dict(tcp=p,rotation=r,joints=q[arm],fingers=q[fingers],actuator_applied_torque_field=None if effort is None else arr(effort)[0].tolist())
  FEEDBACK.update(poses=poses,rotations=rotations,hands=hands);return poses,rotations
 def _task_step(self):
  if self.remove_broken_joint:
   self._remove_seam()
  poses,rot=self._read();d=np.linalg.norm(poses['shell_a']-poses['shell_b']);h=self.history
  # Native break state is diagnostic; observable first-fissure provenance qualifies scoring.
  clearance_poses=dict(poses,fixture=np.array([HINGE[0],HINGE[1],TABLE]))
  c,s=math.cos(FIGURE_YAW),math.sin(FIGURE_YAW);yaw=np.array([[c,-s,0],[s,c,0],[0,0,1]])
  clearance_rot=dict(rot,fixture=yaw)
  clear,clearance=grippers_clear(FEEDBACK['hands'],clearance_poses,clearance_rot,self.boxes)
  fixture_ok=fixture_stable(poses,rot,HINGE,yaw)
  support_ok=supported([self.ticks-self.support_tick[n] for n in ('shell_a','shell_b')],self.ticks-self.shell_pair_tick,whole_shell=not h.broken)
  # Rest on a side/front guide can be friction-supported. Contact direction
  # alone is insufficient; require a stable 2 mm / 2 degree envelope during
  # the final hold as well as actual contact with table/jaw or supported half.
  rest_stable=self.rest_reference is not None
  if self.rest_reference is not None:
   for n in ('shell_a','shell_b'):
    rp,rr=self.rest_reference[n]
    angle=math.acos(float(np.clip((np.trace(rr.T@rot[n])-1)/2,-1.,1.)))
    if np.linalg.norm(poses[n]-rp)>.002 or angle>math.radians(2.):rest_stable=False
  if not rest_stable:self.rest_reference={n:(poses[n].copy(),rot[n].copy()) for n in ('shell_a','shell_b')}
  on_table=table_bounds_ok([poses[n] for n in ('shell_a','shell_b')]) and support_ok and rest_stable
  rim_offset=rim_opening(poses['shell_a'],rot['shell_a'],poses['shell_b'],rot['shell_b'])
  # Seat checks use the visible mouth edges in the moving jaw frame. Contact
  # must be on an actual jaw collider, rather than a sleeve, hat or body.
  center=(poses['shell_a']+poses['shell_b'])/2
  local=rot['lower'].T@(center-poses['lower'])
  recent_lower=self.ticks-self.lower_support_tick<=24
  placed=abs(local[0])<=.022 and .025<=local[1]<=.065 and .010<=local[2]<=.040 and recent_lower
  opposing=self.ticks-self.opposing_tick<=120
  direct=self.ticks-self.contact_tick['robot_shell']<=120
  # Generous, visible mouth custody region in the fixed figure frame:
  # centres of both pieces within X±45, Y−5..105, Z−55..55 mm.
  mouth_custody=all(abs((yaw.T@(poses[n]-HINGE))[0])<=.045 and
      -.005<=(yaw.T@(poses[n]-HINGE))[1]<=.105 and
      -.055<=(yaw.T@(poses[n]-HINGE))[2]<=.055 for n in ('shell_a','shell_b'))
  compression=self.ticks-self.compression_tick<=24
  foreign=self.ticks-self.foreign_tick<=120
  h.score=self.rubric.update(opening_m=rim_offset,opposing_jaw_contact=opposing,
      robot_shell_contact=direct,seated=placed,fixture_stable=fixture_ok,
      contained=on_table,clear=clear,compression=compression,mouth_custody=mouth_custody,foreign_contact=foreign,observe_evidence=False)
  h.hold=self.rubric.crack_hold
  h.placement_hold=self.rubric.seat_hold
  FEEDBACK.update(robot_contacts=sorted(self.robot_contacts),contact_peak=dict(self.contact_peak),rim_offset=rim_offset,observable_fracture_qualified=self.rubric.fracture_qualified,visible_crack_qualified=self.rubric.visible_crack_qualified,compression=compression,mouth_custody=mouth_custody,foreign_contact=foreign,compression_witness=self.rubric.witness_valid,opposing_jaw_contact=opposing,robot_shell_contact=direct,lower_jaw_force=self.contact_force['lower'],upper_jaw_force=self.contact_force['upper'],lower_jaw_contact=self.ticks-self.lower_support_tick<=24,seat_local=local,minimum_gripper_clearance=clearance,fixture_stable=fixture_ok,support_ok=support_ok,rest_stable=rest_stable,placed=placed,separation=float(d),broken=h.broken,valid_break=h.valid_break,clear=clear,on_table=on_table,hold=h.hold,score=h.score)
  from inspect_robots.types import StepResult
  result=super()._task_step()
  return StepResult(observation=result.observation,terminated=False,info=result.info)
def make_embodiment(**kwargs):return NutcrackerEmbodiment(**kwargs)
