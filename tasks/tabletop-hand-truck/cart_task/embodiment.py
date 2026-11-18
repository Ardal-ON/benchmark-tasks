"""Product-derived miniature cart on the unchanged calibrated YAM station."""
from pathlib import Path
from dataclasses import replace
import hashlib,math,numpy as np
from yam_arena import ik,station
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix
from .config import *
from .scoring import rolling_complete,stage_score
ROOT=Path(__file__).resolve().parents[1];FEEDBACK={}
class CartEmbodiment(YamArenaEmbodiment):
 def __init__(self,**kw):
  for k,v in dict(task=str(ROOT/'task.py'),device='cpu',control_hz=20,cam_width=640,cam_height=360,object_jitter=0.).items():kw.setdefault(k,v)
  super().__init__(**kw)
  assert self.rig==station.rig('calibrated')
  self.info=replace(self.info,name='yam_tabletop_cart',environment_revision=hashlib.sha256(b''.join(p.relative_to(ROOT).as_posix().encode()+b'\0'+p.read_bytes() for p in sorted(ROOT.rglob('*')) if p.is_file() and (p.suffix in ('.py','.usda','.png','.toml') or p.name=='physical_design.json'))).hexdigest());self.active=False;self.score=0.;self.trace=[]
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
  old_spawn,old_cb,old_robot=scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment
  def spawn(spec):
   if spec.name=='cart':return Object(name='cart',usd_path=str(ROOT/'assets/cart.usda'),object_type=ObjectType.ARTICULATION,initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=FRAME_QUAT),spawn_cfg_addon=dict(collision_props=sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),rigid_props=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=64,solver_velocity_iteration_count=8,max_depenetration_velocity=.2)))
   if spec.name=='goal_mark':return Object(name=spec.name,object_type=ObjectType.BASE,spawner_cfg=sim.CuboidCfg(size=spec.size,visual_material=sim.PreviewSurfaceCfg(diffuse_color=spec.color)),initial_pose=Pose(position_xyz=spec.pos))
   if spec.name=='load':
    obj=old_spawn(spec);return Object(name=spec.name,object_type=ObjectType.RIGID,spawner_cfg=obj.spawner_cfg,initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=FRAME_QUAT))
   return old_spawn(spec)
  def callback(hz=None):
   parent=old_cb(hz)
   def cb(cfg):
    cfg=parent(cfg);cfg.sim.dt=1/480;cfg.decimation=24;cfg.sim.render_interval=24;cfg.sim.physics.friction_offset_threshold=.001;cfg.sim.physics.friction_correlation_distance=.0005;cfg.episode_length_s=35.;return cfg
   return cb
  class ReadyRobot(old_robot):
   def __init__(robot,**opts):
    opts['robot_contact_offset']=.0002;super().__init__(**opts)
    cfg=robot.scene_config.left_robot;cfg.init_state.joint_pos={**{f'joint{i+1}':float(v) for i,v in enumerate(Q0)},'joint[7-8]':-.015}
  scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=spawn,callback,ReadyRobot
  try:super()._ensure_sim(goal)
  finally:scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=old_spawn,old_cb,old_robot
  import omni.physx
  from pxr import PhysxSchema
  self.native=omni.physx.get_physx_interface();self.physics=omni.physx.get_physx_simulation_interface();self._base.sim.set_setting('/physics/disableContactProcessing',False);self.subscription=self.physics.subscribe_full_contact_report_events(lambda *args:None)
  for n in ('cart/frame','cart/wheel_left','cart/wheel_right','load'):PhysxSchema.PhysxContactReportAPI.Apply(self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+n)).CreateThresholdAttr(0.)
  raw=self._base.sim.step
  def tick(*args,**kw):
   ans=raw(*args,**kw)
   if self.active:self.observe()
   return ans
  self._base.sim.step=tick
 def pose(self,n):
  v=self.native.get_rigidbody_transformation('/World/envs/env_0/'+n);return np.array(v['position']),quat_xyzw_to_matrix(np.array(v['rotation']))
 def observe(self):
  from pxr import PhysicsSchemaTools
  self.tick+=1;p,R=self.pose('cart/frame');lp,LR=self.pose('load');local=R.T@(lp-p);rel=math.acos(float(np.clip((np.trace(R.T@LR)-1)/2,-1,1)))
  if np.max(np.abs(local-LOAD_LOCAL))>.025 or rel>math.radians(15.):self.failed=True
  headers,contacts,_=self.physics.get_full_contact_report()
  for h in headers:
   if not h.num_contact_data:continue
   names=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.actor0,h.actor1)];robot=any('/LeftArm/' in n or '/RightArm/' in n for n in names)
   if robot and any('/cart/' in n for n in names):
    self.touch=self.tick
    for tip in self.grip:
     if any('/LeftArm/' in n and n.endswith('/'+tip) for n in names):self.grip[tip]=self.tick
   if robot and any(n.endswith('/load') for n in names):self.failed=True
   for n in self.support:
    if any(v.endswith('/'+n) for v in names) and any('/table/' in v or v.endswith('/table') for v in names):
     self.support[n]=self.tick;self.contacts[n]=[dict(position=list(c.position),normal=list(c.normal),impulse=str(c.impulse),separation=float(c.separation),actors=names) for c in contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]]
  for n in self.turn:
   wp,W=self.pose('cart/'+n);angle=math.atan2(W[0,2],W[2,2])
   if n in self.previous:self.turn[n]-=math.atan2(math.sin(angle-self.previous[n]),math.cos(angle-self.previous[n]))
   self.previous[n]=angle
   if n not in self.start:self.start[n]=wp[0]
   self.distance[n]=self.start[n]-wp[0]
   if n in self.last_wp and self.tick-self.support[n]<=4:self.supported_distance[n]+=self.last_wp[n][0]-wp[0]
   self.last_wp[n]=wp
   if wp[2]>TABLE+RADIUS+.020 or wp[2]<TABLE+RADIUS-.008:self.failed=True
  if all(self.tick-v>120 for v in self.support.values()):self.failed=True
 def reset(self,scene,*,seed=None):
  self.active=False;self._ensure_sim(None);self._ready=self._torch.tensor([[*Q0,.22,*self._ready[0,7:].cpu().numpy()]],device=self._base.device,dtype=self._torch.float32)
  obs=super().reset(scene,seed=seed);self.failed=False;self.score=0.;self.tick=0;self.touch=-10000;self.grip={n:-10000 for n in ('tip_left','tip_right')};self.support={n:0 for n in ('wheel_left','wheel_right')};self.contacts={n:[] for n in self.support};self.turn={n:0. for n in self.support};self.distance=dict(self.turn);self.supported_distance=dict(self.turn);self.last_wp={};self.previous={};self.start={};self.trace=[];self.tipped=False;self.hold=0;FEEDBACK.clear();self.active=True
  from runtime_audit import write_reset
  write_reset(self,obs,['cart', 'load'])
  return obs
 def close(self):
  from runtime_audit import write_episode
  write_episode(self)
  return super().close()
 def _task_step(self):
  from inspect_robots.types import StepResult
  p,R=self.pose('cart/frame');lp,LR=self.pose('load');angle=math.acos(float(np.clip(R[2,2],-1,1)));toe=p+R@np.array([0.,-.046875,.002]);self.tipped=self.tipped or (angle>math.radians(8) and toe[2]>TABLE+.008)
  wheels=[self.pose('cart/'+n)[0] for n in self.turn];axle=np.mean(wheels,axis=0);rolling=rolling_complete(self.supported_distance,self.distance,self.turn,RADIUS)
  tcps=[]
  for side in ('left','right'):
   ids=self._ids[side+'_robot'][0];q=self._base.scene[side+'_robot'].data.joint_pos.torch[0,ids].cpu().numpy();tcp,_=ik.tcp_pose(q,BASES[side]);tcps.append(tcp)
  clear=self.tick-self.touch>240 and all(np.linalg.norm(t-(p+R@np.array([0.,0.,.296])))>.12 for t in tcps)
  stable=not self.trace or (np.linalg.norm(p-np.array(self.trace[-1]['frame']))*20<.01 and np.linalg.norm(lp-np.array(self.trace[-1]['load']))*20<.01)
  self.score,self.hold=stage_score(failed=self.failed,tipped=self.tipped,rolling=rolling,axle=axle,angle=angle,clear=clear,stable=stable,hold=self.hold)
  row=dict(t=self._t,score=self.score,failed=self.failed,frame=p.tolist(),load=lp.tolist(),angle_deg=math.degrees(angle),frame_rotation=R.tolist(),toe_height=toe[2]-TABLE,axle=axle.tolist(),wheel_contact=self.contacts.copy(),wheel_joint_pos=self._base.scene['cart'].data.joint_pos.torch[0].cpu().numpy().tolist(),wheel_joint_vel=self._base.scene['cart'].data.joint_vel.torch[0].cpu().numpy().tolist(),wheel_turn=dict(self.turn),wheel_distance=dict(self.distance),supported_distance=dict(self.supported_distance),support_age={n:self.tick-v for n,v in self.support.items()},tipped=self.tipped,grip_age={n:self.tick-v for n,v in self.grip.items()},grippers_clear=clear,hold=self.hold,tcps=[t.tolist() for t in tcps]);self.trace.append(row);FEEDBACK.clear();FEEDBACK.update(row)
  return StepResult(observation=self._observation(),terminated=self.failed or self.score==1.,termination_reason='success' if self.score==1. else ('invalid' if self.failed else None),info={'score':self.score,'success':self.score==1.})
def make_embodiment(**kw):return CartEmbodiment(**kw)
