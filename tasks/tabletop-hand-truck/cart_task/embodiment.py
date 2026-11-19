"""Full-size floor cart, calibrated arms/cameras, and a reduced robot platform."""
from pathlib import Path
from dataclasses import replace
import hashlib,math,numpy as np
from yam_arena import ik,station
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix
from .config import *
from .gravity import target as gravity_target
from .scoring import rolling_complete,stage_score,pose_stable,load_bearing_contact,supported_motion,support_lost
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
  old_spawn,old_cb,old_robot,old_room=scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment,scene._station_room
  def spawn(spec):
   if spec.name=='cart':return Object(name='cart',usd_path=str(ROOT/'assets/cart.usda'),object_type=ObjectType.ARTICULATION,initial_pose=Pose(position_xyz=spec.pos,rotation_xyzw=FRAME_QUAT),spawn_cfg_addon=dict(collision_props=sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),rigid_props=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=64,solver_velocity_iteration_count=8,max_depenetration_velocity=.2)))
   if spec.name=='goal_mark':return Object(name=spec.name,object_type=ObjectType.BASE,spawner_cfg=sim.CuboidCfg(size=spec.size,visual_material=sim.PreviewSurfaceCfg(diffuse_color=spec.color)),initial_pose=Pose(position_xyz=spec.pos))
   return old_spawn(spec)
  def room():
   result=[]
   for obj in old_room():
    if obj.name=='table':result.extend(scene._box(name,size,pos,scene.STATION_TABLE_COLOR,cls=scene._Table) for name,(size,pos) in PLATFORM_PARTS.items())
    else:result.append(obj)
   return result
  def callback(hz=None):
   parent=old_cb(hz)
   def cb(cfg):
    cfg=parent(cfg);cfg.sim.dt=1/480;cfg.decimation=24;cfg.sim.render_interval=24;cfg.sim.physics.friction_offset_threshold=.001;cfg.sim.physics.friction_correlation_distance=.0005;cfg.episode_length_s=35.;return cfg
   return cb
  class ReadyRobot(old_robot):
   def __init__(robot,**opts):
    opts['robot_contact_offset']=.0002;super().__init__(**opts)
    for side in ('left','right'):
     cfg=getattr(robot.scene_config,side+'_robot');cfg.init_state.joint_pos={**{f'joint{i+1}':float(v) for i,v in enumerate(Q0[side])},'joint[7-8]':(-.020 if side=='left' else -.032)}
  scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment,scene._station_room=spawn,callback,ReadyRobot,room
  try:super()._ensure_sim(goal)
  finally:scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment,scene._station_room=old_spawn,old_cb,old_robot,old_room
  import omni.physx
  from pxr import PhysxSchema
  self.native=omni.physx.get_physx_interface();self.physics=omni.physx.get_physx_simulation_interface();self._base.sim.set_setting('/physics/disableContactProcessing',False);self.subscription=self.physics.subscribe_full_contact_report_events(lambda *args:None)
  for n in ('cart/frame','cart/wheel_left','cart/wheel_right'):PhysxSchema.PhysxContactReportAPI.Apply(self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+n)).CreateThresholdAttr(0.)
  raw=self._base.sim.step
  def tick(*args,**kw):
   ans=raw(*args,**kw)
   if self.active:self.observe()
   return ans
  self._base.sim.step=tick
 def fault(self,reason):
  self.failed=True
  if not any(x['reason']==reason for x in self.faults):self.faults.append(dict(reason=reason,tick=self.tick))
 def pose(self,n):
  v=self.native.get_rigidbody_transformation('/World/envs/env_0/'+n);return np.array(v['position']),quat_xyzw_to_matrix(np.array(v['rotation']))
 def observe(self):
  from pxr import PhysicsSchemaTools
  self.tick+=1;p,R=self.pose('cart/frame')
  headers,contacts,_=self.physics.get_full_contact_report()
  for h in headers:
   if not h.num_contact_data:continue
   colliders=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.collider0,h.collider1)]
   names=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.actor0,h.actor1)];robot=any('/LeftArm/' in n or '/RightArm/' in n for n in names)
   if robot and any('/cart/' in n for n in names):
    self.touch=self.tick
    for key in self.grip:
     side,tip=key.split(':')
     if any('/'+side.title()+'Arm/' in n and n.endswith('/'+tip) for n in names):
      self.grip[key]=self.tick;self.grip_contacts[key]=[dict(position=list(c.position),normal=list(c.normal),impulse=str(c.impulse),colliders=colliders) for c in contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]]
   if robot and any(set(n.split('/')) & {'table','table_left_pad','table_right_pad','camera_housing','stand_mast','stand_arm','stand_foot'} for n in names):
    self.robot_station_contacts[' | '.join(names)]=dict(tick=self.tick,actors=names,contacts=[dict(position=list(c.position),normal=list(c.normal),impulse=str(c.impulse),colliders=colliders) for c in contacts[h.contact_data_offset:h.contact_data_offset+min(h.num_contact_data,4)]])
   if any('/cart/' in n for n in names):
    key=' | '.join(names)
    self.other_contacts[key]=dict(tick=self.tick,actors=names,contacts=[dict(position=list(c.position),normal=list(c.normal),impulse=str(c.impulse),colliders=colliders) for c in contacts[h.contact_data_offset:h.contact_data_offset+min(h.num_contact_data,8)]])
    if any(set(n.split('/')) & {'camera_housing','stand_mast','stand_arm','stand_foot','table','table_left_pad','table_right_pad'} for n in names):self.fault('cart_station_hardware_contact')
   if any('/cart/frame/toe_' in path for path in colliders) and any('/floor/' in n or n.endswith('/floor') for n in names):
    if any(load_bearing_contact(float(c.separation),list(c.impulse),list(c.normal)) for c in contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]):self.toe_support=self.tick
   for n in self.support:
    if any(v.endswith('/'+n) for v in names) and any('/floor/' in v or v.endswith('/floor') for v in names):
     active_contacts=contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]
     if any(load_bearing_contact(float(c.separation),list(c.impulse),list(c.normal)) for c in active_contacts):self.support[n]=self.tick;self.support_seen[n]=True
     self.contacts[n]=[dict(position=list(c.position),normal=list(c.normal),impulse=str(c.impulse),separation=float(c.separation),actors=names) for c in contacts[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]]
  distance_delta={};turn_delta={}
  for n in self.turn:
   wp,W=self.pose('cart/'+n);angle=math.atan2(W[0,2],W[2,2])
   turn_delta[n]=0. if n not in self.previous else ROLL_DIRECTION*math.atan2(math.sin(angle-self.previous[n]),math.cos(angle-self.previous[n]))
   self.turn[n]+=turn_delta[n];self.previous[n]=angle
   if n not in self.start:self.start[n]=wp[0]
   self.distance[n]=ROLL_DIRECTION*(wp[0]-self.start[n])
   distance_delta[n]=0. if n not in self.last_wp else ROLL_DIRECTION*(wp[0]-self.last_wp[n][0])
   self.last_wp[n]=wp
   if wp[2]>GROUND+RADIUS+.020 or wp[2]<GROUND+RADIUS-.008:self.fault('wheel_height')
  sd,st=supported_motion({n:(self.tick-v if self.support_seen[n] else 10000) for n,v in self.support.items()},distance_delta,turn_delta)
  for n in self.turn:self.supported_distance[n]+=sd[n];self.supported_turn[n]+=st[n]
  if support_lost({n:self.tick-v for n,v in self.support.items()},self.tick-self.toe_support,math.acos(float(np.clip(R[2,2],-1,1)))<math.radians(2)):self.fault('wheel_support_loss')
 def reset(self,scene,*,seed=None):
  self.active=False;self._ensure_sim(None);self._ready=self._torch.tensor([[*gravity_target(Q_HOLD['left']),GRIP_ACTION['left'],*gravity_target(Q_HOLD['right']),GRIP_ACTION['right']]],device=self._base.device,dtype=self._torch.float32)
  from yam_arena import inspect_embodiment as base
  old_settle=base.SETTLE_S;base.SETTLE_S=2.
  try:obs=super().reset(scene,seed=seed)
  finally:base.SETTLE_S=old_settle
  self.failed=False;self.faults=[];self.toe_support=-10000;self.score=0.;self.tick=0;self.touch=-10000;self.grip={side+':'+tip:-10000 for side in ('left','right') for tip in ('tip_left','tip_right')};self.grip_contacts={};self.other_contacts={};self.robot_station_contacts={};self.support={n:0 for n in ('wheel_left','wheel_right')};self.contacts={n:[] for n in self.support};self.support_seen=dict.fromkeys(self.support,False);self.turn={n:0. for n in self.support};self.distance=dict(self.turn);self.supported_distance=dict(self.turn);self.supported_turn=dict(self.turn);self.last_wp={};self.previous={};self.start={};self.trace=[];self.tipped=False;self.hold=0;FEEDBACK.clear();self.active=True
  rp,rR=self.pose('cart/frame');wa=np.mean([self.pose('cart/'+n)[0] for n in self.turn],axis=0)
  FEEDBACK.update(frame=rp.tolist(),frame_rotation=rR.tolist(),axle=wa.tolist(),angle_deg=math.degrees(math.acos(float(np.clip(rR[2,2],-1,1)))))
  from runtime_audit import write_reset
  write_reset(self,obs,['cart'])
  return obs
 def close(self):
  from runtime_audit import write_episode
  write_episode(self)
  return super().close()
 def _task_step(self):
  from inspect_robots.types import StepResult
  p,R=self.pose('cart/frame');angle=math.acos(float(np.clip(R[2,2],-1,1)));toe=p+R@np.array([0.,-NOSE_DEPTH,NOSE_TOP]);self.tipped=self.tipped or (angle>math.radians(8) and toe[2]>GROUND+.008)
  wheels=[self.pose('cart/'+n)[0] for n in self.turn];axle=np.mean(wheels,axis=0);rolling=rolling_complete(self.supported_distance,self.distance,self.supported_turn,RADIUS)
  tcps=[]
  for side in ('left','right'):
   ids=self._ids[side+'_robot'][0];q=self._base.scene[side+'_robot'].data.joint_pos.torch[0,ids].cpu().numpy();tcp,_=ik.tcp_pose(q,BASES[side]);tcps.append(tcp)
  clear=self.tick-self.touch>240 and all(np.linalg.norm(t-(p+R@gp))>.12 for t in tcps for gp in GRIP_LOCAL.values())
  stable=not self.trace or (pose_stable(self.trace[-1]['frame'],p,self.trace[-1]['frame_rotation'],R))
  self.score,self.hold=stage_score(failed=self.failed,tipped=self.tipped,rolling=rolling,axle=axle,angle=angle,clear=clear,stable=stable,hold=self.hold)
  row=dict(t=self._t,score=self.score,failed=self.failed,faults=list(self.faults),frame=p.tolist(),angle_deg=math.degrees(angle),frame_rotation=R.tolist(),toe_height=toe[2]-GROUND,toe_support_age=self.tick-self.toe_support,axle=axle.tolist(),wheel_contact=self.contacts.copy(),all_cart_contacts=self.other_contacts.copy(),robot_station_contacts=self.robot_station_contacts.copy(),wheel_joint_pos=self._base.scene['cart'].data.joint_pos.torch[0].cpu().numpy().tolist(),wheel_joint_vel=self._base.scene['cart'].data.joint_vel.torch[0].cpu().numpy().tolist(),wheel_turn=dict(self.turn),wheel_distance=dict(self.distance),supported_distance=dict(self.supported_distance),supported_turn=dict(self.supported_turn),support_age={n:self.tick-v for n,v in self.support.items()},tipped=self.tipped,grip_age={n:self.tick-v for n,v in self.grip.items()},grip_contacts=self.grip_contacts.copy(),grippers_clear=clear,hold=self.hold,tcps=[t.tolist() for t in tcps]);self.trace.append(row);FEEDBACK.clear();FEEDBACK.update(row)
  return StepResult(observation=self._observation(),terminated=self.failed or self.score==1.,termination_reason='success' if self.score==1. else ('invalid' if self.failed else None),info={'score':self.score,'success':self.score==1.})
def make_embodiment(**kw):return CartEmbodiment(**kw)
