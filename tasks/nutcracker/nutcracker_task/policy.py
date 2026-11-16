"""Two-arm contact-only figure demo: lift the rear lever, load walnut, press the lever."""
from dataclasses import replace
import math
import numpy as np
from inspect_robots.types import Action,ActionChunk
from yam_arena.scripted import YamScriptedPolicy
from yam_arena import ik,station
from yam_arena.demo import grip_rotation
from yam_arena.constants import READY
from nutcracker_task.config import *
from nutcracker_task.motion import Plan,Target
RZ=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
RIGHT_R=np.array([[0.,1.,0.],[0.,0.,1.],[1.,0.,0.]])
LEFT_R=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
def rx(a):
 c,s=np.cos(a),np.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
class NutcrackerPolicy(YamScriptedPolicy):
 def __init__(self,**kwargs):super().__init__(**kwargs);self.info=replace(self.info,name='nutcracker_scripted')
 def reset(self,scene):
  self.stop=False;self.phase='lever_approach';self.start=0.;self.integral=np.zeros(14);self.left_q=np.array(READY);self.right_q=np.array(READY);self.left_grip=1.;self.right_grip=1.;self.started=False;self.opened=False;self.loaded=False;self.pressed=False
 def begin(self,phase,targets,t,f,side):
  h=f['hands'][side];g=self.left_grip if side=='left' else self.right_grip
  self.plan=Plan(targets,side,h['joints'],g);self.plan_side=side;self.phase=phase;self.start=t;self.end=t+self.plan.duration;self.integral[:7 if side=='left' else 0]=0.;self.integral[7 if side=='right' else 14:]=0.;print('PHASE',phase,round(t,2),flush=True)
 def pack(self,left,right,obs,label):
  self.left_grip=float(left[6]);self.right_grip=float(right[6]);target=np.r_[left,right];actual=np.asarray(obs.state['joint_pos']);self.integral=np.clip(self.integral+self._dt*(target-actual),-.08,.08);self.integral[[6,13]]=0
  return ActionChunk(actions=[Action(data=target+self.integral,meta={'phase':self.phase,'waypoint':label})],control_hz=1/self._dt)
 def act(self,obs):
  from nutcracker_task.embodiment import FEEDBACK as f
  t=float(obs.state_time)+self._dt;left=np.r_[self.left_q,self.left_grip];right=np.r_[self.right_q,self.right_grip]
  hand=lambda: f['poses']['lower']+f['rotations']['lower']@LEVER_LOCAL
  center=lambda:(f['poses']['shell_a']+f['poses']['shell_b'])/2
  if not self.started:
   self.started=True;p=hand();self.approach_from=f['hands']['right']['joints'].copy();self.approach_q,pe,re=ik.solve(p+[0,-.03,0],ik.matrix_to_quat_xyzw(RIGHT_R),self.approach_from,base_pos=station.BASES['right'],iters=500)
   if pe>.003 or re>.03:raise RuntimeError(f'Hand pregrasp unreachable: {pe:.4f}, {re:.4f}')
   self.phase='joint_approach';self.start=t;print('PHASE joint_approach',round(t,2),flush=True)
  if self.phase=='joint_approach':
   if t-self.start<4.:
    u=ik.smoothstep((t-self.start)/4.);self.right_q=self.approach_from+u*(self.approach_q-self.approach_from)
    return self.pack(left,np.r_[self.right_q,1.],obs,'approach rear lever')
   p=hand();self.begin('lever_approach',[Target(3.,p,RIGHT_R,1.,'surround lever'),Target(2.,p,RIGHT_R,0.,'grasp rear lever')],t,f,'right')
  if self.phase=='lever_approach' and t>=self.end:
   gap=-float(np.sum(f['hands']['right']['fingers']))
   if gap<.012:raise RuntimeError(f'Empty lever grasp: {gap:.4f} m')
   self.phase='lift_lever';self.start=t;self.theta=0.;self.hand_offset=f['hands']['right']['tcp']-hand();self.right_q=f['hands']['right']['joints'].copy();print('PHASE lift_lever',round(t,2),flush=True)
  if self.phase in ('lift_lever','load_nut','clamp_nut','reposition','reopen','push_nut','press_lever'):
   if self.phase=='lift_lever':self.theta=max(OPEN_ANGLE,self.theta-.06*self._dt)
   if self.phase=='press_lever':self.theta=min(.02,self.theta+.12*self._dt)
   if self.phase=='clamp_nut':
    self.theta=min(.02,self.theta+.08*self._dt)
    left[6]=min(.45,max(0.,(t-self.start)*.12))
   if self.phase=='reopen':self.theta=max(OPEN_ANGLE,self.theta-.08*self._dt)
   p=HINGE+RZ@(JAW_PIVOT+rx(self.theta)@(LEVER_LOCAL-JAW_PIVOT))+(RZ@rx(self.theta)@RZ.T)@self.hand_offset
   self.right_q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw((RZ@rx(self.theta*.5)@RZ.T)@RIGHT_R),self.right_q,base_pos=station.BASES['right'],iters=120)
   if pe>.004 or re>.04:raise RuntimeError(f'Hand arc IK failed: {pe:.4f}, {re:.4f}')
   right=np.r_[self.right_q,0.]
   if self.phase=='lift_lever' and self.theta<=OPEN_ANGLE:
    measured=RZ.T@f['rotations']['lower'];angle=math.atan2(measured[2,1],measured[1,1])
    if angle>-.40 and t-self.start<18.:return self.pack(left,right,obs,'hold lever up')
    print('OPEN_ANGLE',angle,flush=True)
    if angle>-.40:raise RuntimeError('Rear lever did not open mouth sufficiently')
    self.theta=angle
    self.hand_offset=(RZ@rx(-angle)@RZ.T)@(f['hands']['right']['tcp']-hand())
    self.opened=True;p=center()+[-.009,0,.010];down=grip_rotation(0.)
    self.begin('load_nut',[Target(4.,p+[0,0,.10],down,1.,'approach walnut'),Target(3.,p,down,1.,'surround walnut'),Target(2.,p,down,0.,'grasp walnut'),Target(3.,p+[0,0,.10],down,0.,'lift walnut'),Target(3.,p+[0,0,.10],LEFT_R,0.,'turn walnut toward mouth'),Target(4.,NUT_BENCH+[-.080,0,.010],LEFT_R,0.,'align in front of mouth'),Target(4.,NUT_BENCH+[-.004,.012,-.008],LEFT_R,0.,'partially insert walnut')],t,f,'left')
   if self.phase=='load_nut':
    if t<self.end:
     left,label=self.plan.action7(t-self.start);self.left_q=left[:6];return self.pack(left,right,obs,label)
    p=center();print('NUT_PARTIAL',p.tolist(),flush=True)
    if f.get('broken'):raise RuntimeError('Walnut broke before jaw squeeze')
    if np.linalg.norm(p-NUT_BENCH)>.055:raise RuntimeError('Walnut not at mouth')
    p=f['hands']['left']['tcp'];self.begin('reposition',[Target(4.,p+[-.07,0,0],LEFT_R,1.,'release and move behind walnut'),Target(1.,p+[-.07,0,0],LEFT_R,0.,'prepare to push')],t,f,'left')
   if self.phase=='clamp_nut' and self.theta>=.02:
    measured=RZ.T@f['rotations']['lower'];actual=math.atan2(measured[2,1],measured[1,1])
    if actual<-.18 and t-self.start<16.:return self.pack(left,right,obs,'wait for jaw to hold walnut')
    print('CLAMP_ANGLE',actual,flush=True)
    if actual<-.18:raise RuntimeError('Jaw did not close enough to hold walnut')
    p=f['hands']['left']['tcp'];self.begin('reposition',[Target(2.,p,LEFT_R,1.,'release held walnut'),Target(3.,p+[-.07,0,0],LEFT_R,1.,'move behind walnut'),Target(1.,p+[-.07,0,0],LEFT_R,0.,'prepare to push')],t,f,'left')
   if self.phase=='reposition':
    if t<self.end:
     left,label=self.plan.action7(t-self.start);self.left_q=left[:6];return self.pack(left,right,obs,label)
    self.phase='reopen';self.start=t;print('PHASE reopen',round(t,2),flush=True)
   if self.phase=='reopen' and self.theta<=OPEN_ANGLE:
    p=center();print('NUT_BEFORE_PUSH',p.tolist(),flush=True)
    q=f['hands']['left']['tcp'];self.begin('push_nut',[Target(4.,q+[.080,0,0],LEFT_R,0.,'push walnut into mouth'),Target(2.,q+[.080,0,0],LEFT_R,0.,'hold walnut in place')],t,f,'left')
   if self.phase=='push_nut':
    if t<self.end:
     left,label=self.plan.action7(t-self.start);self.left_q=left[:6];return self.pack(left,right,obs,label)
    p=center();print('NUT_AFTER_LOAD',p.tolist(),'goal',NUT_BENCH.tolist(),flush=True)
    if np.linalg.norm(p-NUT_BENCH)>.040:raise RuntimeError('Walnut not in figure mouth')
    self.loaded=True;q=f['hands']['right']['tcp'];self.begin('release_for_regrasp',[Target(1.,q,RIGHT_R,1.,'release lever'),Target(2.,q+[0,-.04,.025],RIGHT_R,1.,'clear moving lever'),Target(2.,q+[0,-.04,.025],RIGHT_R,1.,'let lever settle')],t,f,'right')
   if self.phase=='press_lever' and (self.theta>=.02 or f.get('broken')):
    if not f.get('broken') and t-self.start<12.:return self.pack(left,right,obs,'hold pressure on lever')
    self.pressed=True;p=f['hands']['right']['tcp'];self.begin('release_lever',[Target(1.,p,RIGHT_R,1.,'release rear lever'),Target(3.,p+[0,-.08,0],RIGHT_R,1.,'withdraw from lever')],t,f,'right')
   if self.phase in ('lift_lever','clamp_nut','reopen','press_lever'):return self.pack(left,right,obs,self.phase)
  if self.phase=='release_for_regrasp' and t>=self.end:
   p=hand()+[0.,-.005,0.];down=RIGHT_R;self.begin('press_from_above',[Target(2.,p+[0,0,.04],down,0.,'position above lever'),Target(5.,p+[.06,0,-.06],down,0.,'press lever down'),Target(2.,p+[.06,0,-.06],down,0.,'hold pressure'),Target(2.,p+[.06,0,.04],down,1.,'release lever')],t,f,'right')
   lp=f['hands']['left']['tcp'];self.clear_plan=Plan([Target(3.,lp+[-.06,0,.015],LEFT_R,0.,'withdraw from mouth'),Target(1.,lp+[-.06,0,.015],LEFT_R,1.,'open clear of mouth')],'left',f['hands']['left']['joints'],self.left_grip)
  if self.phase=='regrasp' and t>=self.end:
   measured=RZ.T@f['rotations']['lower'];angle=math.atan2(measured[2,1],measured[1,1]);self.theta=angle
   self.hand_offset=(RZ@rx(-angle)@RZ.T)@(f['hands']['right']['tcp']-hand())
   p=hand()+[0.,-.005,0.];down=RIGHT_R;self.begin('press_from_above',[Target(2.,p+[0,0,.04],down,0.,'position above lever'),Target(5.,p+[.06,0,-.06],down,0.,'press lever down'),Target(2.,p+[.06,0,-.06],down,0.,'hold pressure'),Target(2.,p+[.06,0,.04],down,1.,'release lever')],t,f,'right')
   lp=f['hands']['left']['tcp'];self.clear_plan=Plan([Target(3.,lp+[-.06,0,.015],LEFT_R,0.,'withdraw from mouth'),Target(1.,lp+[-.06,0,.015],LEFT_R,1.,'open clear of mouth')],'left',f['hands']['left']['joints'],self.left_grip)
  if self.phase=='press_from_above' and t<self.end:
   right,label=self.plan.action7(t-self.start);self.right_q=right[:6]
   if t-self.start>=3.:
    left,_=self.clear_plan.action7(t-self.start-3.);self.left_q=left[:6]
   return self.pack(left,right,obs,label)
  if self.phase=='press_from_above' and t>=self.end:
   self.phase='finish';self.start=t;self.l0=f['hands']['left']['joints'].copy();self.r0=f['hands']['right']['joints'].copy();print('PHASE finish',round(t,2),flush=True)
  if self.phase=='release_lever' and t>=self.end:
   self.phase='finish';self.start=t;self.l0=f['hands']['left']['joints'].copy();self.r0=f['hands']['right']['joints'].copy();print('PHASE finish',round(t,2),flush=True)
  if self.phase=='finish':
   u=ik.smoothstep((t-self.start)/4.);self.stop=t-self.start>8.
   return self.pack(np.r_[self.l0+u*(np.array(READY)-self.l0),1.],np.r_[self.r0+u*(np.array(READY)-self.r0),1.],obs,'park arms')
  action,label=self.plan.action7(t-self.start)
  if self.plan_side=='left':self.left_q=action[:6];left=action
  else:self.right_q=action[:6];right=action
  return self.pack(left,right,obs,label)
def make_policy(**kwargs):return NutcrackerPolicy(**kwargs)
