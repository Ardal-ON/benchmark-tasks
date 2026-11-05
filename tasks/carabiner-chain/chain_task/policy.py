"""Initial quiet/reset probe; physical clipping controller follows preflight."""
import numpy as np
from yam_arena.constants import READY
from yam_arena.scripted import YamScriptedPolicy
from inspect_robots.types import Action,ActionChunk
class HoldPolicy(YamScriptedPolicy):
 def reset(self,scene):pass
 def act(self,observation):return ActionChunk(actions=[Action(data=np.r_[READY,1.,READY,1.],meta={'request_stop':True} if observation.state_time>=20 else {})],control_hz=1/self._dt)
def make_hold_policy(**kwargs):return HoldPolicy(**kwargs)

class FirstClipPolicy(YamScriptedPolicy):
 """One-link physical milestone. No second/third assembly is scripted yet."""
 def reset(self,scene):
  self.phase='start';self.plan=None;self.start=0.;self.integral=np.zeros(14);self.stop_at=None
 def begin(self,phase,targets,t,f):
  from chain_task.motion import Plan
  hand=f['hands']['right'];self.plan=Plan(targets,'right',hand['joints'],float(-np.mean(hand['fingers'])/.04695));self.start=t;self.end=t+self.plan.duration;self.phase=phase;self.integral[:]=0.;print('CHAIN_POLICY',phase,t,flush=True)
 def body_target(self,seconds,p,r,label,grip=0.):
  from chain_task.motion import Target
  return Target(seconds,p+r@self.offset,r@self.relative.T,grip,label)
 def act(self,observation):
  from chain_task.embodiment import FEEDBACK as f
  from chain_task.motion import pickup,Target
  from chain_task.geometry import rx,rz,TABLE
  from yam_arena import ik,station
  t=float(observation.state_time)+self._dt
  if self.phase=='start':
   p,r=f['poses']['link2'],f['rotations']['link2'];self.before=p.copy();self.face=1 if r[2,2]>0 else -1;self.begin('pickup',pickup(p,r),t,f)
  if t>=self.end:
   p,r=f['poses']['link2'],f['rotations']['link2'];hand=f['hands']['right'];tcp=np.array(hand['tcp']);tool=np.array(hand['rotation'])
   if self.phase=='pickup':
    if p[2]-self.before[2]<.08:raise RuntimeError('Physical bottom-arch grasp did not lift link2')
    self.offset=r.T@(tcp-p);self.relative=tool.T@r;self.desired=rz(np.deg2rad(-10))@rx(self.face*np.pi/2);self.pre=np.array([.42,0,TABLE+.2225-self.face*.005])-self.desired@np.array([.022,0,0]);self.begin('orient',[self.body_target(4,np.array([.34,-.16,.94]),self.desired,'orient loose link'),self.body_target(3,self.pre+[0,-.09,0],self.desired,'approach eye from side'),self.body_target(4,self.pre,self.desired,'align gate outside eye'),self.body_target(2,self.pre,self.desired,'settle alignment')],t,f)
   elif self.phase=='orient':
    # Re-measure the ordinary grasp to compensate contact slip; never move a body.
    self.offset=r.T@(tcp-p);self.relative=tool.T@r;end=self.pre+self.desired@np.array([.028,0,0]);self.begin('clip',[self.body_target(6,end,self.desired,'push eye rod through passive gate'),self.body_target(3,end,self.desired,'allow spring closure')],t,f)
   elif self.phase=='clip':
    self.begin('release',[Target(2,tcp,tool,.6,'open fingers'),Target(3,tcp-tool[:,2]*.09,tool,.6,'withdraw from hanging link'),Target(4,*ik.tcp_pose(np.array(READY),station.BASES['right']),1.,'return right ready'),Target(12,*ik.tcp_pose(np.array(READY),station.BASES['right']),1.,'released hanging observation')],t,f)
   elif self.phase=='release':self.stop_at=t;self.end=t+100.;self.phase='done';print('CHAIN_MILESTONE_RESULT',f.get('score'),f.get('score_reasons'),flush=True)
  a,label=self.plan.action7(t-self.start);target=np.r_[READY,1.,a];actual=np.asarray(observation.state['joint_pos']);self.integral+=1.5*self._dt*(target-actual);self.integral[[6,13]]=0.;self.integral=np.clip(self.integral,-.25,.25)
  return ActionChunk(actions=[Action(data=target+self.integral,meta={'request_stop':True} if self.stop_at is not None else {})],control_hz=1/self._dt)

def make_first_clip_policy(**kwargs):return FirstClipPolicy(**kwargs)
