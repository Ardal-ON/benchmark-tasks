"""Pose-feedback brush sweep; only robot joint targets are commanded."""
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN as O,FINGER_CLOSED as C
from yam_arena.demo import Waypoint,ArmPlan,grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import quat_xyzw_to_matrix
from inspect_robots_yam.packing import split

def make_plan(top):
 def w(t,x,y,z,f,label):return Waypoint(t,(x,y,top+z),f,label)
 left=[w(2,.14,.27,.16,O,'approach'),w(1.5,.14,.27,.064,O,'lower'),w(1,.14,.27,.064,C,'grasp'),
 w(1.5,.14,.27,.12,C,'lift'),w(2,.309,0,.12,C,'carry'),w(2,.309,0,.039,C,'lower brush'),
 w(1,.309,0,.039,C,'align'),w(6,.11,0,.039,C,'sweep'),w(1,.11,0,.039,C,'settle chips'),
 w(2,.11,0,.12,C,'lift between passes'),w(2,.24,-.025,.12,C,'carry for edge pass'),
 w(2,.24,-.025,.039,C,'lower for edge pass'),w(4,.09,-.025,.039,C,'sweep edge'),
 w(1,.09,-.025,.039,C,'settle edge'),w(2,.09,-.025,.12,C,'lift clear'),w(2,.14,.27,.12,C,'return'),w(2,.14,.27,.064,C,'park'),
 w(1,.14,.27,.064,O,'release'),w(2,.14,.27,.16,O,'retreat')]
 right=[w(sum(p.duration for p in left),.18,-.25,.20,O,'wait')]
 return {'left':left,'right':right}

class FeedbackPlan(ArmPlan):
 def target(self,t):
  pos,rot,finger,label=super().target(t)
  from chip_sweep.embodiment import DEMO_STATES
  if self.side=='left' and 'brush' in DEMO_STATES and label not in ('approach','lower','grasp','release','retreat','done'):
   tool=DEMO_STATES['brush'];tcp,actual_rot=self.actual_pose;tr=quat_xyzw_to_matrix(tool.quat_xyzw)
   correction=tr.T@actual_rot@grip_rotation().T
   self.correction=ik.slerp_matrix(getattr(self,'correction',np.eye(3)),correction,.12)
   rot=self.correction@grip_rotation()
   desired_center=pos+(.08875,0,-.0085)
   pos=desired_center+tcp-tool.pos
   self.last=(pos.copy(),rot.copy())
  elif self.side=='left' and hasattr(self,'last'):
   if label=='release':pos,rot=self.last
   elif label=='retreat':
    previous,rot=self.last;pos=pos+previous-(.14,.27,self.top+.064)
  return pos,rot,finger,label
class ChipPolicy(YamScriptedPolicy):
 def reset(self,scene):
  plans=self.plan_for(scene.instruction,scene.target)
  self._plans={side:FeedbackPlan(list(ws),self._bases[side]) for side,ws in plans.items()}
  for side,p in self._plans.items():p.side=side;p.top=self._task.table_top;p.actual_pose=ik.tcp_pose(np.asarray(READY),self._bases[side])
  self._end=max(p.duration for p in self._plans.values())
 def act(self,obs):
  for side,q in zip(('left','right'),split(obs.state['joint_pos'])):self._plans[side].actual_pose=ik.tcp_pose(q[:6],self._bases[side])
  return super().act(obs)
def make_policy(**kwargs):return ChipPolicy(**kwargs)
