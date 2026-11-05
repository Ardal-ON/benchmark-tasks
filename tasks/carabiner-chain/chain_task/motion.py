"""Measured-body grasp targets and guarded Cartesian interpolation."""
from dataclasses import dataclass
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
@dataclass
class Target:
 seconds:float
 pos:np.ndarray
 rot:np.ndarray
 grip:float
 label:str
class Plan:
 def __init__(self,targets,side,q=None,grip=1.):
  self.targets=targets;self.base=station.BASES[side];self.q=np.array(READY if q is None else q);self.start_p,self.start_r=ik.tcp_pose(self.q,self.base);self.start_g=grip;self.duration=sum(t.seconds for t in targets);self.worst_position_error=0.;self.worst_rotation_error=0.
 def action7(self,time):
  p0,r0,g0=self.start_p,self.start_r,self.start_g;label='done'
  for t in self.targets:
   if time<t.seconds:
    s=ik.smoothstep(time/t.seconds);p=p0+s*(t.pos-p0);r=ik.slerp_matrix(r0,t.rot,s);g=g0+s*(t.grip-g0);label=t.label;break
   time-=t.seconds;p0,r0,g0=t.pos,t.rot,t.grip
  else:p,r,g=p0,r0,g0
  self.q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),self.q,base_pos=self.base,iters=80);self.worst_position_error=max(pe,self.worst_position_error);self.worst_rotation_error=max(re,self.worst_rotation_error)
  if pe>.002 or re>.02:raise RuntimeError('IK guard '+label+f' position={pe} rotation={re}')
  return np.r_[self.q,g],label

def grasp(p,r,flip=0):
 s=1 if r[2,2]>0 else -1
 x=r[:,0];z=np.array([0.,0.,-1.]);y=np.cross(z,x);tool=np.column_stack([x,y,z])@np.diag([-1.,-1.,1.]) if flip else np.column_stack([x,y,z])
 tcp=p+r@np.array([0.,-s*.036,0.])+np.array([0,0,.0125]);return tcp,tool

def pickup(p,r,flip=0):
 tcp,tool=grasp(p,r,flip)
 return [Target(3,tcp+[0,0,.075],tool,.55,'above loose carabiner'),Target(2,tcp,tool,.55,'descend bottom arch'),Target(1,tcp,tool,0.,'pinch bottom arch'),Target(3,tcp+[0,0,.12],tool,0.,'lift carabiner'),Target(2,tcp+[0,0,.12],tool,0.,'observe physical grasp')]
