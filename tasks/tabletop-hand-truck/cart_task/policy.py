"""Ordinary robot targets tip, roll, stand, then release the compact trolley."""
import numpy as np,math
from inspect_robots.policy import PolicyBase,PolicyInfo
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.urdf import axis_angle_matrix
from .config import *
class CartPolicy(PolicyBase):
 def __init__(self):self.info=PolicyInfo(name='cart_scripted',action_space=action_box(np.r_[ik.LOWER,0,ik.LOWER,0],np.r_[ik.UPPER,1,ik.UPPER,1]))
 def bind(self,info):pass
 def reset(self,scene):self.q=Q0.copy();self.bias=np.zeros(3);self.rbias=np.zeros(3)
 def act(self,obs):
  t=float(obs.state_time)+.05;grip=.22
  if t<1:a=0.;travel=0.;phase='hold'
  elif t<3:a=12*ik.smoothstep((t-1)/2);travel=0.;phase='lift nose onto wheels'
  elif t<7:a=12.;travel=.15*ik.smoothstep((t-3)/4);phase='roll'
  else:a=12*(1-ik.smoothstep((t-7)/2));travel=.15;phase='stand'
  rot=ry(-math.radians(a));p=AXLE-np.array([travel,0,0])+rot@GRASP;R=rot@GRIP_ROT
  if t>=8.2:grip=.22+.78*ik.smoothstep((t-8.2)/.8);phase='release'
  if t>=9.5:
   p-=ik.smoothstep(t-9.5)*.06*GRIP_ROT[:,2];phase='withdraw'
  if t>=10.5:p+=ik.smoothstep((t-10.5)/2)*np.array([0.,.24,-.06]);phase='clear'
  if 3<t<9:p[2]-=.002*ik.smoothstep((t-3)/.5)*(1-ik.smoothstep((t-8.2)/.8))
  measured=np.asarray(obs.state['joint_pos']);ap,ar=ik.tcp_pose(measured[:6],BASES['left'])
  if t<8.2:
   self.bias=np.clip(self.bias+.05*(p-ap),-.012,.012);self.rbias=np.clip(self.rbias+.05*ik._rot_error(R,ar),-.08,.08)
  else:self.bias*=.9;self.rbias*=.9
  p+=self.bias;ang=np.linalg.norm(self.rbias)
  if ang>1e-8:R=axis_angle_matrix(self.rbias/ang,ang)@R
  q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),self.q,base_pos=BASES['left'],iters=180)
  if pe>.005 or re>.04:raise RuntimeError(f'IK {phase} {pe} {re}')
  self.q=q;state=np.asarray(obs.state['joint_pos']);right=state[7:13]
  return ActionChunk(actions=[Action(data=np.r_[q,grip,right,1.],meta={'phase':phase})],control_hz=20.)
def make_policy(**kw):return CartPolicy(**kw)
