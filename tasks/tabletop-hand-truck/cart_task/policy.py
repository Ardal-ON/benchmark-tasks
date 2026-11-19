"""Contact-driven joint targets with measured phase-boundary grasp calibration."""
import math
import numpy as np
from inspect_robots.policy import PolicyBase,PolicyInfo
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.urdf import axis_angle_matrix
from .config import AXLE,BASES,GRASPS,GRIP_ROTS,Q0,Q_HOLD,TCP_PRELOAD_Z,GRIP_ACTION,rx,ry
from .embodiment import FEEDBACK
from . import gravity

class CartPolicy(PolicyBase):
    def __init__(self):
        self.info=PolicyInfo(name='cart_scripted',action_space=action_box(
            np.r_[ik.LOWER,0,ik.LOWER,0],np.r_[ik.UPPER,1,ik.UPPER,1]))
    def bind(self,info):pass
    def reset(self,scene):
        self.q={s:gravity.target(Q_HOLD[s]) for s in Q0}
        self.bias={s:np.zeros(3) for s in Q0}
        self.rbias={s:np.zeros(3) for s in Q0}
        self.preload={s:np.zeros(6) for s in Q0}
        self.roll_bias=0.;self.down_bias=0.;self.phase_start=0.
        self.phase='tip';self.release_time=None;self.release_pose={};self.release_command={};self.trace=[]
    def acquire(self,measured,phase):
        """Preserve joint commands while expressing the grasp at its actual pose."""
        self.anchor=np.asarray(FEEDBACK['axle'])
        self.frame=np.asarray(FEEDBACK['frame_rotation'])
        self.relative_p={};self.relative_R={}
        for side,offset in [('left',0),('right',7)]:
            actual=measured[offset:offset+6]
            p,R=ik.tcp_pose(actual,BASES[side])
            self.relative_p[side]=self.frame.T@(p-self.anchor)
            self.relative_R[side]=self.frame.T@R
            self.preload[side]=self.q[side]-actual-gravity.offset(actual)
            self.q[side]=actual.copy()
            self.bias[side][:]=0.;self.rbias[side][:]=0.
        self.pitch=math.atan2(-self.frame[0,2],self.frame[2,2])
        self.phase=phase;self.roll_bias=0.;self.down_bias=0.
    def act(self,obs):
        t=float(obs.state_time)+.05;measured=np.asarray(obs.state['joint_pos'])
        level_error=-float(FEEDBACK['frame_rotation'][2][0])
        ages=FEEDBACK.get('support_age',dict.fromkeys(('wheel_left','wheel_right'),0))
        balance=int(ages['wheel_left']>8 and ages['wheel_right']<=8)-int(ages['wheel_right']>8 and ages['wheel_left']<=8)
        self.roll_bias=float(np.clip(self.roll_bias+(.20*level_error if abs(level_error)>.003 else 0.)+.002*balance,-.08,.08))
        if all(v>8 for v in ages.values()):self.down_bias=min(.02,self.down_bias+.002)
        level_correction=float(np.clip(self.roll_bias+3.*level_error,-.10,.10))
        if t>=5 and self.phase=='tip' and abs(level_error)<.002 and all(v<=4 for v in FEEDBACK.get('support_age',{}).values()):
            self.acquire(measured,'pull');self.phase_start=t;level_correction=0.
        if self.phase=='pull' and t>=self.phase_start+8:
            self.acquire(measured,'stand');self.phase_start=t;level_correction=0.
        if self.release_time is None and self.phase=='stand' and t>self.phase_start+4 and FEEDBACK.get('angle_deg',180.)<2.:
            self.release_time=t
        released=self.release_time is not None
        age=0. if not released else t-self.release_time
        grip=0. if not released else ik.smoothstep(age/.6)
        if self.phase=='tip':
            angle=18.*ik.smoothstep(t/4)
            rot=ry(-math.radians(angle));origin=AXLE+np.array([0,0,TCP_PRELOAD_Z])
        elif self.phase=='pull':
            rot=self.frame
            direction=self.frame[:,1].copy();direction[2]=0.
            origin=self.anchor+direction*((.335-self.anchor[0])/direction[0])*ik.smoothstep((t-self.phase_start)/8)
        else:
            delta=self.pitch*ik.smoothstep((t-self.phase_start)/6)
            if t>self.phase_start+6:
                R=np.asarray(FEEDBACK['frame_rotation'])
                delta+=float(np.clip(math.atan2(-R[0,2],R[2,2]),0,math.radians(3)))
            rot=self.frame@rx(delta);origin=self.anchor+np.array([(.335-self.anchor[0])*ik.smoothstep((t-self.phase_start)/6),0,0])
        actions=[]
        for side,offset,sign in [('left',0,1),('right',7,-1)]:
            grip=GRIP_ACTION[side]+(1-GRIP_ACTION[side])*(0. if not released else ik.smoothstep(age/.6))
            actual_p,actual_R=ik.tcp_pose(measured[offset:offset+6],BASES[side])
            if self.phase=='tip':p=origin+rot@GRASPS[side];R=rot@GRIP_ROTS[side]
            else:p=origin+rot@self.relative_p[side];R=rot@self.relative_R[side]
            phase=self.phase
            if released:
                if side not in self.release_command:self.release_command[side]=self.q[side].copy()
                if age<.8:
                    p,R=actual_p.copy(),actual_R.copy();phase='open_hold'
                else:
                    if side not in self.release_pose:self.release_pose[side]=(actual_p.copy(),actual_R.copy())
                    p,R=(v.copy() for v in self.release_pose[side]);phase='release'
                    if age>=1.0:p+=ik.smoothstep((age-1.)/1.5)*np.array([-.16,0,0]);phase='withdraw'
            else:
                p[1]-=sign*level_correction*.233
                p[2]-=self.down_bias
                cap=.012 if self.phase=='tip' else .006
                gain=.05 if self.phase=='tip' else .03
                self.bias[side]=np.clip(self.bias[side]+gain*(p-actual_p),-cap,cap)
                self.rbias[side]=np.clip(self.rbias[side]+gain*ik._rot_error(R,actual_R),-.08,.08)
                p+=self.bias[side]
                norm=np.linalg.norm(self.rbias[side])
                if norm>1e-8:R=axis_angle_matrix(self.rbias[side]/norm,norm)@R
            q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),self.q[side],base_pos=BASES[side],iters=180)
            if pe>.005 or re>.15:raise RuntimeError(f'{side} unreachable target: {pe}, {re}')
            if released:
                q=self.release_command[side].copy() if age<.8 else gravity.target(q)
            else:q=np.clip(q+gravity.offset(q)+self.preload[side],ik.LOWER,ik.UPPER)
            if not np.all(np.isfinite(q)):raise RuntimeError('Nonfinite joint target')
            cp,cR=ik.tcp_pose(q,BASES[side])
            self.trace.append(dict(t=t,side=side,phase=phase,ik_position_error=pe,ik_rotation_error=re,
                command_position_error=float(np.linalg.norm(cp-p)),command_rotation_error=float(np.linalg.norm(ik._rot_error(R,cR))),
                target_tcp=p.tolist(),actual_tcp=actual_p.tolist(),actual_q=measured[offset:offset+6].tolist(),command_q=q.tolist(),
                preload_joint_rad=self.preload[side].tolist(),actual_gripper=float(measured[offset+6]),command_gripper=grip))
            self.q[side]=q;actions.extend([*q,grip])
        return ActionChunk(actions=[Action(data=np.asarray(actions),meta={'phase':phase})],control_hz=20.)
    def transcript(self):return {'control_diagnostics':self.trace}
def make_policy(**kw):return CartPolicy(**kw)
