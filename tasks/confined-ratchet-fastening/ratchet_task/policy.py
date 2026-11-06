"""Reference ratcheting demo using normal joint/gripper actions only."""
from dataclasses import replace
import numpy as np
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.constants import READY
from yam_arena import ik,station
from inspect_robots.types import Action,ActionChunk
from ratchet_task.config import *
from ratchet_task.motion import rz
class FastenPolicy(YamScriptedPolicy):
    def __init__(self,**kwargs):
        super().__init__(**kwargs);self.info=replace(self.info,name='confined_ratchet_scripted')
    def reset(self,scene):
        self.phase='start';self.integral=np.zeros(14);self.stop=False;self.last_report=-100.
    def act(self,observation):
        from ratchet_task.embodiment import FEEDBACK as f
        t=float(observation.state_time)+self._dt;hand=f['hands']['right'];h=f['poses']['handle'];b=f['poses']['bolt'];hw=f['hardware']
        if self.phase=='start':
            hr=f['rotations']['handle'];self.offset=hr.T@(hand['tcp']-h);self.relative=hand['rotation'].T@hr
            self.phase='ratchet';self.stroke_start=t;self.stroke_from=float(hw['angles']['handle']);self.stroke_to=np.deg2rad(14);self.stroke_duration=1.;self.command_q=hand['joints'].copy();self.missing=0.
        seated=-.0003<=hw['gap']<=.0002 and hw['turns']>=REQUIRED_TURNS
        if seated and self.phase=='ratchet':self.phase='hold';self.hold_angle=float(hw['angles']['handle'])
        if self.phase=='hold':theta=self.hold_angle
        else:
            elapsed=t-self.stroke_start
            if elapsed>=self.stroke_duration:
                self.stroke_from=self.stroke_to;self.stroke_to=-self.stroke_to;self.stroke_start=t;elapsed=0.
                self.stroke_duration=2. if self.stroke_to<0 and hw['gap']<.0005 else 1.
            theta=self.stroke_from+(self.stroke_to-self.stroke_from)*ik.smoothstep(elapsed/self.stroke_duration)
        # Control the socket mouth, accounting for its small live tilt.
        root=b+np.array([0,0,.0005])+.028*f['rotations']['drive'][:,2]
        r=rz(theta);p=root+r@self.offset;rotation=r@self.relative.T
        self.command_q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rotation),self.command_q,base_pos=station.BASES['right'],iters=100)
        if pe>.002 or re>.02:raise RuntimeError(f'Ratchet IK: {pe}, {re}')
        self.missing=self.missing+self._dt if not hw['socket_engaged'] else 0.
        if self.missing>3.:raise RuntimeError('Socket lost engagement')
        if t-self.last_report>=10.:
            print('RATCHET_PROGRESS',t,hw['turns'],hw['gap'],hw.get('socket_torque_nm',0.),hw['ratchet_cycles'],flush=True);self.last_report=t
        target=np.r_[READY,1.,self.command_q,0.];actual=np.asarray(observation.state['joint_pos'])
        self.integral+=self._dt*(target-actual);self.integral[[6,13]]=0.;self.integral=np.clip(self.integral,-.1,.1)
        return ActionChunk(actions=[Action(data=target+self.integral,meta={'waypoint':self.phase})],control_hz=1/self._dt)
def make_fasten_policy(**kwargs):return FastenPolicy(**kwargs)
