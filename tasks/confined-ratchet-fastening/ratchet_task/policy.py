"""Reference ratcheting demo using normal joint/gripper actions only."""
from dataclasses import replace
import numpy as np
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.constants import READY
from yam_arena import ik,station
from inspect_robots.types import Action,ActionChunk
from ratchet_task.config import *
from ratchet_task.motion import Plan,Target,rz
class FastenPolicy(YamScriptedPolicy):
    def __init__(self,**kwargs):
        super().__init__(**kwargs);self.info=replace(self.info,name='confined_ratchet_scripted')
    def reset(self,scene):
        self.phase='start';self.plan=None;self.integral=np.zeros(14);self.stop=False;self.last_report=-100.
    def begin(self,phase,targets,t,f):
        hand=f['hands']['right'];grip=float(np.clip(-np.mean(hand['fingers'])/.04695,0,1))
        self.plan=Plan(targets,'right',hand['joints'],grip);self.phase=phase;self.start=t;self.end=t+self.plan.duration;self.integral[:]=0.
        print('ROBOT_POLICY',phase,t,flush=True)
    def calibrate(self,f):
        h=f['poses']['handle'];hr=f['rotations']['handle'];hand=f['hands']['right']
        self.offset=hr.T@(hand['tcp']-h);self.relative=hand['rotation'].T@hr
    def tool_plan(self,phase,goals,t,f):
        self.begin(phase,[Target(seconds,np.asarray(p)+r@self.offset,r@self.relative.T,grip,label) for seconds,p,r,label,grip in goals],t,f)
    def pack(self,a,label,observation):
        target=np.r_[READY,1.,a];actual=np.asarray(observation.state['joint_pos'])
        self.integral+=self._dt*(target-actual);self.integral[[6,13]]=0.;self.integral=np.clip(self.integral,-.1,.1)
        return ActionChunk(actions=[Action(data=target+self.integral,meta={'request_stop':True,'waypoint':label} if self.stop else {'waypoint':label})],control_hz=1/self._dt)
    def act(self,observation):
        from ratchet_task.embodiment import FEEDBACK as f
        t=float(observation.state_time)+self._dt;hand=f['hands']['right'];h=f['poses']['handle'];b=f['poses']['bolt'];hw=f.get('hardware',{})
        if self.phase=='start':
            self.calibrate(f);self.phase='ratchet';self.stroke_start=t;self.stroke_from=float(hw.get('angles',{}).get('handle',0.));self.stroke_to=np.deg2rad(14);self.stroke_duration=1.;self.command_q=hand['joints'].copy();self.missing=0.
        if self.phase in ['ratchet','hold']:
            if f.get('tightened',False):
                self.calibrate(f);angle=float(hw['angles']['handle']);clear=min(np.deg2rad(14),angle+np.deg2rad(3));p=h.copy()
                self.tool_plan('remove',[(1.,p,rz(clear),'unload ratchet',0.),(2.,p+[0,0,.025],rz(clear),'lift socket clear',0.),(2.,p+[0,0,.025],np.eye(3),'straighten for exit',0.),(4.,[.20,0,p[2]+.025],np.eye(3),'withdraw through opening',0.)],t,f)
            else:
                seated=-.0003<=hw['gap']<=.0002 and hw['turns']>=REQUIRED_TURNS
                if seated and self.phase=='ratchet':self.phase='hold';self.hold_angle=float(hw['angles']['handle'])
                if self.phase=='hold':theta=self.hold_angle
                else:
                    elapsed=t-self.stroke_start
                    if elapsed>=self.stroke_duration:
                        self.stroke_from=self.stroke_to;self.stroke_to=-self.stroke_to;self.stroke_start=t;elapsed=0.
                        self.stroke_duration=2. if self.stroke_to<0 and hw['gap']<.0005 else 1.
                    theta=self.stroke_from+(self.stroke_to-self.stroke_from)*ik.smoothstep(elapsed/self.stroke_duration)
                # Control the socket mouth, accounting for the small live tilt.
                root=b+np.array([0,0,.0005])+.028*f['rotations']['drive'][:,2]
                r=rz(theta);p=root+r@self.offset;rotation=r@self.relative.T
                self.command_q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rotation),self.command_q,base_pos=station.BASES['right'],iters=100)
                if pe>.002 or re>.02:raise RuntimeError(f'Ratchet IK: {pe}, {re}')
                self.missing=self.missing+self._dt if not hw['socket_engaged'] else 0.
                if self.missing>3.:raise RuntimeError('Socket lost engagement')
                if t-self.last_report>=10.:
                    print('RATCHET_PROGRESS',t,hw['turns'],hw['gap'],hw.get('socket_torque_nm',0.),hw['ratchet_cycles'],flush=True);self.last_report=t
                return self.pack(np.r_[self.command_q,0.],self.phase,observation)
        if t>=self.end:
            if self.phase=='remove':
                self.calibrate(f);park=TOOL_PARK+[0,0,.002]
                self.tool_plan('park',[(5.,[park[0],park[1],TABLE+.080],np.eye(3),'return to empty cradle',0.),(3.,park,np.eye(3),'lower onto cradle',0.),(1.5,park,np.eye(3),'release ratchet',1.)],t,f)
            elif self.phase=='park':
                p,r=hand['tcp'],hand['rotation'];rp,rr=ik.tcp_pose(np.array(READY),station.BASES['right'])
                self.begin('release',[Target(2,p+[0,0,.08],r,1.,'withdraw open fingers'),Target(3,rp,rr,1.,'right arm ready'),Target(4,rp,rr,1.,'released observation')],t,f)
            elif self.phase=='release':
                if f.get('score',0.)!=1.:raise RuntimeError('Released state did not reach full score')
                self.stop=True;self.end=t+1000.
        a,label=self.plan.action7(t-self.start)
        return self.pack(a,label,observation)
def make_fasten_policy(**kwargs):return FastenPolicy(**kwargs)
