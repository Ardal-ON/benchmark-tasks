"""Physical loose-bolt pickup/insertion milestone using ordinary YAM actions."""
from dataclasses import replace
import numpy as np
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.constants import READY
from yam_arena import ik,station
from inspect_robots.types import Action,ActionChunk
from ratchet_task.config import *
from ratchet_task.motion import Plan,Target,rz
class BoltStartPolicy(YamScriptedPolicy):
    def __init__(self,**kwargs):
        super().__init__(**kwargs);self.info=replace(self.info,name='confined_bolt_start_scripted')
    def reset(self,scene):
        self.phase='start';self.plan=None;self.integral=np.zeros(14);self.stop=False;self.side='left';self.wait_until=None
    def begin(self,phase,targets,t,f):
        hand=f['hands'][self.side]
        closed=self.plan is not None and self.plan.targets[-1].grip==0. and targets[0].grip==0.
        grip=0. if closed else float(-np.mean(hand['fingers'])/.04695)
        self.plan=Plan(targets,self.side,hand['joints'],grip);self.phase=phase;self.start=t;self.end=t+self.plan.duration;self.integral[:]=0.
        print('BOLT_POLICY',phase,t,flush=True)
    def body_target(self,seconds,p,r,label,grip=0.):
        return Target(seconds,p+r@self.offset,r@self.relative.T,grip,label)
    def act(self,observation):
        from ratchet_task.embodiment import FEEDBACK as f
        t=float(observation.state_time)+self._dt
        if self.phase=='start':
            b=f['poses']['bolt'];self.initial_bolt=b.copy();r=np.diag([-1.,1.,-1.]);p=b+np.array([0,0,.01415])
            self.begin('pickup',[Target(2.5,p+[0,0,.075],r,.35,'above bolt'),Target(2,p,r,.35,'descend to bolt head'),Target(1,p,r,0.,'pinch bolt head'),Target(2.5,p+[0,0,.10],r,0.,'lift loose bolt'),Target(1,p+[0,0,.10],r,0.,'verify lift')],t,f)
        if t>=self.end:
            b=f['poses']['bolt'];br=f['rotations']['bolt'];hand=f['hands'][self.side]
            if self.phase=='pickup':
                if b[2]-self.initial_bolt[2]<.06:raise RuntimeError('Bolt head grasp failed to lift the loose bolt')
                self.offset=br.T@(hand['tcp']-b);self.relative=hand['rotation'].T@br
                self.bolt_rotation=rz(float(np.arctan2(br[1,0],br[0,0])))
                self.begin('align',[self.body_target(3,AXIS+[0,0,.035],self.bolt_rotation,'align bolt above bore'),self.body_target(1,AXIS+[0,0,.035],self.bolt_rotation,'settle bolt alignment')],t,f)
            elif self.phase=='align':
                self.offset=br.T@(hand['tcp']-b);self.relative=hand['rotation'].T@br
                self.begin('insert',[self.body_target(3,AXIS+[0,0,.0145],self.bolt_rotation,'insert bolt tip'),self.body_target(1,AXIS+[0,0,.0145],self.bolt_rotation,'settle inserted tip')],t,f)
            elif self.phase=='insert':
                tip=b-br[:,2]*BOLT_LENGTH
                if np.linalg.norm(tip[:2]-AXIS[:2])>.0008 or AXIS[2]-tip[2]<.0007:raise RuntimeError('Physical tip insertion missed the bore')
                self.offset=br.T@(hand['tcp']-b);self.relative=hand['rotation'].T@br
                r=rz(-np.pi/3)@self.bolt_rotation
                self.begin('thread',[self.body_target(3,b+[0,0,-PITCH/6],r,'clockwise thread start'),self.body_target(2,b+[0,0,-PITCH/6],r,'hold started thread')],t,f)
            elif self.phase=='thread':
                if not f.get('hardware',{}).get('captured',False):raise RuntimeError('Bolt did not physically establish threaded engagement')
                p,r=hand['tcp'],hand['rotation']
                ready_p,ready_r=ik.tcp_pose(np.array(READY),station.BASES['left'])
                self.begin('release',[Target(1,p,r,1.,'open after thread capture'),Target(2,p+[0,0,.07],r,1.,'withdraw from bolt'),Target(3,ready_p,ready_r,1.,'left arm ready'),Target(2,ready_p,ready_r,1.,'observe released started bolt')],t,f)
            elif self.phase=='release':
                self.stop=True;self.phase='done';self.end=t+1000.
                print('BOLT_START_COMPLETE',f.get('hardware'),flush=True)
        a,label=self.plan.action7(t-self.start)
        target=np.r_[a,READY,1.]
        actual=np.asarray(observation.state['joint_pos'])
        self.integral+=self._dt*(target-actual);self.integral[[6,13]]=0;self.integral=np.clip(self.integral,-.1,.1)
        return ActionChunk(actions=[Action(data=target+self.integral,meta={'request_stop':True} if self.stop else {})],control_hz=1/self._dt)
def make_bolt_start_policy(**kwargs):return BoltStartPolicy(**kwargs)
