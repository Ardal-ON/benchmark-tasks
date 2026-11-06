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
        self.phase='start';self.plan=None;self.integral=np.zeros(14);self.stop=False;self.side='left';self.wait_until=None;self.insert_stable=0.;self.insert_waits=0;self.thread_retries=0
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
        if self.phase=='insert':
            hw=f.get('hardware',{});b=f['poses']['bolt'];br=f['rotations']['bolt'];depth=THREAD_ENTRY_Z-(b-br[:,2]*BOLT_LENGTH)[2]
            good=hw.get('capture_eligible',False) and hw.get('radial_error',1)<.0006 and .0011<depth<.0028
            self.insert_stable=self.insert_stable+self._dt if good else 0.
            if t>=self.end and self.insert_stable<.25:
                self.insert_waits+=1
                if self.insert_waits>5:raise RuntimeError('Bolt entry pose did not stabilize')
                self.end=t+2.;print('BOLT_WAIT_GEOMETRY',depth,hw.get('radial_error'),flush=True)
        if self.phase=='start':
            b=f['poses']['bolt'];self.initial_bolt=b.copy();r=np.diag([-1.,1.,-1.]);p=b+np.array([0,0,.0175])
            self.begin('pickup',[Target(2.5,p+[0,0,.075],r,.35,'above bolt'),Target(2,p,r,.35,'descend to bolt head'),Target(1,p,r,0.,'pinch bolt head'),Target(2.5,p+[0,0,.10],r,0.,'lift loose bolt'),Target(1,p+[0,0,.10],r,0.,'verify lift')],t,f)
        if t>=self.end:
            b=f['poses']['bolt'];br=f['rotations']['bolt'];hand=f['hands'][self.side]
            if self.phase=='pickup':
                if b[2]-self.initial_bolt[2]<.06:raise RuntimeError('Bolt head grasp failed to lift the loose bolt')
                self.offset=br.T@(hand['tcp']-b);self.relative=hand['rotation'].T@br
                self.bolt_rotation=rz(float(np.arctan2(br[1,0],br[0,0])))
                self.begin('align',[self.body_target(3,np.array([AXIS[0],.12,AXIS[2]+.030]),self.bolt_rotation,'carry outside wall'),self.body_target(1.5,np.array([AXIS[0],.12,AXIS[2]+.020]),self.bolt_rotation,'lower outside wall'),self.body_target(3,AXIS+[0,0,.020],self.bolt_rotation,'approach bore from open side'),self.body_target(1,AXIS+[0,0,.020],self.bolt_rotation,'settle bolt alignment')],t,f)
            elif self.phase=='align':
                self.offset=br.T@(hand['tcp']-b)
                self.begin('insert',[self.body_target(3,AXIS+[0,0,START_GAP],self.bolt_rotation,'insert bolt tip'),self.body_target(1,AXIS+[0,0,START_GAP],self.bolt_rotation,'settle inserted tip')],t,f)
            elif self.phase=='insert':
                tip=b-br[:,2]*BOLT_LENGTH
                if np.linalg.norm(tip[:2]-AXIS[:2])>.0008 or AXIS[2]-tip[2]<.0007:raise RuntimeError('Physical tip insertion missed the bore')
                self.offset=br.T@(hand['tcp']-b)
                r=rz(-np.pi/3)@self.bolt_rotation
                self.begin('thread',[self.body_target(5,np.array([AXIS[0],AXIS[1],AXIS[2]+START_GAP-PITCH/6]),r,'clockwise thread start'),self.body_target(2,np.array([AXIS[0],AXIS[1],AXIS[2]+START_GAP-PITCH/6]),r,'hold started thread')],t,f)
            elif self.phase=='thread':
                if not f.get('hardware',{}).get('captured',False):raise RuntimeError('Bolt did not physically establish threaded engagement')
                p,r=hand['tcp'],hand['rotation']
                ready_p,ready_r=ik.tcp_pose(np.array(READY),station.BASES['left'])
                down=np.diag([-1.,1.,-1.])
                self.begin('release',[Target(1,p,r,.18,'release bolt head'),Target(1.5,p+[0,0,.010],r,.18,'clear bolt head'),Target(1.5,p+[0,0,.010],down,.18,'straighten open fingers'),Target(2.5,np.array([AXIS[0],.12,p[2]+.010]),down,.18,'withdraw beside wall'),Target(3,ready_p,ready_r,1.,'left arm ready'),Target(1,ready_p,ready_r,1.,'observe started bolt')],t,f)
            elif self.phase=='release':
                self.after_bolt(t,f)
        a,label=self.plan.action7(t-self.start)
        return self.pack_action(a,label,observation)
    def after_bolt(self,t,f):
        self.stop=True;self.phase='done';self.end=t+1000.
        print('BOLT_START_COMPLETE',f.get('hardware'),flush=True)
    def pack_action(self,a,label,observation):
        from ratchet_task.embodiment import FEEDBACK as f
        bolt_phases=['align','insert','thread']
        tool_phases=['tool_approach','tool_enter','tool_align','tool_seat','ratchet','proof','tool_remove','tool_park']
        tracked='bolt' if self.phase in bolt_phases else 'handle' if self.phase in tool_phases and a[6]<.1 else None
        if tracked is not None:
            nominal,rotation=ik.tcp_pose(a[:6],station.BASES[self.side])
            desired_rotation=rotation@self.relative
            desired=nominal-desired_rotation@self.offset
            hand=f['hands'][self.side]
            delta=desired_rotation@f['rotations'][tracked].T
            # Convert the desired object pose through the CURRENT measured grasp,
            # so physical pad compliance/slip does not become a persistent bias.
            # Tool translation follows the measured grasp directly. Applying the
            # full orientation error here while commanding only a bounded angular
            # correction creates a lever-arm position bias.
            grasp=hand['tcp']-f['poses'][tracked]
            position=desired+grasp if tracked=='handle' else desired+delta@grasp
            if self.phase not in ['ratchet','proof']:
                error=ik._rot_error(desired_rotation,f['rotations'][tracked]);n=np.linalg.norm(error)
                if n>1.e-8:
                    from yam_arena.urdf import axis_angle_matrix
                    rotation=axis_angle_matrix(error/n,min(.01,.5*n))@rotation
            q,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(rotation),a[:6],base_pos=station.BASES[self.side],iters=100)
            if pe>.002 or re>.02:raise RuntimeError(f'Body feedback IK: {pe}, {re}')
            a=np.r_[q,a[6]]
        target=np.r_[a,READY,1.] if self.side=='left' else np.r_[READY,1.,a]
        actual=np.asarray(observation.state['joint_pos'])
        self.integral+=self._dt*(target-actual);self.integral[[6,13]]=0;self.integral=np.clip(self.integral,-.1,.1)
        return ActionChunk(actions=[Action(data=target+self.integral,meta={'request_stop':True,'waypoint':label} if self.stop else {'waypoint':label})],control_hz=1/self._dt)
def make_bolt_start_policy(**kwargs):return BoltStartPolicy(**kwargs)

class FastenPolicy(BoltStartPolicy):
    """Reference controller; all manipulation uses ordinary 14-D robot actions.

    Privileged object poses/contact torque plan normal actions for the reference
    demonstration. Dream receives the ordinary camera/joint observations only.
    """
    def __init__(self,**kwargs):
        super().__init__(**kwargs);self.info=replace(self.info,name='confined_ratchet_scripted')
    def after_bolt(self,t,f):
        self.side='right';self.plan=None;self.stop=False
        h=f['poses']['handle'];hr=f['rotations']['handle'];self.initial_tool=h.copy()
        p=h+hr@np.array([-.13,0,.0185]);r=hr@np.array([[0,-1,0],[-1,0,0],[0,0,-1.]])
        self.begin('tool_pickup',[Target(3,p+[0,0,.08],r,.65,'above ratchet'),Target(2.5,p,r,.65,'descend around grip'),Target(1.5,p,r,0.,'pinch handle'),Target(5,p+[0,0,.10],r,0.,'lift ratchet'),Target(1,p+[0,0,.10],r,0.,'settle lifted ratchet')],t,f)
    def tool_calibration(self,f):
        h=f['poses']['handle'];hr=f['rotations']['handle'];hand=f['hands']['right']
        self.offset=hr.T@(hand['tcp']-h);self.relative=hand['rotation'].T@hr
    def tool_plan(self,phase,goals,t,f):
        self.begin(phase,[self.body_target(seconds,np.array(p),r,label,grip) for seconds,p,r,label,grip in goals],t,f)
    def torque_action(self,t,f):
        hw=f['hardware'];torque=hw['interface_torque'];b=f['poses']['bolt']
        if f.get('tightened',False):
            theta=self.command_angle;clear=min(np.deg2rad(13),theta+np.deg2rad(2));p=b+np.array([0,0,.0305])
            self.tool_plan('tool_remove',[(1.,p,rz(clear),'unload clutch',0.),(2.,p+[0,0,.020],rz(clear),'lift socket clear',0.),(2.,p+[0,0,.020],np.eye(3),'straighten for exit',0.),(5.,[.20,0,p[2]+.020],np.eye(3),'withdraw through throat',0.)],t,f)
            return self.plan.action7(0.)
        seated=-.0003<=hw['gap']<=.0002 and hw['turns']>=REQUIRED_TURNS
        if self.phase=='ratchet' and seated and torque>=.35:
            self.phase='proof';self.hold_angle=self.command_angle;self.proof_start=t
        if self.phase=='proof':
            self.hold_angle-=np.clip(.10*(.5-torque)*self._dt,-.002,.002)
            self.hold_angle=float(np.clip(self.hold_angle,-np.deg2rad(14),np.deg2rad(14)))
            theta=self.hold_angle
            if (theta<=-np.deg2rad(13.99) and torque<.35) or t-self.proof_start>12.:
                self.phase='ratchet';self.stroke_start=t;self.first_angle=theta;self.stroke_from=theta;self.stroke_to=np.deg2rad(14);self.stroke_duration=.6
        else:
            elapsed=t-self.stroke_start
            if elapsed>=self.stroke_duration:
                self.stroke_from=self.stroke_to;self.stroke_to=-self.stroke_to;self.stroke_start=t;elapsed=0.
                self.stroke_duration=2. if self.stroke_to<0 and hw['gap']<.0005 else .6
            theta=self.stroke_from+(self.stroke_to-self.stroke_from)*ik.smoothstep(elapsed/self.stroke_duration)
        self.command_angle=float(theta)
        goal=b+np.array([0,0,.0305]);h=f['poses']['handle']
        body=goal
        r=rz(theta);tcp=body+r@self.offset;handrot=r@self.relative.T
        self.command_q,pe,re=ik.solve(tcp,ik.matrix_to_quat_xyzw(handrot),self.command_q,base_pos=station.BASES['right'],iters=100)
        if pe>.002 or re>.02:raise RuntimeError(f'Ratchet IK: {pe}, {re}')
        self.missing=self.missing+self._dt if not hw['socket_engaged'] else 0.
        if self.missing>1.:raise RuntimeError('Socket lost physical engagement during ratcheting')
        if int(t)%10==0 and t-self.last_report>1.:
            print('RATCHET_PROGRESS',t,hw['turns'],hw['gap'],torque,hw['ratchet_cycles'],flush=True);self.last_report=t
        return np.r_[self.command_q,0.],self.phase
    def act(self,observation):
        from ratchet_task.embodiment import FEEDBACK as f
        if self.phase in ['start','pickup','align','insert','thread','release']:return super().act(observation)
        t=float(observation.state_time)+self._dt
        if self.phase in ['ratchet','proof']:
            a,label=self.torque_action(t,f)
        else:
            if t>=self.end:
                b=f['poses']['bolt'];h=f['poses']['handle'];hand=f['hands']['right']
                if self.phase=='tool_pickup':
                    if h[2]-self.initial_tool[2]<.06:raise RuntimeError('Ratchet grasp failed to lift tool')
                    self.tool_calibration(f);self.entry_height=float(b[2]+.047)
                    self.tool_plan('tool_approach',[(4.,[.20,0,self.entry_height],np.eye(3),'approach pocket centreline',0.),(2.,[.20,0,self.entry_height],np.eye(3),'settle before narrow throat',0.)],t,f)
                elif self.phase=='tool_approach':
                    self.tool_calibration(f)
                    self.tool_plan('tool_enter',[(6.,[AXIS[0],0,self.entry_height],np.eye(3),'slide straight through throat',0.),(1.,[AXIS[0],0,self.entry_height],np.eye(3),'settle beyond wall mouth',0.)],t,f)
                elif self.phase in ['tool_enter','tool_align']:
                    self.tool_calibration(f)
                    hr=f['rotations']['handle'];dr=f['rotations']['drive'];br=f['rotations']['bolt']
                    beta=np.arctan2(br[1,0],br[0,0]);delta=np.arctan2(dr[1,0],dr[0,0])-np.arctan2(hr[1,0],hr[0,0])
                    angle=(beta-delta+np.pi/6)%(np.pi/3)-np.pi/6
                    if abs(angle)>np.deg2rad(13.):raise RuntimeError('Required socket phase exceeds wall clearance')
                    mismatch=(beta-np.arctan2(dr[1,0],dr[0,0])+np.pi/6)%(np.pi/3)-np.pi/6
                    if self.phase=='tool_enter' or abs(mismatch)>.012:
                        self.align_attempts=1 if self.phase=='tool_enter' else self.align_attempts+1
                        if self.align_attempts>4:raise RuntimeError('Socket hex alignment did not converge')
                        self.tool_plan('tool_align',[(2.,[AXIS[0],0,self.entry_height],rz(angle),'align socket hex',0.),(1.,[AXIS[0],0,self.entry_height],rz(angle),'settle aligned hex',0.)],t,f)
                    else:
                        self.tool_plan('tool_seat',[(3.,b+[0,0,.0305],rz(angle),'lower socket onto bolt head',0.),(2.,b+[0,0,.0305],rz(angle),'verify hex engagement',0.)],t,f)
                elif self.phase=='tool_seat':
                    if not f['hardware']['socket_engaged']:raise RuntimeError('Socket did not physically engage bolt')
                    self.tool_calibration(f);self.phase='ratchet';self.stroke_start=t;self.first_angle=float(np.arctan2(f['rotations']['handle'][1,0],f['rotations']['handle'][0,0]));self.stroke_from=self.first_angle;self.stroke_to=np.deg2rad(14);self.stroke_duration=.6;self.command_angle=self.first_angle;self.command_q=hand['joints'].copy();self.missing=0.;self.last_report=-100.
                elif self.phase=='tool_remove':
                    self.tool_calibration(f)
                    park=TOOL_START+np.array([0,0,.002])
                    self.tool_plan('tool_park',[(5.,[park[0],park[1],TABLE+.080],np.eye(3),'return to cradle',0.),(3.,park,np.eye(3),'lower onto saddles',0.),(1.5,park,np.eye(3),'release ratchet',1.)],t,f)
                elif self.phase=='tool_park':
                    p,r=hand['tcp'],hand['rotation'];rp,rr=ik.tcp_pose(np.array(READY),station.BASES['right'])
                    self.begin('tool_release',[Target(2,p+[0,0,.08],r,1.,'withdraw open fingers'),Target(3,rp,rr,1.,'right arm ready'),Target(4,rp,rr,1.,'released observation')],t,f)
                elif self.phase=='tool_release':
                    if f.get('score',0.)!=1.:raise RuntimeError('Final released state did not reach full score')
                    self.stop=True;self.end=t+1000.
            if self.phase in ['ratchet','proof']:a,label=self.torque_action(t,f)
            else:a,label=self.plan.action7(t-self.start)
        return self.pack_action(a,label,observation)
def make_fasten_policy(**kwargs):return FastenPolicy(**kwargs)
