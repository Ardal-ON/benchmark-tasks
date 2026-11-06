"""Live PhysX adapter. Only external forces/torques and passive constraints.

Direct state writes are intentionally absent. Diagnostic fixtures are separate
programs; policy actions cannot invoke the force API or change this hardware.
"""
import math
import numpy as np
from ratchet_task.mechanics import Clutch,Screw
from ratchet_task.config import *
class Hardware:
    def __init__(self,stage,dt):
        from pxr import UsdUtils,PhysicsSchemaTools
        import omni.physx
        self.stage=stage;self.dt=dt;self.native=omni.physx.get_physx_interface();self.sim=omni.physx.get_physx_simulation_interface()
        self.stageid=UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.ids={n:PhysicsSchemaTools.sdfPathToInt('/World/envs/env_0/'+n) for n in ['handle','drive','bolt']}
        self.clutch=Clutch();self.screw=Screw();self.last={};self.angles={};self.candidate=None;self.thread_joint=None
        self.cycles=0;self.return_start=None;self.drive_start=None;self.telemetry={};self.enabled=True
    def pose(self,n):
        from yam_arena.urdf import quat_xyzw_to_matrix
        v=self.native.get_rigidbody_transformation('/World/envs/env_0/'+n)
        if not v.get('ret_val'):raise RuntimeError('Missing live body '+n)
        p=np.array(v['position']);q=np.array(v['rotation']);r=quat_xyzw_to_matrix(q)
        angle=math.atan2(r[1,0],r[0,0]);old=self.last.get(n)
        delta=0. if old is None else math.atan2(math.sin(angle-old[1]),math.cos(angle-old[1]))
        unwrapped=self.angles.get(n,angle)+delta
        velocity=np.zeros(3) if old is None else (p-old[0])/self.dt
        self.last[n]=(p,angle);self.angles[n]=unwrapped
        return p,r,q,unwrapped,delta/self.dt,velocity
    def torque(self,n,t):
        from pxr import Gf
        self.sim.apply_torque(self.stageid,self.ids[n],Gf.Vec3f(0,0,float(t)))
    def force(self,n,f,p):
        from pxr import Gf
        self.sim.apply_force_at_pos(self.stageid,self.ids[n],Gf.Vec3f(0,0,float(f)),Gf.Vec3f(*map(float,p)),'Force')
    def capture(self,p,q,theta):
        from pxr import UsdPhysics,Gf,Sdf
        j=UsdPhysics.Joint.Define(self.stage,'/World/envs/env_0/ThreadGuide')
        j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/bolt')]);j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,p)));j.CreateLocalPos1Attr(Gf.Vec3f(0))
        j.CreateLocalRot1Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,-q[:3]))))
        for axis in ['transX','transY','rotX','rotY']:
            limit=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);limit.CreateLowAttr(0.);limit.CreateHighAttr(0.)
        j.CreateCollisionEnabledAttr(True);j.CreateExcludeFromArticulationAttr(True);j.CreateJointEnabledAttr(True)
        self.thread_joint=j;self.screw.capture(float(p[2]),theta)
    def step(self):
        if not self.enabled:return
        h,hr,hq,ha,hw,hv=self.pose('handle');d,dr,dq,da,dw,dv=self.pose('drive');b,br,bq,ba,bw,bv=self.pose('bolt')
        # Relative drive-minus-handle rotation: positive extension is a clockwise handle drive.
        relative=da-ha
        torque=self.clutch.step(relative,dw-hw)
        # Implicit scalar correction controls the small drive inertia numerically.
        inertia=2.e-5
        torque/=1.+self.clutch.stiffness*self.dt*self.dt/inertia
        self.torque('drive',-torque);self.torque('handle',torque)
        tip=b-br[:,2]*BOLT_LENGTH
        radial=float(np.linalg.norm(tip[:2]-AXIS[:2]));tilt=float(math.acos(np.clip(br[2,2],-1,1)))
        inserted=radial<=CAPTURE_OFFSET and tilt<=CAPTURE_TILT and AXIS[2]-tip[2]>=.001 and b[2]>=AXIS[2]-.0003
        if not self.screw.captured:
            if inserted:
                if self.candidate is None:self.candidate=ba
                if ba-self.candidate<=-THREAD_START:self.capture(b,bq,ba)
            else:self.candidate=None
        force=resistance=preload=0.
        gap=float(b[2]-AXIS[2])
        if self.screw.captured:
            force,resistance,preload=self.screw.forces(float(b[2]),ba,float(bv[2]),bw,gap)
            alpha=PITCH/(2*math.pi)
            denom=1.+(self.screw.axial_stiffness*self.dt*self.dt+self.screw.axial_damping*self.dt)/BOLT_MASS
            force/=denom
            # Recompute reciprocal screw torque after the implicit force correction.
            resistance=-alpha*force-self.screw.running_torque*math.tanh(bw/.02)
            if self.screw.seat_angle is not None:
                from pxr import UsdPhysics
                drive=UsdPhysics.DriveAPI.Apply(self.thread_joint.GetPrim(),'rotZ')
                drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(self.screw.clamp_stiffness*math.pi/180);drive.CreateDampingAttr(.01*math.pi/180)
                drive.CreateTargetPositionAttr((self.screw.seat_angle-self.screw.angle0)*180/math.pi);drive.CreateMaxForceAttr(1.2)
            self.force('bolt',force,b);self.torque('bolt',resistance)
        bottom=d+dr@np.array([0.,0.,-.028])
        # Geometric contact eligibility supplements, never replaces, physical hex contact.
        mismatch=math.atan2(math.sin(6*(da-ba)),math.cos(6*(da-ba)))/6
        engagement=float(np.linalg.norm(bottom[:2]-b[:2]))<.0005 and .001<bottom[2]-b[2]<HEAD_HEIGHT-.001 and abs(mismatch)<.07 and dr[2,2]>.996
        if engagement:
            if self.drive_start is None:self.drive_start=ha
            if torque>.01 and ha<self.drive_start-.1:self.return_start=ha
            if self.return_start is not None and torque<=.005 and ha>self.return_start+.1:
                self.cycles+=1;self.return_start=None;self.drive_start=ha
        else:self.drive_start=None;self.return_start=None
        valid=np.all(np.isfinite(np.r_[h,d,b,ha,da,ba]))
        self.telemetry=dict(captured=self.screw.captured,turns=0. if not self.screw.captured else -(ba-self.screw.angle0)/(2*math.pi),gap=gap,clutch_torque=torque,preload=preload,interface_torque=max(0.,torque) if engagement else 0.,socket_engaged=engagement,ratchet_cycles=self.cycles,valid=bool(valid),angles=dict(handle=ha,drive=da,bolt=ba),positions=dict(handle=h.tolist(),drive=d.tolist(),bolt=b.tolist()),thread_force=force,radial_error=radial,tilt=tilt)
        return self.telemetry
