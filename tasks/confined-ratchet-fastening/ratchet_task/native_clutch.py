"""Passive one-way clutch using a native bounded torsional spring.

Rephasing a revolute joint about its *free* Z axis changes its coordinate origin,
not its physical hinge axis, pivot, or either body's pose. On return travel the
unloaded reference follows the shaft. On drive travel it stays fixed. Keeping
quaternion signs aligned avoids artificial whole-turn drive errors.
"""
import math
import numpy as np
from ratchet_task.mechanics import Clutch
class NativeClutch:
    def __init__(self,stage):
        from pxr import UsdPhysics
        self.law=Clutch(stiffness=100.,damping=.001)
        self.joint=UsdPhysics.RevoluteJoint.Get(stage,'/World/envs/env_0/RatchetDriveJoint')
        drive=UsdPhysics.DriveAPI.Apply(self.joint.GetPrim(),'angular')
        drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(self.law.stiffness*math.pi/180);drive.CreateDampingAttr(self.law.damping*math.pi/180);drive.CreateTargetPositionAttr(0.);drive.CreateTargetVelocityAttr(0.)
        self.limit=drive.CreateMaxForceAttr(0.);self.frame=self.joint.CreateLocalRot0Attr();self.last_quat=None
        self.last_relative=None;self.relative=0.
    def step(self,hr,dr,hq,dq,dt):
        from pxr import Gf
        relative_matrix=hr.T@dr
        raw=math.atan2(relative_matrix[1,0],relative_matrix[0,0])
        delta=0. if self.last_relative is None else math.atan2(math.sin(raw-self.last_relative),math.cos(raw-self.last_relative))
        self.relative=raw if self.last_relative is None else self.relative+delta
        self.last_relative=raw;speed=delta/dt
        self.law.step(self.relative,speed)
        phase=self.law.latch
        q=Gf.Quatf(float(math.cos(phase/2)),0.,0.,float(math.sin(phase/2)))
        parent=Gf.Quatf(float(hq[3]),Gf.Vec3f(*map(float,hq[:3])))
        world=parent*q;sign=np.dot(np.r_[list(world.GetImaginary()),world.GetReal()],dq)
        if sign<0:q=Gf.Quatf(-q.GetReal(),-q.GetImaginary())
        parts=np.r_[list(q.GetImaginary()),q.GetReal()]
        if self.last_quat is None or np.max(abs(parts-self.last_quat))>1.e-8:
            self.frame.Set(q);self.last_quat=parts
        extension=max(0.,self.relative-phase)
        signed=self.law.stiffness*extension+self.law.damping*speed
        engaged=self.relative>phase and signed>0.
        self.limit.Set(self.law.capacity if engaged else 0.)
        estimate=min(self.law.capacity,max(0.,signed)) if engaged else 0.
        return float(estimate+self.law.drag*speed),float(self.law.drag*speed)
