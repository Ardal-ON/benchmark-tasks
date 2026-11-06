"""Passive constitutive laws; independent of simulator and policy.

The clutch is a unilateral torsional spring with 5 degree take-up. Screw travel
is an SI-unit compliant helix; no method writes a pose, progress, or a score.
"""
from dataclasses import dataclass
import math

@dataclass
class Clutch:
    backlash: float=math.radians(5)
    stiffness: float=8.
    damping: float=.03
    capacity: float=1.2
    drag: float=.002
    latch: float|None=None
    torque: float=0.
    def reset(self,relative=0.):
        self.latch=relative+self.backlash;self.torque=0.
    def step(self,relative,relative_speed):
        if self.latch is None:self.reset(relative)
        # Forward reference follows return travel, retaining finite take-up.
        self.latch=min(self.latch,relative+self.backlash)
        extension=max(0.,relative-self.latch)
        spring=max(0.,min(self.capacity,self.stiffness*extension+self.damping*max(0.,relative_speed))) if extension>0 else 0.
        if spring>=self.capacity:self.latch=relative-self.capacity/self.stiffness
        # Viscous return drag always removes, rather than creates, energy.
        self.torque=spring+self.drag*relative_speed
        return self.torque

@dataclass
class Screw:
    pitch: float=.00125
    axial_stiffness: float=20000.
    axial_damping: float=12.
    running_torque: float=.015
    clamp_stiffness: float=2.
    captured: bool=False
    z0: float=0.
    angle0: float=0.
    seat_angle: float|None=None
    def capture(self,z,angle):
        self.captured=True;self.z0=z;self.angle0=angle;self.seat_angle=None
    def forces(self,z,angle,vz,omega,gap):
        if not self.captured:return 0.,0.,0.
        alpha=self.pitch/(2*math.pi)
        error=z-self.z0-alpha*(angle-self.angle0) # positive angle is CCW; CCW raises bolt
        error_speed=vz-alpha*omega
        force=max(-30.,min(30.,-self.axial_stiffness*error-self.axial_damping*error_speed))
        # Force acts along +Z; reciprocal torque ensures the spring is passive.
        torque=-alpha*force-self.running_torque*math.tanh(omega/.02)
        if gap<=.0002 and self.seat_angle is None:self.seat_angle=angle
        preload=0.
        if self.seat_angle is not None:
            # Equivalent torsional compliance/preload of a seated bench joint.
            preload=max(0.,self.clamp_stiffness*(self.seat_angle-angle))
            torque+=preload
        return force,torque,preload
