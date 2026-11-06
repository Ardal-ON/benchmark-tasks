"""Passive constitutive laws; independent of simulator and policy.

The clutch is a unilateral torsional spring with 5 degree take-up. Screw travel
is a native PhysX constrained helix; no method writes a pose, progress, or a score.
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
    def step(self,relative,relative_speed,dt=0.,inertia=2.e-5):
        if self.latch is None:self.reset(relative)
        # Forward reference follows return travel, retaining finite take-up.
        self.latch=min(self.latch,relative+self.backlash)
        extension=max(0.,relative-self.latch)
        implicit=1.+(self.stiffness*dt*dt+self.damping*dt)/inertia
        spring=max(0.,min(self.capacity,(self.stiffness*extension+self.damping*max(0.,relative_speed))/implicit)) if extension>0 else 0.
        if spring>=self.capacity:self.latch=relative-self.capacity*implicit/self.stiffness
        # Viscous return drag always removes, rather than creates, energy.
        self.torque=spring+self.drag*relative_speed
        return self.torque

@dataclass
class Screw:
    """Capture reference for the native PhysX helical joint; no pose writes."""
    pitch: float=.00125
    captured: bool=False
    z0: float=0.
    angle0: float=0.
    def capture(self,z,angle):
        self.captured=True;self.z0=z;self.angle0=angle
    def expected_z(self,angle):
        if not self.captured:raise RuntimeError('Thread not captured')
        return self.z0+self.pitch*(angle-self.angle0)/(2*math.pi)
