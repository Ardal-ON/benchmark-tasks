"""URDF gravity feedforward expressed as stock position-servo target offsets."""
import xml.etree.ElementTree as ET
import numpy as np
from yam_arena import YAM_URDF,ik
from yam_arena.urdf import forward_kinematics,parse_joints,rpy_matrix
from yam_arena.constants import ARM_JOINTS,SHOULDER,WRIST
JOINTS=parse_joints(YAM_URDF)
MASSES={}
for link in ET.parse(YAM_URDF).getroot().findall('link'):
    inertial=link.find('inertial')
    if inertial is not None:
        origin=inertial.find('origin')
        MASSES[link.get('name')]=(float(inertial.find('mass').get('value')),np.fromstring(origin.get('xyz','0 0 0'),sep=' '))
DESCENDANTS={}
for name in ARM_JOINTS:
    links={JOINTS[name].child}
    while True:
        expanded=links|{j.child for j in JOINTS.values() if j.parent in links}
        if expanded==links:break
        links=expanded
    DESCENDANTS[name]=links
STIFFNESS=np.array([SHOULDER['stiffness']]*3+[WRIST['stiffness']]*3)
def potential(q):
    poses=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,q)))
    return sum(m*9.81*(poses[n][:3,:3]@com+poses[n][:3,3])[2] for n,(m,com) in MASSES.items())
def torque(q):
    poses=forward_kinematics(YAM_URDF,dict(zip(ARM_JOINTS,q)))
    centres={n:T[:3,:3]@MASSES[n][1]+T[:3,3] for n,T in poses.items() if n in MASSES}
    result=[]
    for name in ARM_JOINTS:
        joint=JOINTS[name];parent=poses[joint.parent]
        origin=parent[:3,:3]@joint.xyz+parent[:3,3]
        axis=parent[:3,:3]@rpy_matrix(joint.rpy)@joint.axis
        result.append(sum(MASSES[n][0]*9.81*np.cross(axis,centres[n]-origin)[2] for n in DESCENDANTS[name] if n in MASSES))
    return np.asarray(result)
def offset(q):return torque(q)/STIFFNESS
def target(q):return np.clip(np.asarray(q)+offset(q),ik.LOWER,ik.UPPER)
