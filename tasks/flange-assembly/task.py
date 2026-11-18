"""Supported PVC-U pipe joint with two upper horizontal bolts."""
import numpy as np
from yam_arena.demo import Waypoint,pick_place_waypoints,grip_rotation
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
from flange_assembly.policy import PoseWaypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,READY
from yam_arena import ik,station
from yam_arena.task_file import TABLE_TOP,Box,Cylinder
from flange_assembly.specs import *

SCENE='station'
TOP=TABLE_TOP[SCENE]
# Real fixed socket flanges on ordinary430/200mm PVC-U pipe, no grip lug.
AXIS_HEIGHT=.17
LOWER=Box('lower_flange',pos=(.3145,.06,TOP+AXIS_HEIGHT),size=(.203,.115,.115))
UPPER=Box('upper_flange',pos=(INITIAL_FACE_X,.06,TOP+AXIS_HEIGHT),size=(.470,.2745,.115),mass=MOVING_MASS)
PIN_START=[(.32,-.30),(.32,-.18)]
PINS=[Cylinder(f'pin_{i+1}',pos=(x,y,TOP+.195),radius=.006,height=.05,mass=.0632,color=(.55,.57,.59)) for i,(x,y) in enumerate(PIN_START)]
# V rests support both pipes throughout ordinary axial slide and clocking.
RESTS=[Box(name,pos=(x,.06,TOP),size=(.03,.06,.177372583),color=(.44,.47,.49)) for name,x in
       [('moving_rest_1',MOVING_REST_X[0]),('moving_rest_2',MOVING_REST_X[1]),('fixed_rest_1',.40),('fixed_rest_2',.48)]]
PIN_SUPPORTS=[Box(f'pin_support_{i+1}',pos=(x+.005,y,TOP+.0925),size=(.05,.028,.185),color=(.34,.32,.28)) for i,(x,y) in enumerate(PIN_START)]
BRANCH_STAGE=Box('branch_stage',pos=(-.16,.195,TOP+.0565),size=(.05,.025,.113),color=(.44,.47,.49))
OBJECTS=[LOWER,UPPER,*RESTS,BRANCH_STAGE,*PINS,*PIN_SUPPORTS]
SCORE_OVER='end'


def score(o):
    from flange_assembly.geometry import score as horizontal_score
    return horizontal_score(o['lower_flange'], o['upper_flange'],
                            [o[f'pin_{i}'] for i in range(1, 3)])


def demo():
    # Pinch the32mm pipe, clock it15degrees, then slide22.5mm into alignment.
    down=grip_rotation(np.pi)
    initial=axis_angle_matrix(np.array([1.,0.,0.]),np.deg2rad(-15)) @ down
    rot=tuple(down.reshape(-1));initial=tuple(initial.reshape(-1))
    pick=(LEFT_PICK_X,.06-.009*np.sin(np.deg2rad(-15)),TOP+.17+.009*np.cos(np.deg2rad(-15)))
    left=[PoseWaypoint(3.,(LEFT_PICK_X,.062329,TOP+.2387),FINGER_OPEN,'approach pipe',rotation=initial),
          PoseWaypoint(3.,pick,FINGER_OPEN,'surround pipe',rotation=initial),
          PoseWaypoint(2.,pick,-.012,'grasp pipe',rotation=initial),
          PoseWaypoint(5.,(LEFT_PICK_X,.06,TOP+.179),-.012,'clock pipe',rotation=rot),
          PoseWaypoint(5.,(LEFT_TARGET_X,.06,TOP+.179),-.012,'slide pipe',rotation=rot),
          PoseWaypoint(95.,(LEFT_TARGET_X,.06,TOP+.179),-.012,'hold flange',rotation=rot)]
    park_pos,park_rot=ik.tcp_pose(np.asarray(READY),station.BASES['right'])
    # Cant the palm upward27deg; the bolt itself stays horizontal in the head grip.
    theta=np.deg2rad(27);approach=np.array([np.cos(theta),0.,-np.sin(theta)])
    closing=np.array([0.,-1.,0.]);side_R=np.column_stack((closing,np.cross(approach,closing),approach))
    side=tuple(side_R.reshape(-1))
    def P(t,p,f,label):return PoseWaypoint(t,p,f,label,rotation=side)
    near=.06-HOLE_COMPONENT;far=.06+HOLE_COMPONENT;hz=TOP+.17+HOLE_COMPONENT-.002
    px=.29125-.004*approach[0];pz=TOP+.191-.004*approach[2]
    def sequence(second,stand_y,hole_y):
        labels=(['approach second pin','lower beside second pin','enter second pin','grasp second pin','lift second pin','approach second hole','insert second pin','seat second pin','release second pin','back off second pin','close to push second','approach second push','push second pin head','settle second push','clear second push','withdraw second'] if second else
                ['approach above pin','lower beside pin','enter pin','grasp pin','lift pin','approach hole','insert pin','seat pin','release pin','back off pin','close to push','approach push','push pin head','settle push','clear push','withdraw'])
        data=[(3.,(.265,stand_y,TOP+.24),FINGER_OPEN),
              (3.,(.265,stand_y,pz),FINGER_OPEN),(2.,(px,stand_y,pz),FINGER_OPEN),
              (1.5,(px,stand_y,pz),FINGER_CLOSED),(3.,(px,stand_y,TOP+.24),FINGER_CLOSED),
              (5.,(.23,hole_y,hz),FINGER_CLOSED),(7.,(.292,hole_y,hz),FINGER_CLOSED),
              (2.,(.292,hole_y,hz),FINGER_CLOSED),(1.,(.292,hole_y,hz),FINGER_OPEN),
              (2.,(.225,hole_y,hz),FINGER_OPEN),(1.,(.225,hole_y,hz),FINGER_CLOSED),
              (2.,(.235,hole_y,hz+.006),FINGER_CLOSED),
              (4.,(.2765,hole_y,hz+.006),FINGER_CLOSED),
              (2.,(.2765,hole_y,hz+.006),FINGER_CLOSED),
              (2.,(.2765,hole_y,hz+.046),FINGER_CLOSED),
              (3.,(.18,hole_y,hz+.046),FINGER_CLOSED)]
        return [P(t,pos,f,label) for (t,pos,f),label in zip(data,labels)]
    right=[PoseWaypoint(25.,tuple(park_pos),FINGER_OPEN,'wait',rotation=tuple(park_rot.reshape(-1))),
           *sequence(False,-.18,near),*sequence(True,-.30,far)]
    return {'left':left,'right':right}
