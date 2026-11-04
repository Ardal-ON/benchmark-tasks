"""Four large-key puzzle pieces lie scattered on a workbench.
Assemble them into a square inside the four orange corner guides at the centre of the table.
Each T-shaped tab must fit its matching recess; all parts should lie flat.
"""
from pathlib import Path
import math
import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import Box, TABLE_TOP

SCENE='station'
TOP=TABLE_TOP[SCENE]
HEIGHT=.024
GRASP_Z_OFFSET=.023
RELEASE_GAP=.008
TABLE_CENTRE=(.36, 0.0)
# Each point is the world-aligned bounding-box centre; yaws are in radians.
START=[(.190,.180,0.0),(.285,.205,.40),(.180,-.180,-.40),(.285,-.205,.65)]
# Local bounding centres differ because tabs protrude; body centres form a 100 x 100 mm square.
GOAL=[(.341,.019),(.385,.019),(.341,-.025),(.385,-.025)]
COLORS=[(.18,.43,.65),(.65,.36,.12),(.20,.53,.37),(.56,.27,.48)]
OBJECTS=[]
for i,((x,y,yaw),color) in enumerate(zip(START,COLORS),1):
    OBJECTS.append(Box(f'square_api_piece_{i}',pos=(x,y,TOP+HEIGHT/2),size=(.05,.05,HEIGHT),mass=.08,color=color,yaw=yaw))

# Install an Isaac Lab primitive-spawner hook; it creates compound bodies in memory.
import importlib.util as _importlib
_spec=_importlib.spec_from_file_location('square_api_hook',Path(__file__).with_name('procedural.py'))
_hook=_importlib.module_from_spec(_spec)
_spec.loader.exec_module(_hook)
_hook.install(Path(__file__).parent)

# Four flat corner marks, outside the 10 cm square; they do not form walls or supports.
for n,(x,y) in enumerate([(.307,-.053),(.307,.053),(.413,-.053),(.413,.053)],1):
    OBJECTS.append(Box(f'mark_{n}',pos=(x,y,TOP+.00025),size=(.006,.006,.0005),color=(.9,.35,.04)))


def _placed(obj,goal):
    q=obj.quat_xyzw/np.linalg.norm(obj.quat_xyzw)
    angle=2*math.acos(float(np.clip(abs(q[3]),0,1)))
    return bool(np.linalg.norm(obj.pos[:2]-goal)<=.004 and
                abs(obj.pos[2]-(TOP+HEIGHT/2))<=.004 and
                angle<=math.radians(8) and obj.resting())


demo_states={}


def score(objects):
    demo_states.update(objects)
    n=sum(_placed(objects[f'square_api_piece_{i}'],np.asarray(goal)) for i,goal in enumerate(GOAL,1))
    if n<4: return n/4
    # All four seams must also be aligned, preventing credit for four nearby separated pieces.
    for i,j in ((0,1),(0,2),(1,3),(2,3)):
        delta=objects[f'square_api_piece_{j+1}'].pos-objects[f'square_api_piece_{i+1}'].pos
        target=np.r_[np.subtract(GOAL[j],GOAL[i]),0.0]
        if np.max(np.abs(delta-target))>.002: return .75
    return 1.0


def success(objects): return score(objects)==1.0


def distance(objects):
    return float(sum(np.linalg.norm(objects[f'square_api_piece_{i}'].pos[:2]-goal) for i,goal in enumerate(GOAL,1)))


def grasp_offset(index):
    """Grip a solid strip clear of all four mating profiles."""
    cells=_hook.geometry.cells_for(index)
    cx=(min(x for x,_ in cells)+max(x for x,_ in cells)+1)*.0005
    cy=(min(y for _,y in cells)+max(y for _,y in cells)+1)*.0005
    gx,gy=(.015,0.)
    return np.array([gx-cx,gy-cy])


def grasp_yaw(index):
    return 0.


def demo():
    left=[];right=[Waypoint(28.0,(.12,-.30,TOP+.13),FINGER_OPEN,'wait clear of left arm')]
    for i in range(4):
        x,y,yaw=START[i]; gx,gy=GOAL[i]
        offset=grasp_offset(i)
        c,s=math.cos(yaw),math.sin(yaw)
        pick_offset=np.array([[c,-s],[s,c]])@offset
        moves=pick_place_waypoints((x+pick_offset[0],y+pick_offset[1],TOP+HEIGHT/2+GRASP_Z_OFFSET),
                                  (gx+offset[0],gy+offset[1],TOP+HEIGHT/2+GRASP_Z_OFFSET+RELEASE_GAP),
                                  travel_z=TOP+.09,yaw=yaw+grasp_yaw(i),place_yaw=grasp_yaw(i))
        # Let horizontal motion settle before entering the 2 mm mating clearance.
        moves.insert(5,Waypoint(1.0,moves[4].pos,moves[4].finger,'align',grasp_yaw(i)))
        moves[6].duration=2.0
        moves.insert(7,Waypoint(1.0,moves[6].pos,moves[6].finger,'seat',grasp_yaw(i)))
        moves.insert(8,Waypoint(.5,moves[7].pos,moves[7].finger,'settle',grasp_yaw(i)))
        for move in moves: move.label=f'{i+1}:{move.label}'
        (left if i<2 else right).extend(moves)
    left.append(Waypoint(2.0,(.12,.30,TOP+.12),FINGER_OPEN,'park left clear'))
    right.append(Waypoint(2.0,(.12,-.30,TOP+.12),FINGER_OPEN,'park right clear'))
    return {'left':left,'right':right}
