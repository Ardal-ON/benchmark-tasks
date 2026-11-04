"""Four large-key puzzle pieces lie scattered on a workbench.
Assemble them into a square inside the four orange corner guides at the centre of the table.
Each T-shaped tab must fit its matching recess; all parts should lie flat.
"""
from pathlib import Path
import math
import numpy as np
from isaaclab_arena.assets.registries import AssetRegistry
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import Asset, Box, TABLE_TOP

SCENE='station'
TOP=TABLE_TOP[SCENE]
HEIGHT=.024
TABLE_CENTRE=(.36, 0.0)
# Each point is the world-aligned bounding-box centre; yaws are in radians.
START=[(.190,.180,0.0),(.285,.205,.40),(.180,-.180,-.40),(.285,-.205,.65)]
# Local bounding centres differ because tabs protrude; body centres form a 100 x 100 mm square.
GOAL=[(.341,.019),(.385,.019),(.341,-.025),(.385,-.025)]
COLORS=[(.18,.43,.65),(.65,.36,.12),(.20,.53,.37),(.56,.27,.48)]
OBJECTS=[]
for i,((x,y,yaw),color) in enumerate(zip(START,COLORS),1):
    name=f'robocurve_square_puzzle_piece_{i}'
    registry=AssetRegistry()
    if not registry.is_registered(name,ensure_loaded=False):
        registry.register(type(name,(),{'usd_path':str(Path(__file__).parent/'assets'/f'piece_{i}.usda')}),name)
    OBJECTS.append(Asset(f'piece_{i}',name,xy=(x,y),yaw=yaw,extent=.062 if i<4 else .05,mass=.08,color=color))

# Four flat corner marks, outside the 10 cm square; they do not form walls or supports.
for n,(x,y) in enumerate([(.307,-.053),(.307,.053),(.413,-.053),(.413,.053)],1):
    OBJECTS.append(Box(f'mark_{n}',pos=(x,y,TOP+.00025),size=(.006,.006,.0005),color=(.9,.35,.04)))


def _placed(obj,goal):
    q=obj.quat_xyzw/np.linalg.norm(obj.quat_xyzw)
    angle=2*math.acos(float(np.clip(abs(q[3]),0,1)))
    return bool(np.linalg.norm(obj.pos[:2]-goal)<=.004 and
                abs(obj.pos[2]-(TOP+HEIGHT/2))<=.004 and
                angle<=math.radians(8) and obj.resting())


def score(objects):
    n=sum(_placed(objects[f'piece_{i}'],np.asarray(goal)) for i,goal in enumerate(GOAL,1))
    if n<4: return n/4
    # All four seams must also be aligned, preventing credit for four nearby separated pieces.
    for i,j in ((0,1),(0,2),(1,3),(2,3)):
        delta=objects[f'piece_{j+1}'].pos-objects[f'piece_{i+1}'].pos
        target=np.r_[np.subtract(GOAL[j],GOAL[i]),0.0]
        if np.max(np.abs(delta-target))>.002: return .75
    return 1.0


def success(objects): return score(objects)==1.0


def distance(objects):
    return float(sum(np.linalg.norm(objects[f'piece_{i}'].pos[:2]-goal) for i,goal in enumerate(GOAL,1)))


def demo():
    # Initial candidate: left arm places the two near its side; right arm waits, then places the others.
    left=[];right=[Waypoint(22.0,(.12,-.30,TOP+.13),FINGER_OPEN,'wait clear of left arm')]
    for i in range(4):
        x,y,yaw=START[i]; gx,gy=GOAL[i]
        moves=pick_place_waypoints((x,y,TOP+HEIGHT/2+.003),
                                  (gx,gy,TOP+HEIGHT/2+.007),
                                  travel_z=TOP+.12,yaw=yaw,place_yaw=0.0)
        (left if i<2 else right).extend(moves)
    return {'left':left,'right':right}
