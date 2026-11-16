"""Pure kinematic reach test for a range of handle lengths; no physics."""
import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from yam_arena import ik,station
from yam_arena.demo import grip_rotation
from yam_arena.constants import READY
from nutcracker_task.config import HINGE,ANGLE
for r in (.125,.15,.175):
 for dx in (-.08,-.06,-.04,0.):
  q=np.array(READY);worst=[]
  for a in np.linspace(-ANGLE,.25,9):
   p=HINGE+[dx,0,0]+[r*math.cos(a),r*math.sin(a),.010]
   q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(grip_rotation(a)),q,base_pos=station.BASES['right'],iters=400)
   worst.append((pe,re))
  print('REACH',r,dx,'worst',np.max(worst,axis=0).tolist(),flush=True)
