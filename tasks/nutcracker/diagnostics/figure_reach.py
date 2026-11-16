"""Kinematic feasibility of the figurine hand and the nut insertion, not collision certification."""
import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from yam_arena import ik,station
from yam_arena.demo import grip_rotation
from yam_arena.constants import READY
from nutcracker_task.config import *
RZ=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
H=np.array([[0,0,1],[0,1,0],[-1,0,0.]])
for side in ('left','right'):
 print('SIDE',side)
 if side=='right':
  for a in np.linspace(0,OPEN_ANGLE,6):
   c,s=np.cos(a),np.sin(a);RX=np.array([[1,0,0],[0,c,-s],[0,s,c]])
   p=HINGE+RZ@RX@HAND_LOCAL+[0,0,.012]
   q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(grip_rotation(0)),np.array(READY),base_pos=station.BASES[side],iters=500)
   print('hand',round(a,2),np.round(p,4),round(pe,4),round(re,4),np.round(q,3).tolist(),flush=True)
 else:
  for dx in (-.10,-.07,-.04,-.015,0.):
   p=NUT_BENCH+[dx,0,0]
   for off in (0.,-.4,.4):
    c,s=np.cos(off),np.sin(off);rot=np.array([[c,-s,0],[s,c,0],[0,0,1]])@H
    q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rot),np.array(READY),base_pos=station.BASES[side],iters=400)
    print('mouth',dx,off,np.round(p,4),round(pe,4),round(re,4),np.round(q,3).tolist(),flush=True)
