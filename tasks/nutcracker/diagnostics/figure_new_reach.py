import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from nutcracker_task.config import *
R=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
orient=np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])
for dy in (0.,.04,.07):
 q=np.array(READY);errors=[]
 for a in np.linspace(0,OPEN_ANGLE,7):
  c,s=np.cos(a),np.sin(a);rot=np.array([[1,0,0],[0,c,-s],[0,s,c]]);p=HINGE+[0,dy,0]+R@rot@HAND_LOCAL
  q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(orient),q,base_pos=station.BASES['right'],iters=400);errors.append((pe,re))
 print('HAND_Y_OFFSET',dy,'WORST',np.max(errors,axis=0).tolist(),flush=True)
