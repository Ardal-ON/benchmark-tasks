import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from nutcracker_task.config import *
rng=np.random.default_rng(12);RZ=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
for k in (1,2,3):
 down=np.array([0.,1.,0.]);a=np.array([-1.,0.,0.]);b=np.cross(down,a);u=a*np.cos(k*np.pi/2)+b*np.sin(k*np.pi/2);v=-a*np.sin(k*np.pi/2)+b*np.cos(k*np.pi/2);rot=np.column_stack([u,v,down])
 for gap in (.005,.015,.025,.035):
  p=HINGE+RZ@HAND_LOCAL+[0,-gap,0]
  for n in range(20):
   seed=np.array(READY) if n==0 else rng.uniform(ik.LOWER,ik.UPPER)
   q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rot),seed,base_pos=station.BASES['right'],iters=250)
   if pe<.002 and re<.02:
    worst=[]
    for a in np.linspace(0,OPEN_ANGLE,6):
     c,s=np.cos(a),np.sin(a);rx=np.array([[1,0,0],[0,c,-s],[0,s,c]]);p2=HINGE+RZ@(JAW_PIVOT+rx@(HAND_LOCAL-JAW_PIVOT))
     q,pe2,re2=ik.solve(p2,ik.matrix_to_quat_xyzw(rot),q,base_pos=station.BASES['right'],iters=180);worst.append((pe2,re2))
    print('FOUND',k,gap,n,'pre',pe,re,'arc',np.max(worst,axis=0).tolist(),'rot',rot.tolist(),'q',q.tolist(),flush=True);break
