import sys,itertools
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from nutcracker_task.config import *
RZ=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
for down in ([0,0,-1],[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]):
 down=np.array(down);ref=np.array([0.,0,1.]) if down[2]==0 else np.array([1.,0,0]);a=np.cross(ref,down);b=np.cross(down,a)
 for k in range(4):
  theta=k*np.pi/2;u=a*np.cos(theta)+b*np.sin(theta);v=-a*np.sin(theta)+b*np.cos(theta);rot=np.column_stack([u,v,down]);q=np.array(READY);err=[]
  for i in range(6):
   ang=OPEN_ANGLE*i/5;c,s=np.cos(ang),np.sin(ang);rx=np.array([[1,0,0],[0,c,-s],[0,s,c]])
   p=HINGE+RZ@(JAW_PIVOT+rx@(HAND_LOCAL-JAW_PIVOT))
   q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rot),q,base_pos=station.BASES['right'],iters=300);err.append((pe,re))
  worst=np.max(err,axis=0)
  if worst[0]<.003 and worst[1]<.03:
   for gap in (.03,.05,.07):
    p0=HINGE+RZ@HAND_LOCAL-gap*down
    _,pe,re=ik.solve(p0,ik.matrix_to_quat_xyzw(rot),np.array(READY),base_pos=station.BASES['right'],iters=400)
    print('FOUND',down.tolist(),k,gap,'arc',worst.tolist(),'approach',[pe,re],'rot',rot.tolist(),flush=True)
