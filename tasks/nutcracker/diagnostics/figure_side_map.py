import sys,itertools
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from nutcracker_task.config import *
RZ=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
rots=[np.array([[1.,0,0],[0,0,1],[0,-1,0]]),np.array([[0.,1,0],[0,0,1],[1,0,0.]])]
for dx,dy,rot in itertools.product((-.10,-.05,0,.05,.10),(-.06,-.03,0,.03,.06),rots):
 p=HINGE+[dx,dy,0]+RZ@HAND_LOCAL+[0,-.020,0];q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rot),np.array(READY),base_pos=station.BASES['right'],iters=120)
 if pe<.003 and re<.03:
  worst=[]
  for a in np.linspace(0,OPEN_ANGLE,5):
   c,s=np.cos(a),np.sin(a);rx=np.array([[1,0,0],[0,c,-s],[0,s,c]]);q,ep,er=ik.solve(HINGE+[dx,dy,0]+RZ@(JAW_PIVOT+rx@(HAND_LOCAL-JAW_PIVOT)),ik.matrix_to_quat_xyzw(rot),q,base_pos=station.BASES['right'],iters=120);worst.append((ep,er))
  print('MAP',dx,dy,'rot',rot.tolist(),'pre',pe,re,'arc',np.max(worst,axis=0).tolist(),flush=True)
