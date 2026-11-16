import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from yam_arena import ik,station
from yam_arena.demo import grip_rotation
from yam_arena.constants import READY
from nutcracker_task.config import *
RZ=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
for dx in (-.10,-.08,-.06):
 for y in (-.08,-.10,-.13):
  for yaw in (0.,-.5,.5,-1.):
   q=np.array(READY);worst=[]
   for a in np.linspace(0,OPEN_ANGLE,7):
    c,s=np.cos(a),np.sin(a);RX=np.array([[1,0,0],[0,c,-s],[0,s,c]])
    p=HINGE+[dx,0,0]+RZ@RX@np.array([.04,y,.035])+[0,0,.008]
    q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(grip_rotation(yaw)),q,base_pos=station.BASES['right'],iters=180);worst.append([pe,re])
   print('RIGHT',dx,y,yaw,*np.round(np.max(worst,axis=0),5),flush=True)
