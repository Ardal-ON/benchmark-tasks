import sys,itertools
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from nutcracker_task.config import *
RZ=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
for dx,dy,dz in itertools.product((-.08,-.04,0.),(-.07,-.03),(-.04,-.02,0.)):
 for down in ([0,0,-1],[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]):
  down=np.array(down);ref=np.array([0,0,1.]) if down[2]==0 else np.array([1.,0,0]);a=np.cross(ref,down);b=np.cross(down,a)
  for k in (0,1,2,3):
   ang=k*np.pi/2;u=a*np.cos(ang)+b*np.sin(ang);v=-a*np.sin(ang)+b*np.cos(ang);rot=np.column_stack([u,v,down]);q=np.array(READY);err=[]
   for theta in (0.,OPEN_ANGLE/2,OPEN_ANGLE):
    c,s=np.cos(theta),np.sin(theta);rx=np.array([[1,0,0],[0,c,-s],[0,s,c]])
    p=HINGE+[dx,dy,dz]+RZ@rx@HAND_LOCAL
    q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(rot),q,base_pos=station.BASES['right'],iters=100);err.append((pe,re))
   worst=np.max(err,axis=0)
   if worst[0]<.002 and worst[1]<.02:print('FOUND',dx,dy,dz,down.tolist(),k,rot.tolist(),worst.tolist(),flush=True)
