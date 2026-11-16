"""Bench only: pin one lever and apply a diagnostic hinge motor; no robot success claim."""
import sys,json,math
from pathlib import Path
from dataclasses import replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np,imageio.v3 as iio
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY
from nutcracker_task.config import *
from nutcracker_task.embodiment import make_embodiment,FEEDBACK
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False);emb=make_embodiment();rows=[]
# Diagnostic initial nut centered in jaws; full task starts loose on table.
emb.task=replace(emb.task,objects=tuple(replace(o,pos=tuple(map(float,NUT_BENCH+[0,.008 if o.name=='shell_a' else -.008,0]))) if o.name in ('shell_a','shell_b') else o for o in emb.task.objects))
try:
 obs=emb.reset(Scene(id='nutcracker-bench',instruction=INSTRUCTION),seed=0)
 from pxr import UsdPhysics,Gf,Sdf
 import omni.physx
 print('API', [x for x in dir(emb.physics) if 'joint' in x.lower() or 'event' in x.lower()], [x for x in dir(emb.native) if 'joint' in x.lower() or 'event' in x.lower()],flush=True)
 s=emb._base.sim.stage;base='/World/envs/env_0/'

 j=UsdPhysics.RevoluteJoint.Get(s,base+'NutcrackerHinge');drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(.3);drive.CreateDampingAttr(.01);drive.CreateMaxForceAttr(10.);drive.CreateTargetPositionAttr(0.)
 for i in range(240):
  drive.GetTargetPositionAttr().Set(float(min(i/120,1)*42.))
  result=emb.step(Action(data=np.r_[READY,1.,READY,1.]));f=FEEDBACK
  rows.append(dict(step=i,separation=f['separation'],poses={k:v.tolist() for k,v in f['poses'].items()},rotations={k:v.tolist() for k,v in f['rotations'].items()}))
  if i in (0,60,120,180,239):
   print('BENCH',i,'separation',f['separation'],flush=True)
   for k,img in result.observation.images.items():iio.imwrite(out/f'{i:03}-{k}.png',img)
finally:
 (out/'trace.json').write_text(json.dumps(rows));emb.close()
