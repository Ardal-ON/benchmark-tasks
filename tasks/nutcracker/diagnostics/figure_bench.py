"""Development fixture only: motor articulates the hand, and the walnut is loaded at reset.

This is a mechanism test, not robot control or a full task demonstration.
"""
import sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np,imageio.v3 as iio
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY
from nutcracker_task.config import *
from nutcracker_task.embodiment import make_embodiment,FEEDBACK
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False);emb=make_embodiment();rows=[]
try:
 obs=emb.reset(Scene(id='figure-bench',instruction=INSTRUCTION),seed=0)
 from pxr import UsdPhysics,Gf
 s=emb._base.sim.stage;base='/World/envs/env_0/';j=UsdPhysics.RevoluteJoint.Get(s,base+'NutcrackerHinge')
 drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(float(__import__('os').environ.get('JAW_DRIVE_STIFFNESS','.3')));drive.CreateDampingAttr(.01);drive.CreateMaxForceAttr(12.);drive.CreateTargetPositionAttr(-38.)
 for phase,steps,target in [('open',100,-38.),('load',1,-38.),('press',160,3.),('release',60,3.)]:
  if phase=='load':
   # Explicit test-fixture placement, solely for this diagnostic.
   for n,sgn in [('shell_a',1),('shell_b',-1)]:
    obj=emb._base.scene[n];pose=obj.data.default_root_pose.torch.clone();pose[:,:3]=emb._torch.tensor(NUT_BENCH+[0,0,sgn*.008],dtype=pose.dtype,device=pose.device);pose[:,3:]=emb._torch.tensor([math.sqrt(.5),0,0,math.sqrt(.5)],dtype=pose.dtype,device=pose.device);obj.write_root_pose_to_sim_index(root_pose=pose);obj.write_root_velocity_to_sim_index(root_velocity=emb._torch.zeros_like(obj.data.default_root_vel.torch))
   # Reset seam after loading; test-only procedure.
   s.RemovePrim(base+'WalnutSeam');emb._joints_seam_only()
  for i in range(steps):
   drive.GetTargetPositionAttr().Set(float(-38.+41.*i/max(steps-1,1) if phase=='press' else target));r=emb.step(Action(data=np.r_[READY,1.,READY,1.]));f=FEEDBACK
   rows.append(dict(phase=phase,step=i,separation=f['separation'],broken=f['broken'],valid_break=f['valid_break'],contact_peak=f['contact_peak'],poses={k:v.tolist() for k,v in f['poses'].items()},rotations={k:v.tolist() for k,v in f['rotations'].items()}))
   if i==steps-1:
    print('PHASE',phase,'break',f['broken'],'valid',f['valid_break'],'separation',f['separation'],flush=True)
    for name,img in r.observation.images.items():iio.imwrite(out/f'{phase}-{name}.png',img)
finally:
 (out/'trace.json').write_text(json.dumps(rows));emb.close()
