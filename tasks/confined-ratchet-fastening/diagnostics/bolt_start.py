"""Robot-only open-bench insertion milestone, not a full-score task demo."""
import sys,json,traceback,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import imageio.v3 as iio
from ratchet_task.embodiment import RatchetEmbodiment,FEEDBACK,ROOT
from ratchet_task.policy import BoltStartPolicy
from ratchet_task.config import INSTRUCTION
from inspect_robots import Scene
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
def serial(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if isinstance(v,dict):return {k:serial(x) for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [serial(x) for x in v]
 return v
emb=RatchetEmbodiment(task=str(ROOT/'task.py'),mechanisms=True,walls=False,device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360)
scene=Scene(id='loose-bolt-robot-milestone',instruction=INSTRUCTION)
rows=[];error=None;started=time.monotonic();last_phase=None;policy=BoltStartPolicy()
try:
 obs=emb.reset(scene,seed=17);policy.bind(emb.info);policy.reset(scene)
 for i in range(1000):
  action=policy.act(obs).actions[0];result=emb.step(action);obs=result.observation
  rows.append(serial(dict(step=i,phase=policy.phase,feedback=FEEDBACK,history=vars(emb.history))))
  if policy.phase!=last_phase:
   for name,im in obs.images.items():iio.imwrite(out/(f'{i:04d}-{policy.phase}-{name}.png'),im)
   print('ROBOT_PHASE',i,policy.phase,json.dumps(rows[-1]),flush=True);last_phase=policy.phase
  if i%10==0:iio.imwrite(out/(f'frame-{i:04d}.png'),obs.images['top_cam'])
  if emb.history.invalid:raise RuntimeError('Scorer invalid: '+emb.history.reason)
  if policy.stop:break
except Exception as e:
 error=type(e).__name__+': '+str(e);traceback.print_exc()
finally:
 result=dict(milestone_only=True,full_task_demo=False,completed=bool(policy.stop and error is None),error=error,elapsed_seconds=time.monotonic()-started,steps=len(rows),revision=emb.info.environment_revision,last=rows[-1] if rows else None)
 json.dump(result,open(out/'milestone-result.json','w'),indent=2);json.dump(rows,open(out/'milestone-trace.json','w'));print('BOLT_MILESTONE_COMPLETE',json.dumps(result),flush=True);emb.close()
