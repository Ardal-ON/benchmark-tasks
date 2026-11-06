"""Record a full normal-action confined-ratchet demo on three native cameras."""
import sys,json,traceback,time,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import imageio.v3 as iio
from ratchet_task.embodiment import make_embodiment,FEEDBACK,ROOT
from ratchet_task.policy import FastenPolicy
from ratchet_task.config import INSTRUCTION,MAX_STEPS
from inspect_robots import Scene
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
def serial(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if isinstance(v,dict):return {k:serial(x) for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [serial(x) for x in v]
 return v
emb=make_embodiment(task=str(ROOT/'task.py'),device='cpu',top_cam='demo',control_hz=20,cam_width=640,cam_height=360)
scene=Scene(id='confined-ratchet-demo',instruction=INSTRUCTION)
full=True
tool_only=False
rows=[];error=None;started=time.monotonic();last_phase=None;last_label=None;policy=FastenPolicy()
writers={}
try:
 obs=emb.reset(scene,seed=17);policy.bind(emb.info);policy.reset(scene)
 if full:
  for name,im in obs.images.items():
   height,width=im.shape[:2]
   writers[name]=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}','-r','20','-i','-','-an','-c:v','libx264','-threads','1','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out/(name+'.mp4'))],stdin=subprocess.PIPE)
   writers[name].stdin.write(im.tobytes())
 for i in range(MAX_STEPS if full else 1200):
  action=policy.act(obs).actions[0];result=emb.step(action);obs=result.observation
  for name,writer in writers.items():writer.stdin.write(obs.images[name].tobytes())
  rows.append(serial(dict(step=i,phase=policy.phase,feedback=FEEDBACK,history=vars(emb.history))))
  label=action.meta.get('waypoint','unknown')
  if policy.phase!=last_phase or label!=last_label:
   for name,im in obs.images.items():iio.imwrite(out/(f'{i:04d}-{policy.phase}-{label.replace(" ","-")}-{name}.png'),im)
   print('ROBOT_PHASE',i,policy.phase,json.dumps(rows[-1]),flush=True);last_phase=policy.phase;last_label=label
  if i%10==0:iio.imwrite(out/(f'frame-{i:04d}.png'),obs.images['top_cam'])
  if emb.history.invalid:raise RuntimeError('Scorer invalid: '+emb.history.reason)
  if policy.stop or result.terminated or (tool_only and policy.phase=='tool_approach'):break
except Exception as e:
 error=type(e).__name__+': '+str(e);traceback.print_exc()
finally:
 for writer in writers.values():
  writer.stdin.close();writer.wait(timeout=30)
 result=dict(full_task_demo=True,walls=emb.walls,mechanisms=emb.mechanisms,max_steps=MAX_STEPS,seed=17,completed=bool(error is None and (emb.history.score==1. if full else (policy.phase=='tool_approach' if tool_only else policy.stop))),error=error,score=getattr(getattr(emb,"history",None),"score",0.),elapsed_seconds=time.monotonic()-started,steps=len(rows),revision=emb.info.environment_revision,last=rows[-1] if rows else None)
 json.dump(result,open(out/'demo-result.json','w'),indent=2);json.dump(rows,open(out/'demo-trace.json','w'));print('RATCHET_DEMO_COMPLETE',json.dumps(result),flush=True);emb.close()
