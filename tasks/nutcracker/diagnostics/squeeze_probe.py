"""Diagnostic only: reset walnut between jaws and test robot handle squeeze."""
import sys,json,time,subprocess,traceback
from pathlib import Path
from dataclasses import replace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import imageio.v3 as iio
from inspect_robots import Scene
from nutcracker_task.embodiment import make_embodiment,FEEDBACK
from nutcracker_task.policy import NutcrackerPolicy
from nutcracker_task.config import INSTRUCTION,NUT_BENCH
MAX_STEPS=2400
out=Path(sys.argv[1]);seed=int(sys.argv[2]);out.mkdir(parents=True,exist_ok=False)
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:serial(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [serial(v) for v in x]
 return x
emb=make_embodiment();policy=NutcrackerPolicy();scene=Scene(id='nutcracker',instruction=INSTRUCTION)
emb.task=replace(emb.task,objects=tuple(replace(o,pos=tuple(map(float,NUT_BENCH+[0,.008 if o.name=='shell_a' else -.008,0]))) if o.name in ('shell_a','shell_b') else o for o in emb.task.objects))
writers={};rows=[];started=time.monotonic();error=None;last=None
try:
 obs=emb.reset(scene,seed=seed);policy.bind(emb.info);policy.reset(scene)
 from yam_arena.demo import ArmPlan
 from yam_arena import station
 policy.plans['right']=ArmPlan(policy.plans['right'].waypoints[1:],station.BASES['right'])
 policy.plans['left']=ArmPlan([],station.BASES['left']);policy.end=policy.plans['right'].duration
 for name,im in obs.images.items():
  h,w=im.shape[:2]
  writers[name]=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r','20','-i','-','-an','-c:v','libx264','-threads','1','-preset','veryfast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(out/(name+'.mp4'))],stdin=subprocess.PIPE)
  writers[name].stdin.write(im.tobytes());iio.imwrite(out/('initial-'+name+'.png'),im)
 for step in range(MAX_STEPS):
  action=policy.act(obs).actions[0];result=emb.step(action);obs=result.observation
  for name,writer in writers.items():writer.stdin.write(obs.images[name].tobytes())
  row=serial(dict(step=step,phase=action.meta.get('phase'),feedback=FEEDBACK,action=action.data));rows.append(row)
  if step%20==0:(out/'checkpoint.json').write_text(json.dumps(row))
  label=action.meta.get('phase')
  if label!=last or step%100==0:
   print('PROGRESS',step,action.meta.get('phase'),FEEDBACK.get('separation'),None,FEEDBACK.get('score'),flush=True)
   for name,im in obs.images.items():iio.imwrite(out/(f'{step:04d}-{name}.png'),im)
   last=label

  if result.terminated or policy.stop:break
except Exception as exc:
 error=type(exc).__name__+': '+str(exc);traceback.print_exc()
finally:
 for writer in writers.values():writer.stdin.close();writer.wait(timeout=30)
 result=serial(dict(seed=seed,score=emb.history.score,completed=error is None and emb.history.score==1.,steps=len(rows),wall_seconds=time.monotonic()-started,error=error,revision=emb.info.environment_revision,final=rows[-1] if rows else None))
 (out/'result.json').write_text(json.dumps(result,indent=2));(out/'trace.json').write_text(json.dumps(rows))
 print('RESULT',json.dumps(result),flush=True);emb.close()
