"""Numeric end-state score and measured contact provenance."""
from dataclasses import dataclass,field
import numpy as np
@dataclass
class History:
 touched:set=field(default_factory=set)
 invalid:bool=False
 open_clear:bool=False
 def contact(self,a,b):
  names=(a.lower(),b.lower())
  for i in range(5):
   key=f'chip_{i}'
   if not any('/'+key+'/' in n or n.endswith('/'+key) for n in names):continue
   if any('/brush/' in n or n.endswith('/brush') for n in names):self.touched.add(key)
   if any('left_robot' in n or 'right_robot' in n for n in names):self.invalid=True
 def update(self,objects,top):
  for k in ['brush']+[f'chip_{i}' for i in range(5)]:
   s=objects[k]
   if not np.all(np.isfinite(s.pos)) or s.low[2]<top-.02 or not -.1<s.pos[0]<.95 or abs(s.pos[1])>.62:self.invalid=True

def collected(s,top):
 return bool(.185<s.low[0] and s.high[0]<.305 and -.085<s.low[1] and s.high[1]<.085 and abs(s.low[2]-(top+.012))<.003 and s.resting())
def parked(s,top):
 return bool(np.linalg.norm(s.pos[:2]-(.22875,.27))<.025 and abs(s.low[2]-(top+.038))<.006 and s.high[2]-s.low[2]<.045 and s.resting())
def evaluate(objects,h,top):
 if h.invalid:return 0.
 count=sum(f'chip_{i}' in h.touched and collected(objects[f'chip_{i}'],top) for i in range(5))
 return count*.16+(.2 if count==5 and parked(objects['brush'],top) and h.open_clear else 0.)
