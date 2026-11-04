"""Sweep five loose machining-chip surrogates into the green end of the chip tray.

Use the handled brush, leave every chip inside the green collection bay, and
return the brush to its two supports. The machining fixture is unpowered.
"""
import numpy as np
from yam_arena.task_file import Box,TABLE_TOP
from chip_sweep.scoring import History,evaluate
SCENE='station';TOP=TABLE_TOP[SCENE]
INSTRUCTION='Use the brush to sweep all five metal chips into the green collection bay. Return the brush to its supports and open both grippers.'
OBJECTS=[Box('brush',pos=(.24,.27,TOP+.048),size=(.273,.115,.035),mass=.08),
 Box('chip_tray',pos=(.35,0,TOP),size=(.36,.20,.036)),
 Box('handle_support',pos=(.18,.27,TOP+.028),size=(.025,.055,.056),color=(.22,.28,.31)),
 Box('head_support',pos=(.34,.27,TOP+.019),size=(.025,.13,.038),color=(.22,.28,.31))]
CHIPS=[(.405,-.036,0.),(.428,.030,.3),(.455,-.012,-.4),(.475,.035,.6),(.485,-.038,-.2)]
for i,(x,y,yaw) in enumerate(CHIPS):OBJECTS.append(Box(f'chip_{i}',pos=(x,y,TOP+.015),size=(.012,.008,.006),yaw=yaw,mass=.006,color=(.6,.64,.67)))
HISTORY=History();SCORE_OVER='end';SUCCESS_HOLD_S=1.
def score(objects):return evaluate(objects,HISTORY,TOP)
def success(objects):return score(objects)>=1.
def demo():
 from chip_sweep.policy import make_plan
 return make_plan(TOP)
