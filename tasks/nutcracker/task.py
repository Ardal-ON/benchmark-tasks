"""A hinged nutcracker and a two-piece breakable walnut on the YAM table."""
from yam_arena.task_file import Box
from nutcracker_task.config import *
SCENE='station'
SCORE_OVER='end'
HISTORY=None
OBJECTS=[Box('fixture',pos=(float(HINGE[0]),float(HINGE[1]),TABLE),size=(.13,.13,.012)),Box('upper',pos=tuple(map(float,HINGE)),yaw=FIGURE_YAW,size=(.1,.08,.24),mass=.45),Box('lower',pos=tuple(map(float,HINGE)),yaw=FIGURE_YAW,size=(.07,.19,.04),mass=.16),Box('shell_a',pos=tuple(map(float,NUT+[0,0,.008])),size=(.035,.016,.036),mass=.007),Box('shell_b',pos=tuple(map(float,NUT+[0,0,-.008])),size=(.035,.016,.036),mass=.007)]
def score(objects):return 0. if HISTORY is None else HISTORY.score
def success(objects):return HISTORY is not None and HISTORY.score==1.
