"""Insert and fasten a bracket bolt with a socket ratchet in limited swing space."""
from yam_arena.task_file import Box
from ratchet_task.config import *
SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=.05
HISTORY=None
OBJECTS=[Box('fixture',pos=(AXIS[0],AXIS[1],TABLE),size=(.18,.16,.093)),
 Box('entry_stop',pos=(AXIS[0],AXIS[1],TABLE),size=(.009,.009,.001)),
 Box('bolt_holder',pos=(BOLT_START[0],BOLT_START[1],TABLE),size=(.044,.044,.022)),
 Box('bolt',pos=tuple(BOLT_START),size=(.015,.015,.022),mass=BOLT_MASS),
 Box('handle',pos=tuple(TOOL_START),size=(.203,.039,.031),mass=.378),
 Box('drive',pos=tuple(TOOL_START),size=(.021,.021,.035),mass=.086),
 Box('tool_cradle',pos=(TOOL_START[0]-.08,TOOL_START[1],TABLE),size=(.23,.065,.03))]
def score(objects):return 0. if HISTORY is None else HISTORY.score
def success(objects):return HISTORY is not None and HISTORY.score==1.
