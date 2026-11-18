"""Tip the compact loaded trolley onto its wheels, roll it to the near mark, stand and release it.
A 300mm scale replica on the supplied station; this is not a full-size freight-handling trial.
The handle starts physically between the left jaws; the right arm is free.
"""
from yam_arena.task_file import Box
from cart_task.config import FRAME_POS,LOAD_POS,CART_MASS
SCENE='station'
OBJECTS=[Box('cart',tuple(map(float,FRAME_POS)),size=(.115625,.1125,.309),mass=CART_MASS),Box('load',tuple(map(float,LOAD_POS)),size=(.06,.035,.1),mass=.105,color=(.65,.43,.20)),Box('goal_mark',(.31,.1,.748),size=(.04,.20,.001),color=(.1,.6,.2))]
