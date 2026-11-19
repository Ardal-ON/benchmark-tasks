"""Tip the empty full-size floor cart, pull150mm backward, stand and release it.
The user-authorized reduced table supports both unchanged calibrated robot bases.
"""
from yam_arena.task_file import Box
from cart_task.config import *
SCENE='station'
OBJECTS=[Box('cart',tuple(map(float,FRAME_POS)),size=(WIDTH,LENGTH,HEIGHT),mass=CART_MASS),Box('goal_mark',(.35,0.,GROUND+.0005),size=(.06,.55,.001),color=(.1,.6,.2))]
