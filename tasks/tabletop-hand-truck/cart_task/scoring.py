"""Numeric progress rules, independent of the simulator and controller."""
import math

def rolling_complete(supported, distance, turns, radius):
    return all(supported[n] > .125 and distance[n] > .125 and turns[n] > 3.5
               and abs(turns[n]*radius-distance[n]) < .025 for n in ('wheel_left','wheel_right'))

def stage_score(*, failed, tipped, rolling, axle, angle, clear, stable, hold):
    candidate = not failed and tipped and rolling and .285 < axle[0] < .335 and abs(axle[1]-.1) < .04
    hold = hold + 1 if candidate and angle < math.radians(2) and clear and stable else 0
    score = 0. if failed else (1. if hold >= 20 else (.5 if candidate else (.25 if tipped else 0.)))
    return score,hold
