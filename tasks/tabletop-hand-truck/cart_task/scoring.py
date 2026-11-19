"""Numeric progress rules, independent of the simulator and controller."""
import math

def rolling_complete(supported, distance, turns, radius):
    return all(supported[n] > .125 and distance[n] > .125 and turns[n] > .8
               and abs(turns[n]*radius-supported[n]) < .025 for n in ('wheel_left','wheel_right'))

def stage_score(*, failed, tipped, rolling, axle, angle, clear, stable, hold):
    candidate = not failed and tipped and rolling and .315 < axle[0] < .385 and abs(axle[1]) < .04
    hold = hold + 1 if candidate and angle < math.radians(2) and clear and stable else 0
    score = 0. if failed else (1. if hold >= 20 else (.5 if candidate else (.25 if tipped else 0.)))
    return score,hold


def pose_stable(previous_position,position,previous_rotation,rotation,dt=.05):
    """Bound translation and rotation: an in-place spin is not a parked finish."""
    import numpy as np
    displacement=np.linalg.norm(np.asarray(position)-previous_position)
    relative=np.asarray(previous_rotation).T@np.asarray(rotation)
    angle=math.acos(float(np.clip((np.trace(relative)-1)/2,-1,1)))
    return displacement/dt<.01 and angle/dt<math.radians(2.)


def load_bearing_contact(separation,impulse,normal):
    """Require near contact and nonzero normal-impulse magnitude, for either actor order."""
    return separation<=.0005 and abs(sum(float(i)*float(n) for i,n in zip(impulse,normal)))>1e-7


def supported_motion(ages,distance_delta,turn_delta):
    """Use identical intervals for both wheels; freshness allowance is four ticks."""
    names=('wheel_left','wheel_right')
    supported=all(ages[n]<=4 for n in names)
    return ({n:distance_delta[n] if supported else 0. for n in names},
            {n:turn_delta[n] if supported else 0. for n in names})


def support_lost(wheel_ages,toe_age,upright):
    """An upright hand truck can stand on its nose plate with unloaded wheels."""
    return all(age>120 for age in wheel_ages.values()) and not (upright and toe_age<=4)
