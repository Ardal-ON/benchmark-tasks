"""Five colored blocks travel left on a striped conveyor. Place each in the matching bin."""
import math
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.constants import FINGER_OPEN
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BELT_TOP = TOP + 0.02
SPEED = 0.012
COLORS = {"red": (0.85, 0.08, 0.08), "blue": (0.05, 0.20, 0.85)}
STARTS = [("red_1", 0.30), ("blue_1", -0.30), ("red_2", 0.10), ("blue_2", -0.50), ("red_3", -0.10)]
BINS = {"red": (0.15, 0.22), "blue": (0.15, -0.22)}
OBJECTS = [Box("belt", pos=(0.34, 0, TOP+0.01), size=(0.12, 1.20, 0.02), color=(0.08, 0.08, 0.08))]
for name, y in STARTS:
    OBJECTS.append(Box(name, pos=(0.34, y, BELT_TOP+0.02), size=(0.04,)*3, mass=0.05, color=COLORS[name.split("_")[0]]))
# Thin fixed-collider stripes are animated by the custom embodiment.
for i in range(10):
    OBJECTS.append(Box(f"stripe_{i}", pos=(0.34, -0.54+i*0.12, BELT_TOP+0.0003), size=(0.12, 0.008, 0.0006), color=(0.95, 0.75, 0.10)))
for color, (x, y) in BINS.items():
    rgb = COLORS[color]
    OBJECTS.extend([
        Box(f"{color}_floor", pos=(x, y, TOP+0.005), size=(0.14, 0.20, 0.01), color=rgb),
        Box(f"{color}_front", pos=(x-0.065, y, TOP+0.03), size=(0.01, 0.20, 0.06), color=rgb),
        Box(f"{color}_back", pos=(x+0.065, y, TOP+0.03), size=(0.01, 0.20, 0.06), color=rgb),
        Box(f"{color}_left", pos=(x, y+0.095, TOP+0.03), size=(0.12, 0.01, 0.06), color=rgb),
        Box(f"{color}_right", pos=(x, y-0.095, TOP+0.03), size=(0.12, 0.01, 0.06), color=rgb),
    ])


def placed(obj, xy):
    x, y = xy
    return bool(obj.low[0] >= x-0.06 and obj.high[0] <= x+0.06 and obj.low[1] >= y-0.09 and obj.high[1] <= y+0.09 and abs(obj.low[2]-(TOP+0.01)) < 0.012 and obj.resting())


def success(objects):
    return all(placed(objects[name], BINS[name.split("_")[0]]) for name, _ in STARTS)


def score(objects):
    total = 0.0
    for name, _ in STARTS:
        obj = objects[name]
        xy = BINS[name.split("_")[0]]
        if placed(obj, xy):
            total += 1.0
        else:
            lift = min(1.0, max(0.0, (obj.low[2]-BELT_TOP)/0.04))
            distance = math.hypot(obj.pos[0]-xy[0], obj.pos[1]-xy[1])
            progress = max(0.0, min(1.0, 1.0-distance/0.45))
            total += lift * (0.2 + 0.78*progress)
    return total / len(STARTS)


def demo():
    # Open-loop approximation; validate after any change to traction or timing.
    plans = {"left": [], "right": []}
    counts = {"left": 0, "right": 0}
    for name, y in STARTS:
        color = name.split("_")[0]
        arm = "left" if color == "red" else "right"
        order = counts[arm]
        delay = 8.0 if name == "blue_2" else 0.0
        if delay:
            bx, by = BINS[color]
            plans[arm].append(Waypoint(delay, (bx, by-0.052, TOP+0.16), FINGER_OPEN, "wait for block"))
        pick = (0.34, y + SPEED*(3.5 + order*9.6 + delay), BELT_TOP+0.02)
        bx, by = BINS[color]
        place = (bx, by + (order-1)*0.052, TOP+0.095)
        plans[arm].extend(pick_place_waypoints(pick, place, travel_z=TOP+0.16))
        counts[arm] += 1
    return plans
