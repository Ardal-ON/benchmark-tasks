import numpy as np
TABLE=.7475
HINGE=np.array([.32,-.04,TABLE+.12])
NUT=np.array([.20,.13,TABLE+.019])
NUT_BENCH=HINGE+[-.040,0,.012]
FIGURE_YAW=np.pi/2
JAW_PIVOT=np.array([0.,.020,0.])
LEVER_LOCAL=np.array([0.,-.120,.040])
OPEN_ANGLE=-.60
CONTROL_HZ=20
PHYSICS_HZ=480
INSTRUCTION='Lift the lever on the nutcracker’s back to open its mouth, put the walnut in the mouth, then press the lever to crack it.'
