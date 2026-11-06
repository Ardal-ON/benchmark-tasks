"""SI units; a small instrument bracket, not a service-torque connection."""
import numpy as np
TABLE = .7475
AXIS = np.array([.34, 0., TABLE + .093])
PITCH = .00125
BOLT_LENGTH = .016
HEAD_AF = .013
HEAD_HEIGHT = .0053
BOLT_MASS = .014
CAPTURE_OFFSET = .0005
CAPTURE_TILT = np.deg2rad(5)
THREAD_START = np.deg2rad(30)
TARGET_TORQUE = (.4, .6)
MAX_TORQUE = 1.
PHYSICS_HZ = 480
CONTROL_HZ = 20
MAX_STEPS = 7200
BOLT_START = np.array([.20, .17, TABLE+.022])
TOOL_START = np.array([.30,-.21,TABLE+.034])
INSTRUCTION = ('Insert the loose bolt through the bracket and start its thread. '
 'Use the socket ratchet to tighten it with short back-and-forth strokes inside the walls, '
 'then remove the tool, set it in its cradle and release both hands.')
