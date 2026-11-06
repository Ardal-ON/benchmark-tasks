"""SI units; a small instrument bracket, not a service-torque connection."""
import numpy as np
TABLE = .7475
AXIS = np.array([.34, 0., TABLE + .093])
PITCH = .00125
BOLT_LENGTH = .016
BRACKET_THICKNESS = .003
THREAD_ENTRY_Z = AXIS[2]-BRACKET_THICKNESS
START_GAP = BOLT_LENGTH-BRACKET_THICKNESS-.0018
REQUIRED_TURNS = 8.
HEAD_AF = .013
HEAD_HEIGHT = .0053
BOLT_MASS = .014
HANDLE_MASS = .3954052755732919 + .004  # Source shell plus two light rubber grip pads.
PAWL_MASS = .02835521567254335
SOCKET_MASS = .040
CAPTURE_OFFSET = .0007
CAPTURE_TILT = np.deg2rad(5)
THREAD_START = np.deg2rad(15)
TARGET_TORQUE = (.4, .6)
MAX_TORQUE = 1.
SOLVER_POSITION = 64
SOLVER_VELOCITY = 16
PHYSICS_HZ = 480
CONTROL_HZ = 20
MAX_STEPS = 10000
BOLT_START = np.array([.20, .17, TABLE+.022])
TOOL_START = np.array([.30,-.21,TABLE+.036])
INSTRUCTION = ('Insert the loose bolt through the bracket and start its thread. '
 'Use the socket ratchet to tighten it with short back-and-forth strokes inside the walls, '
 'Tighten to 0.5 N m (within 0.1 N m), then remove the tool, set it in its cradle and release both hands.')
