"""Stock station and a dimensioned 300mm Magliner-inspired scale replica."""
import numpy as np,math
from yam_arena import station,ik
from yam_arena.demo import grip_rotation
TABLE=station.TABLE_TOP;BASES=station.BASES
SCALE=.300/1.2192;MASS=24*.45359237*SCALE**3;WHEEL_MASS=3*.45359237*SCALE**3
CART_MASS=MASS+350*math.pi*(.013**2-.004**2)*.060
RADIUS=.03125;WIDTH=.115625;AXLE_Y=.034375
AXLE=np.array([.46,.10,TABLE+RADIUS]);YAW=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
FRAME_POS=AXLE-YAW@np.array([0.,AXLE_Y,RADIUS]);FRAME_QUAT=(0.,0.,2**-.5,2**-.5)
LOAD_LOCAL=np.array([0.,-.0225,.052]);LOAD_POS=FRAME_POS+YAW@LOAD_LOCAL
GRASP=np.array([AXLE_Y,0.,.296-RADIUS]);a=.3;GRIP_ROT=np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])@np.array([[0.,0.,1.],[0.,-1.,0.],[1.,0.,0.]])
def ry(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
Q0,PE,RE=ik.solve(AXLE+GRASP,ik.matrix_to_quat_xyzw(GRIP_ROT),base_pos=BASES['left'],iters=500)
if PE>.002 or RE>.02:raise RuntimeError(f'initial IK {PE}, {RE}')
