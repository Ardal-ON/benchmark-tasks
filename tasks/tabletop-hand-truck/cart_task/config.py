"""Actual Wesco220646 operating envelope; internal proxy dimensions are estimates."""
import math,numpy as np
from yam_arena import station,ik
TABLE=station.TABLE_TOP;GROUND=station.FLOOR_Z;BASES=station.BASES
PLATFORM_SIZE=(.39,.86,.06);PLATFORM_CENTER=(-.125,0.,TABLE-.03)
PLATFORM_PARTS={'table':((.22,.86,.06),(-.21,0.,TABLE-.03)),'table_left_pad':((.17,.24,.06),(-.015,.31,TABLE-.03)),'table_right_pad':((.17,.24,.06),(-.015,-.31,TABLE-.03))}
ROLL_DIRECTION=-1.
SCALE=1.;MASS=CART_MASS=9*.45359237
HEIGHT=41*.0254;WIDTH=15.5*.0254;LENGTH=17.5*.0254
RADIUS=2.5*.0254;WHEEL_WIDTH=1.25*.0254;HUB_RADIUS=.045;BORE_RADIUS=.004
WHEEL_MASS=math.pi*WHEEL_WIDTH*((RADIUS**2-HUB_RADIUS**2)*1100+(HUB_RADIUS**2-BORE_RADIUS**2)*950)
AXLE_Y=LENGTH-12.5*.0254-RADIUS
FRAME_WIDTH=.28;RAIL_WIDTH=.022;RAIL_DEPTH=.028;RAIL_X=(FRAME_WIDTH-RAIL_WIDTH)/2
NOSE_DEPTH=12.5*.0254;NOSE_TOP=.012
AXLE=np.array([.50,0.,GROUND+RADIUS]);YAW=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
FRAME_POS=AXLE-YAW@np.array([0.,AXLE_Y,RADIUS]);FRAME_QUAT=(0.,0.,2**-.5,2**-.5)
GRIP_HEIGHT=HEIGHT-.0135;GRIP_X=0.
GRIP_LOCAL={'left':np.array([GRIP_X,.009,GRIP_HEIGHT]),'right':np.array([-GRIP_X,-.015,.5621])}
def rx(a):
 c,s=math.cos(a),math.sin(a);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def ry(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def rz(a):
 c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
GRIP_ROTS={}
for side,yaw,pitch in [('left',-.6,.3),('right',.6,.9)]:
 # Roll the wrist so the jaw closing axis is perpendicular to the crossbar.
 phi=math.atan(math.tan(yaw)*math.sin(pitch))
 GRIP_ROTS[side]=rz(yaw)@ry(pitch)@np.array([[0.,0.,1.],[0.,-1.,0.],[1.,0.,0.]])@rz(phi)
GRASPS={side:YAW@(p-np.array([0.,AXLE_Y,RADIUS]))+GRIP_ROTS[side][:,2]*(.008 if side=='left' else .012) for side,p in GRIP_LOCAL.items()}
Q_SEEDS={'left':[-.5,1.9,1.6,.03,.08,-1.74],'right':[.522251,2.899,1.827,.1704,-.0483,-1.1398]}
Q0={}
for side in ('left','right'):
 q,pe,re=ik.solve(AXLE+GRASPS[side],ik.matrix_to_quat_xyzw(GRIP_ROTS[side]),q_init=Q_SEEDS[side],base_pos=BASES[side],iters=500)
 if pe>.002 or re>.02:raise RuntimeError(f'{side} initial IK {pe}, {re}')
 Q0[side]=q
# A small downward position preload keeps the passive tires load-bearing.
TCP_PRELOAD_Z=-.020
Q_HOLD={}
for side in Q0:
 q,pe,re=ik.solve(AXLE+GRASPS[side]+[0,0,TCP_PRELOAD_Z],ik.matrix_to_quat_xyzw(GRIP_ROTS[side]),Q0[side],base_pos=BASES[side],iters=500)
 if pe>.002 or re>.02:raise RuntimeError(f'{side} hold IK {pe}, {re}')
 Q_HOLD[side]=q

# Gaps below the smallest bar cross-section retain contact as the wrist deflects.
# These are normalized position commands; stock50N finger force limits remain.
GRIP_ACTION={'left':.20,'right':.35}
