"""Offline nominal two-link planning against a recorded released first pose.

The recorded pose is a planning fixture, never credited as an assembly demo.
"""
from pathlib import Path
import argparse,json,numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from chain_task.geometry import *
from chain_task.motion import *
from collisions import *
ROOT=Path(__file__).resolve().parents[1];ap=argparse.ArgumentParser();ap.add_argument('--state-log',type=Path,required=True);a=ap.parse_args();d=json.loads([x for x in a.state_log.read_text().splitlines() if x.startswith('CHAIN_STATE ')][-1].split(' ',2)[2]);pp,pr=np.array(d['positions']['link1']),np.array(d['rotations']['link1']);rod=pp+pr@np.array([-.002,-.030,0.]);n=pr[:,0];yy=np.array([0,0,1.])-n*n[2];yy/=np.linalg.norm(yy);rd=np.column_stack([np.cross(yy,n),yy,n]);part=layout(17)['link0'];p,r=part['position'],part['rotation'];tcp,tool=grasp(p,r,1,'spine');off=r.T@(tcp-p);rel=tool.T@r;center=np.array([.30,.16,.96]);lt=pickup(p,r,1,'spine')+[Target(4,center+rd@off,rd@rel.T,0.,'orient child'),Target(5,center+rd@off,rd@rel.T,0.,'counterhold wait')]
for seconds,local,label in [(4,[.022,.020,.070],'normal approach'),(4,[.022,.020,0.],'align gate'),(2,[.022,.020,0.],'settle'),(4,[.007,.020,0.],'press'),(3,[.007,.024,0.],'round tip'),(3,[.002,.024,0.],'enter parent'),(3,[.002,.024,0.],'close')]:lt.append(Target(seconds,rod-rd@np.array(local)+rd@off,rd@rel.T,0.,label))
left=Plan(lt,'left');parent_tool=pr@np.diag([1.,-1.,-1.])@rx(np.deg2rad(-15));parent_tcp=pp+pr@np.array([-.016,0,0.])-parent_tool[:,2]*.004;rp=[Target(3,parent_tcp-parent_tool[:,2]*.06,parent_tool,.18,'parent approach'),Target(3,parent_tcp,parent_tool,.18,'parent entry'),Target(2,parent_tcp,parent_tool,0.,'parent pinch'),Target(600,parent_tcp,parent_tool,0.,'counterhold')];right=Plan(rp,'right');errors=[];hits=[];parent_obs=asset('parent',ROOT/'assets/frame1.usda',pp,pr)+asset('parent_gate',ROOT/'assets/gate.usda',np.array(d['positions']['gate1']),np.array(d['rotations']['gate1']))
for t in np.arange(.1,left.duration+.001,.1):
 try:la,label=left.action7(t);ra=np.r_[READY,1.] if t<11 else right.action7(t-11)[0]
 except RuntimeError as exc:errors.append((float(t),str(exc)));break
 ls=robot(la[:6],max(la[6],.0045/.0939),'left');rs=robot(ra[:6],max(ra[6],.0045/.0939),'right');ht,hr=ik.tcp_pose(la[:6],station.BASES['left']);br=hr@rel;bp=ht-br@off
 if t<6.:bp,br=p,r
 child_obs=asset('child',ROOT/'assets/frame0.usda',bp,br)+asset('child_gate',ROOT/'assets/gate.usda',bp+br@GATE_OFFSET,br);obs=static_scene(ROOT)+parent_obs+child_obs;allowed_l={(tip,'child') for tip in ['tip_left','tip_right']} if t>=4.5 else set();allowed_r={(tip,'parent') for tip in ['tip_left','tip_right']} if t>=16.5 else set();lh,rh=against(ls,obs,allowed_l),against(rs,obs,allowed_r);inter=cross(ls,rs)
 if lh or rh or inter:hits.append((float(t),label,lh,rh,inter))
print(json.dumps(dict(planning_fixture_source=a.state_log.name,errors=errors,left_errors=[left.worst_position_error,left.worst_rotation_error],right_errors=[right.worst_position_error,right.worst_rotation_error],hits=hits),indent=2))
