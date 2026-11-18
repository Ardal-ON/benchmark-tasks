"""Scripted Cartesian poses with explicit orientations for side grips."""
from dataclasses import dataclass,replace
import numpy as np
from yam_arena import ik
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix
from inspect_robots_yam.packing import split
from inspect_robots.types import Action,ActionChunk
from yam_arena.constants import READY
from flange_assembly.specs import MOVING_BBOX,GRASP_LOCAL_Z,LEFT_TARGET_X,HOLE_COMPONENT


CURRENT_PHASES={}
CONTROL_STATE={}

@dataclass
class PoseWaypoint(Waypoint):
    rotation: tuple | None = None  # 3x3 matrix, row-major, or None for downward grip


class PosePlan(ArmPlan):
    def target(self, t):
        original_t = t
        def adjusted(pos, rot, finger, label):
            callback = getattr(self, "adjust", None)
            return callback(original_t, pos, rot, finger, label) if callback else (pos, rot, finger, label)
        prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
        for w in self.waypoints:
            goal = np.asarray(w.pos, dtype=float)
            rot = np.asarray(w.rotation).reshape(3, 3) if getattr(w, 'rotation', None) is not None else grip_rotation(w.yaw)
            if t < w.duration:
                s = ik.smoothstep(t / w.duration)
                return adjusted(prev_pos+s*(goal-prev_pos), ik.slerp_matrix(prev_rot,rot,s),
                                prev_finger+s*(w.finger-prev_finger), w.label)
            t -= w.duration
            prev_pos, prev_rot, prev_finger = goal, rot, w.finger
        return adjusted(prev_pos, prev_rot, prev_finger, 'done')


class PanelPolicy(YamScriptedPolicy):
    def __init__(self,release_probe=False,grasp_probe=False,transfer_probe=False,**kwargs):
        super().__init__(**kwargs);self.release_probe=bool(release_probe);self.grasp_probe=bool(grasp_probe);self.transfer_probe=bool(transfer_probe)
    def act(self, observation):
        left, right = split(observation.state['joint_pos'])
        self._observed_joints = {'left': left[:6], 'right': right[:6]}
        CONTROL_STATE.clear();CONTROL_STATE['observation_time']=float(observation.state_time)
        for side,values in (('left',left),('right',right)):
            tcp,hand_R=ik.tcp_pose(values[:6],self._bases[side]);CONTROL_STATE[side]={'joints':np.asarray(values).tolist(),'tcp':tcp.tolist(),'rotation':hand_R.tolist()}
        chunk=super().act(observation)
        action=chunk.actions[0];target=np.asarray(action.data).copy();actual=np.r_[left,right]
        self._joint_integral=np.clip(self._joint_integral+.5*self._dt*(target-actual),-.08,.08)
        self._joint_integral[[6,13]]=0.
        chunk=ActionChunk(actions=[Action(data=target+self._joint_integral,meta={**action.meta,'phases':CURRENT_PHASES.copy()})],control_hz=1/self._dt)
        if not self.release_probe:return chunk
        t=float(observation.state_time)+self._dt
        data=np.asarray(chunk.actions[0].data).copy();data[7:]=np.r_[READY,1.]
        if t>=22.:
            if self._probe_from is None:self._probe_from=left[:6].copy()
            u=ik.smoothstep(np.clip((t-23.)/4.,0.,1.))
            data[:6]=self._probe_from+u*(np.asarray(READY)-self._probe_from);data[6]=1.
        return ActionChunk(actions=[Action(data=data,meta={'release_probe':True,'request_stop':t>=35.})],control_hz=1/self._dt)

    def reset(self, scene):
        self._probe_from=None;self._joint_integral=np.zeros(14);self._pin_adjust_start={};self._grasp_offsets={};self._pin_rotations={};self._released=set();self._push_data={};self._withdraw_start={};self._pick_offsets={};self._pick_start={};CURRENT_PHASES.clear()
        plan = self.plan_for(scene.instruction, scene.target)
        if self.grasp_probe:
            plan['right']=plan['right'][:6]  # preserve full wait and left-arm context
        if self.transfer_probe:
            plan['right']=plan['right'][:7]+[replace(plan['right'][6],duration=20.)]
        self._plans = {arm: PosePlan(list(waypoints), self._bases[arm]) for arm, waypoints in plan.items()}
        self._end = max((p.duration for p in self._plans.values()), default=0.)
        self._offsets = {}
        self._upper_yaw = 0.
        self._pin_holes = {}
        self._align_offset = np.zeros(3)
        self._align_rot = np.zeros(3)
        self._insertion_offsets = {'second': np.zeros(3)}
        for side, p in self._plans.items():
            p.adjust = lambda t, pos, rot, finger, label, side=side: self._feedback(
                side, t, pos, rot, finger, label)

    def _feedback(self, side, t, pos, rot, finger, label):
        from flange_assembly.embodiment import demo_states
        states=demo_states();CURRENT_PHASES[side]=label
        if not states:return pos,rot,finger,label
        if side=='left' and label in ('hold flange','done'):
            # Bare pipe remains on its V rests; correct axial/clocking drift.
            target=np.array([LEFT_TARGET_X,.06,self._task.module.TOP+.17])
            current_R=quat_xyzw_to_matrix(states['upper_flange'].quat_xyzw)
            root=np.asarray(states['upper_flange'].pos)-current_R@np.array(MOVING_BBOX)
            grasp_axis=root+current_R@np.array([0.,0.,GRASP_LOCAL_Z])
            error=target-grasp_axis
            desired=axis_angle_matrix(np.array([0.,1.,0.]),-np.pi/2)
            angular=ik._rot_error(desired,current_R)
            self._align_offset += np.clip(.003*error,-.00002,.00002)
            np.clip(self._align_offset,-.025,.025,out=self._align_offset)
            self._align_rot += np.clip(.03*angular,-.0004,.0004)
            np.clip(self._align_rot,-.12,.12,out=self._align_rot)
            angle=np.linalg.norm(self._align_rot)
            if angle>1e-8:rot=axis_angle_matrix(self._align_rot/angle,angle)@rot
            return pos+self._align_offset+.7*error,rot,finger,label
        if side=='right':
            if self.grasp_probe and label=='done':return pos,rot,finger,label
            second='second' in label or label=='done'
            key='second' if second else 'first'
            pins=[states['pin_1'],states['pin_2']]
            pickup=any(word in label for word in ('above pin','second pin','beside pin','enter pin','grasp pin','lift pin')) and not any(word in label for word in ('insert','seat','release','back off','push','clear','withdraw'))
            if pickup:
                pin=states['pin_1' if second else 'pin_2']
                if key not in self._pick_offsets:
                    axis=quat_xyzw_to_matrix(pin.quat_xyzw)[:,2];head=np.asarray(pin.pos)+.025*axis
                    nominal=np.array([.29125,-.30 if second else -.18,self._task.module.TOP+.191])
                    self._pick_offsets[key]=head-nominal;self._pick_start[key]=t
                    print('PICKUP_CENTRE_CORRECTION',key,self._pick_offsets[key].tolist(),flush=True)
                blend=ik.smoothstep(np.clip((t-self._pick_start[key])/3.,0.,1.))
                return pos+blend*self._pick_offsets[key],rot,finger,label
            if label in ('release pin','release second pin') and not self.release_probe and key not in self._released:
                from flange_assembly.geometry import count_engaged_bolts
                count=count_engaged_bolts(states['lower_flange'],states['upper_flange'],pins)
                if count<(2 if second else 1):raise RuntimeError(f'Bolt shaft not through both flanges before release: {count}')
                self._released.add(key)
            if 'push' in label and 'close' not in label:
                pin=states['pin_1' if second else 'pin_2']
                if key not in self._push_data:
                    # Head centre is25mm behind the bounding-box centre.
                    axis=quat_xyzw_to_matrix(pin.quat_xyzw)[:,2]
                    head=np.asarray(pin.pos)+.025*axis
                    body_R=quat_xyzw_to_matrix(states['upper_flange'].quat_xyzw)
                    face=np.asarray(states['upper_flange'].pos)-body_R@np.array(MOVING_BBOX)
                    hole_y=.06+(HOLE_COMPONENT if second else -HOLE_COMPONENT)
                    hole_z=self._task.module.TOP+.17+HOLE_COMPONENT
                    delta=np.array([face[0]-.3125,head[1]-hole_y,head[2]-hole_z])
                    self._push_data[key]=(t,delta)
                start,delta=self._push_data[key]
                if 'clear' in label and not self.release_probe:
                    from flange_assembly.geometry import score
                    value=score(states['lower_flange'],states['upper_flange'],pins)
                    if value<(1. if second else .7)-1e-9:raise RuntimeError(f'Bolt not seated after head push: {value}')
                if 'approach' in label:
                    blend=ik.smoothstep(np.clip((t-start)/2.,0.,1.))
                    return pos+(1-blend)*self._pin_holes[key]+blend*delta,ik.slerp_matrix(self._pin_rotations[key],rot,blend),finger,label
                return pos+delta,rot,finger,label
            if 'withdraw' in label and key in self._push_data:
                if key not in self._withdraw_start:self._withdraw_start[key]=t
                blend=ik.smoothstep(np.clip((t-self._withdraw_start[key])/3.,0.,1.))
                return pos+(1-blend)*self._push_data[key][1],rot,finger,label
            if label=='done' and key in self._released:return pos,rot,finger,label
            post_pick=any(word in label for word in ('hole','insert','seat','release','back off','push','clear','withdraw')) or label=='done'
            if post_pick:
                pin=states['pin_1' if second else 'pin_2']
                if key not in self._pin_holes:
                    tcp,hand_R=ik.tcp_pose(self._observed_joints['right'],self._bases['right'])
                    offset=np.asarray(pin.pos)-tcp
                    pin_axis=quat_xyzw_to_matrix(pin.quat_xyzw)[:,2];desired_axis=np.array([-1.,0.,0.])
                    if np.dot(pin_axis,desired_axis)<0:raise RuntimeError('Bolt grasp is reversed relative to insertion direction')
                    if not self.release_probe and pin.pos[2]<self._task.module.TOP+.215:raise RuntimeError('Bolt did not lift from its stand')
                    cross=np.cross(pin_axis,desired_axis);sine=np.linalg.norm(cross);angle=np.arctan2(sine,np.dot(pin_axis,desired_axis))
                    if not self.release_probe and angle>np.deg2rad(10):raise RuntimeError(f'Bolt tilt after lift is too large: {np.rad2deg(angle):.2f}deg')
                    correction=axis_angle_matrix(cross/sine,angle) if sine>1e-8 else np.eye(3)
                    goal_R=correction@hand_R;self._pin_rotations[key]=goal_R
                    target_offset=goal_R@hand_R.T@offset
                    print('BOLT_GRASP_OFFSET',key,round(t,3),offset.tolist(),flush=True)
                    if not self.release_probe and np.linalg.norm(offset)>.08:raise RuntimeError('Bolt was not held after pickup')
                    # Bolt bbox centre:head bears on loose14mm plate atx=.2985.
                    hole_y=.06+(HOLE_COMPONENT if second else -HOLE_COMPONENT)
                    hole_z=self._task.module.TOP+.17+HOLE_COMPONENT
                    target=np.array([.31025,hole_y,hole_z])
                    endpoint=np.array([.292,hole_y,hole_z-.002])
                    self._pin_holes[key]=target-endpoint-target_offset;self._pin_adjust_start[key]=t
                if not self.release_probe and key not in self._released and any(word in label for word in ('hole','insert','seat')):
                    tcp,current_hand_R=ik.tcp_pose(self._observed_joints['right'],self._bases['right'])
                    # Compare to the measured rigid grasp, not to a desired object pose.
                    if not hasattr(self,'_grasp_offsets'):self._grasp_offsets={}
                    if key not in self._grasp_offsets:self._grasp_offsets[key]=current_hand_R.T@(np.asarray(pin.pos)-tcp)
                    if t-self._pin_adjust_start[key]>1. and np.linalg.norm(current_hand_R.T@(np.asarray(pin.pos)-tcp)-self._grasp_offsets[key])>.025:raise RuntimeError('Bolt slipped from gripper before seating')
                blend=ik.smoothstep(np.clip((t-self._pin_adjust_start[key])/5.,0.,1.))
                return pos+(1-blend)*self._pick_offsets.get(key,np.zeros(3))+blend*self._pin_holes[key],ik.slerp_matrix(rot,self._pin_rotations[key],blend),finger,label
        return pos,rot,finger,label


def make_policy(**kwargs):
    return PanelPolicy(**kwargs)
