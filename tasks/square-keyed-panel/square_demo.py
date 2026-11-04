"""Privileged pose-feedback demonstration; commands robot joints only."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
from inspect_robots_yam.packing import split
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.demo import ArmPlan, grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import quat_xyzw_to_matrix

class PlacementPlan(ArmPlan):
    def target(self,t):
        pos,rot,finger,label=super().target(t)
        if ':' not in label:
            return pos,rot,finger,label
        index,phase=label.split(':',1)
        obj=self.task.demo_states.get(f'square_api_piece_{index}')
        if obj is None:
            return pos,rot,finger,label
        tcp,tcp_rot=self.actual_pose
        if phase in ('lift','carry','align','lower','seat'):
            # The object follows the Cartesian path despite a slightly off-centre grasp.
            if getattr(self,'piece',None)!=index:
                self.piece=index
                self.bias=np.zeros(3)
                self.rotation_bias=np.eye(3)
            obj_rot=quat_xyzw_to_matrix(obj.quat_xyzw)
            desired=rot @ grip_rotation(self.task.grasp_yaw(int(index)-1)).T
            correction=desired @ obj_rot.T
            goal_rot=correction @ tcp_rot
            offset=np.r_[self.task.grasp_offset(int(index)-1),self.task.GRASP_Z_OFFSET]
            center=pos-desired@offset
            if phase in ('align','seat'):
                self.bias=np.clip(self.bias+.12*(center-obj.pos),-.012,.012)
                self.rotation_bias=ik.slerp_matrix(np.eye(3),correction,.08)@self.rotation_bias
                q=ik.matrix_to_quat_xyzw(self.rotation_bias)
                angle=2*np.arccos(np.clip(abs(q[3]),0.,1.))
                if angle>np.deg2rad(12):
                    self.rotation_bias=ik.slerp_matrix(np.eye(3),self.rotation_bias,np.deg2rad(12)/angle)
            rot=self.rotation_bias@goal_rot
            pos=center+correction@(tcp-obj.pos)+self.bias
            self.last_command=(pos.copy(),rot.copy())
        elif phase in ('settle','release') and hasattr(self,'last_command'):
            pos,rot=self.last_command
        elif phase=='retreat' and hasattr(self,'last_command'):
            previous,rot=self.last_command
            pos=np.array([previous[0],previous[1],pos[2]])
        return pos,rot,finger,label

class SquarePolicy(YamScriptedPolicy):
    def __init__(self, trace_path=None, **kwargs):
        super().__init__(**kwargs)
        self.info=replace(self.info,name="square_scripted")
        self.trace_path=Path(trace_path) if trace_path else None
        self.trace_step=0
    def reset(self,scene):
        plans=self.plan_for(scene.instruction,scene.target)
        self._plans={side:PlacementPlan(list(waypoints),self._bases[side]) for side,waypoints in plans.items()}
        for side,p in self._plans.items():
            p.task=self._task.module
            p.actual_pose=ik.tcp_pose(np.asarray(READY),self._bases[side])
        self._end=max(p.duration for p in self._plans.values())
    def act(self,observation):
        self.trace_step+=1
        if self.trace_path and self.trace_step%10==0:
            row={"step":self.trace_step,"objects":{k:{"pos":v.pos.tolist(),"quat":v.quat_xyzw.tolist()} for k,v in self._task.module.demo_states.items() if k.startswith("square_api_piece_")}}
            row["arms"]={side:{"tcp":p.actual_pose[0].tolist(),"tcp_rotation":p.actual_pose[1].tolist(),"command_pos":p.last_command[0].tolist() if hasattr(p,"last_command") else None,"command_rotation":p.last_command[1].tolist() if hasattr(p,"last_command") else None} for side,p in self._plans.items()}
            with self.trace_path.open("a") as f:f.write(json.dumps(row)+"\n")
        for side,q in zip(('left','right'),split(observation.state['joint_pos'])):
            if side in self._plans:self._plans[side].actual_pose=ik.tcp_pose(q[:6],self._bases[side])
        return super().act(observation)

def make_policy(**kwargs):return SquarePolicy(**kwargs)
