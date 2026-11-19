"""Record actual simulator mass properties and the supplied station geometry."""
from dataclasses import asdict
import numpy as np

def array(x):
    if hasattr(x,'torch'):x=x.torch
    if hasattr(x,'detach'):x=x.detach().cpu()
    if hasattr(x,'numpy'):x=x.numpy()
    return np.asarray(x).tolist()

def capture(emb, observation, names):
    from pxr import UsdGeom,Usd,UsdPhysics,UsdShade,Gf
    from yam_arena import station
    assert emb.rig == station.rig('calibrated')
    stage=emb._base.sim.stage
    result={'revision':emb.info.environment_revision,'rig':asdict(emb.rig),
            'dt':float(emb._base.cfg.sim.dt),'decimation':int(emb._base.cfg.decimation),
            'images':{n:list(im.shape) for n,im in observation.images.items()},'assets':{}}
    for name in names:
        asset=emb._base.scene[name];view=asset.root_physx_view
        result['assets'][name]={'body_names':list(asset.body_names),
          'mass_kg':array(view.get_masses()),'inertia_kg_m2':array(view.get_inertias()),
          'com_pose':array(view.get_coms())}
    bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_])
    table=stage.GetPrimAtPath('/World/envs/env_0/table')
    box=bounds.ComputeWorldBound(table).ComputeAlignedRange()
    result['task_object_world_bounds']={}
    for name in names:
        prim=stage.GetPrimAtPath('/World/envs/env_0/'+name)
        if not prim.IsValid():continue
        bound=bounds.ComputeWorldBound(stage.GetPrimAtPath('/World/envs/env_0/'+name)).ComputeAlignedRange()
        result['task_object_world_bounds'][name]={'min':list(bound.GetMin()),'max':list(bound.GetMax()),'size':list(bound.GetSize())}
    from yam_arena.robot import asset_link_paths
    result['robot_link_hierarchy_bounds']={}
    result['robot_mount_footprints']={}
    xforms=UsdGeom.XformCache()
    for side in ('left','right'):
        prim=stage.GetPrimAtPath('/World/envs/env_0/'+side.title()+'Arm'+asset_link_paths()['base'])
        b=bounds.ComputeWorldBound(prim).ComputeAlignedRange()
        result['robot_link_hierarchy_bounds'][side]={'min':list(b.GetMin()),'max':list(b.GetMax())}
        points=[]
        for child in prim.GetChildren():
            if child.HasAPI(UsdPhysics.RigidBodyAPI):continue
            for mesh in Usd.PrimRange(child,Usd.TraverseInstanceProxies()):
                if mesh.IsA(UsdGeom.Mesh):
                    vertices=UsdGeom.Mesh(mesh).GetPointsAttr().Get()
                    if vertices:
                        tr=xforms.GetLocalToWorldTransform(mesh)
                        points.extend([tr.Transform(Gf.Vec3d(*v)) for v in vertices])
        points=np.asarray(points);foot=points[points[:,2]<points[:,2].min()+.002]
        result['robot_mount_footprints'][side]={'min':foot.min(0).tolist(),'max':foot.max(0).tolist()}

    floor=bounds.ComputeWorldBound(stage.GetPrimAtPath('/World/envs/env_0/floor')).ComputeAlignedRange()
    result['floor_world_bounds']={'min':list(floor.GetMin()),'max':list(floor.GetMax())}
    v=emb._base.scene['cart'].root_physx_view
    result['native_cart_shape_materials']=array(v.get_material_properties())
    result['native_wheel_drives']={'stiffness':array(v.get_dof_stiffnesses()),'damping':array(v.get_dof_dampings()),'max_force':array(v.get_dof_max_forces())}
    result['native_robot_drives']={}
    for side in ('left','right'):
        view=emb._base.scene[side+'_robot'].root_physx_view
        result['native_robot_drives'][side]={'stiffness':array(view.get_dof_stiffnesses()),'damping':array(view.get_dof_dampings()),'max_force':array(view.get_dof_max_forces()),'max_velocity':array(view.get_dof_max_velocities())}
    result['native_wheel_joint_extra']={}
    for method in ('get_dof_friction_coefficients','get_dof_friction_properties','get_dof_armatures','get_dof_limits','get_dof_max_velocities'):
        if hasattr(v,method):
            try:result['native_wheel_joint_extra'][method]=array(getattr(v,method)())
            except Exception as exc:result['native_wheel_joint_extra'][method]=str(type(exc).__name__)
    result['table_world_bounds']={'min':list(box.GetMin()),'max':list(box.GetMax())}
    result['robot_platform_parts']={}
    for name in ('table','table_left_pad','table_right_pad'):
        prim=stage.GetPrimAtPath('/World/envs/env_0/'+name)
        if prim.IsValid():
            b=bounds.ComputeWorldBound(prim).ComputeAlignedRange()
            result['robot_platform_parts'][name]={'min':list(b.GetMin()),'max':list(b.GetMax())}
    mins=np.min([v['min'] for v in result['robot_platform_parts'].values()],axis=0)
    maxs=np.max([v['max'] for v in result['robot_platform_parts'].values()],axis=0)
    result['robot_platform_world_bounds']={'min':mins.tolist(),'max':maxs.tolist()}
    result['wheel_collider_cooking']={}
    for n in ('wheel_left','wheel_right'):
        prim=stage.GetPrimAtPath('/World/envs/env_0/cart/'+n+'/Contact')
        result['wheel_collider_cooking'][n]={'authored_vertex_count':len(UsdGeom.Mesh(prim).GetPointsAttr().Get()),'hull_vertex_limit':prim.GetAttribute('physxConvexHullCollision:hullVertexLimit').Get()}
    result['joints']=[];result['physics_materials']=[]
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if '/env_0/' not in path:continue
        if prim.IsA(UsdPhysics.RevoluteJoint) and '/cart/' in path:
            result['joints'].append({'path':path,'attributes':{a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('drive:','physics:'))}})
        if prim.HasAPI(UsdPhysics.MaterialAPI):
            result['physics_materials'].append({'path':path,'attributes':{a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('physics:','physxMaterial:'))}})
    return result


def write_reset(emb, observation, names):
    import os,json
    from pathlib import Path
    dest=os.environ.get('TASK_TELEMETRY_DIR')
    if not dest:return
    out=Path(dest);out.mkdir(parents=True,exist_ok=True)
    (out/'runtime-audit.json').write_text(json.dumps(capture(emb,observation,names),indent=2))


def write_episode(emb):
    import os,json
    from pathlib import Path
    dest=os.environ.get('TASK_TELEMETRY_DIR')
    if not dest or not hasattr(emb,'trace'):return
    out=Path(dest);out.mkdir(parents=True,exist_ok=True)
    (out/'physics.json').write_text(json.dumps(emb.trace,default=array))
    (out/'native-result.json').write_text(json.dumps(dict(score=emb.score,steps=len(emb.trace),revision=emb.info.environment_revision,events=getattr(emb,'events',[]),faults=getattr(emb,'faults',[])),indent=2,default=array))
