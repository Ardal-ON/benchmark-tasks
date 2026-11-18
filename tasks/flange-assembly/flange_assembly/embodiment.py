"""Supported PVC-U pipe joint with ordinary bare-pipe gripping."""
from pathlib import Path
from dataclasses import replace,fields
import hashlib,math,json,os,subprocess
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from flange_assembly.specs import MOVING_BBOX
ROOT=Path(__file__).resolve().parents[1]
_DEMO_STATES={}
CONTACTS=[]
def demo_states():return _DEMO_STATES

def spawn_pin(prim_path,cfg,translation=None,orientation=None,**kwargs):
    """DIN931 M12x50 envelope; blunt12mm shaft, threads/chamfer omitted."""
    import omni.usd
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
    stage=omni.usd.get_context().get_stage();root=UsdGeom.Xform.Define(stage,prim_path)
    root.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0,0,0))))
    q=orientation or (1,0,0,0);root.AddOrientOp().Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*map(float,q[1:]))))
    p=root.GetPrim();UsdPhysics.RigidBodyAPI.Apply(p);UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(.0632)
    def piece(name, points, faces):
        m=UsdGeom.Mesh.Define(stage,f'{prim_path}/{name}')
        m.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in points]);m.CreateFaceVertexCountsAttr([len(f) for f in faces]);m.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(.55,.57,.59)])
        UsdPhysics.CollisionAPI.Apply(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
        c=PhysxSchema.PhysxCollisionAPI.Apply(m.GetPrim());c.CreateContactOffsetAttr().Set(.00015);c.CreateRestOffsetAttr().Set(0.)
    # Small-radius monolithic hulls are simplified by PhysX cooking even below
    # the vertex limit. Six-vertex sectors preserve the actual32-sided shaft.
    prism_faces=[(2,1,0),(3,4,5),(0,1,4,3),(1,2,5,4),(2,0,3,5)]
    for i in range(32):
        angles=(2*math.pi*i/32,2*math.pi*(i+1)/32)
        def ring(z,r):return [(0.,0.,z)]+[(r*math.cos(a),r*math.sin(a),z) for a in angles]
        piece(f'shaft_{i}',ring(-.025,.006)+ring(.025,.006),prism_faces)
    n=6;r=.019/math.sqrt(3)
    points=[(r*math.cos(2*math.pi*i/n),r*math.sin(2*math.pi*i/n),z) for z in (.025,.0325) for i in range(n)]
    faces=[tuple(reversed(range(n))),tuple(n+i for i in range(n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
    piece('head',points,faces)
    return p

class FlangeEmbodiment(YamArenaEmbodiment):
    def __init__(self,**kwargs):
        for k,v in dict(task=str(ROOT/'task.py'),device='cpu',control_hz=20,cam_width=640,cam_height=360,object_jitter=0.).items():kwargs.setdefault(k,v)
        super().__init__(**kwargs);h=hashlib.sha256()
        for p in sorted(ROOT.rglob('*')):
            if p.is_file() and p.suffix in ('.py','.usda','.toml','.csv'):h.update(p.relative_to(ROOT).as_posix().encode()+b'\0'+p.read_bytes())
        self.info=replace(self.info,name='yam_flange',environment_revision=h.hexdigest())
    def reset(self,scene,*,seed=None):
        self.contact_trace=[];self._witness_writer=None
        obs=super().reset(scene,seed=seed);self._publish_demo_states()
        folder=os.environ.get('FLANGE_DIAGNOSTICS_DIR')
        if folder:
            out=Path(folder);out.mkdir(parents=True,exist_ok=True);im=self._fixture_image();h,w=im.shape[:2]
            self._witness_writer=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r','20','-i','-','-an','-c:v','libx264','-threads','1','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'fixture_cam.mp4')],stdin=subprocess.PIPE,stderr=subprocess.PIPE)
            self._witness_writer.stdin.write(im.tobytes())
            import imageio.v3 as iio
            iio.imwrite(out/'fixture-initial.png',im)
        return obs
    def _fixture_image(self):
        value=self._base.scene['fixture_cam'].data.output['rgb']
        value=value.torch if hasattr(value,'torch') else value
        value=value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)
        return value[0,:,:,:3].astype(np.uint8)
    def _publish_demo_states(self):
        global _DEMO_STATES
        from yam_arena.scene import object_states
        _DEMO_STATES=object_states(self._env,('lower_flange','upper_flange','pin_1','pin_2'));return _DEMO_STATES
    def _ensure_sim(self,goal):
        if self._env is not None:return
        import atexit
        from yam_arena import inspect_embodiment as base
        if base._APP is None:
            from isaaclab.app import AppLauncher
            base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app;atexit.register(base._close_app_at_exit)
        from yam_arena import scene,embodiment as robots
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        import isaaclab.sim as sim
        from yam_arena.ik import matrix_to_quat_xyzw
        from yam_arena.urdf import axis_angle_matrix
        original,old_callback,old_robot=scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment
        def spawn(spec):
            if spec.name in ('lower_flange','upper_flange') or '_rest_' in spec.name:
                name=('fixed_pipe' if spec.name=='lower_flange' else 'moving_pipe') if 'flange' in spec.name else 'v_rest'
                rotation=axis_angle_matrix(np.array([0.,1.,0.]),np.pi/2 if spec.name=='lower_flange' else -np.pi/2) if 'flange' in spec.name else np.eye(3)
                if spec.name=='upper_flange':rotation=axis_angle_matrix(np.array([1.,0.,0.]),np.deg2rad(-15))@rotation
                return Object(name=spec.name,usd_path=str(ROOT/'assets'/f'{name}.usda'),object_type=ObjectType.RIGID if spec.name=='upper_flange' else ObjectType.BASE,
                    initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=tuple(map(float,matrix_to_quat_xyzw(rotation)))),
                    spawn_cfg_addon=dict(physics_material=sim.RigidBodyMaterialCfg(static_friction=.25 if '_rest_' in spec.name else .4,dynamic_friction=.2 if '_rest_' in spec.name else .3),
                        collision_props=sim.CollisionPropertiesCfg(contact_offset=.00015,rest_offset=0.),
                        rigid_props=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=64,solver_velocity_iteration_count=16,max_depenetration_velocity=.2)))
            obj=original(spec)
            if spec.name in ('pin_1','pin_2'):
                obj.spawner_cfg.func=spawn_pin;obj.initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,-2**-.5,0.,2**-.5));obj.object_cfg=obj._init_object_cfg();obj._pose_event_cfg=obj._build_reset_event()
            return obj
        def callback(hz=None):
            parent=old_callback(hz)
            def configure(cfg):
                cfg=parent(cfg);cfg.sim.dt=1/480;cfg.decimation=24;cfg.sim.render_interval=24;cfg.sim.physics.enable_ccd=True
                from isaaclab.sensors import CameraCfg
                from isaaclab_arena.utils.configclass import make_configclass
                eye=np.array([.60,-.70,1.85]);target=np.array([.15,.04,.88])
                back=eye-target;back/=np.linalg.norm(back);right=np.cross([0.,0.,1.],back);right/=np.linalg.norm(right);up=np.cross(back,right)
                quat=tuple(map(float,matrix_to_quat_xyzw(np.column_stack((right,up,back)))))
                camera=CameraCfg(prim_path='{ENV_REGEX_NS}/FixtureCam',height=720,width=1280,data_types=['rgb'],update_period=.05,
                    spawn=sim.PinholeCameraCfg(focal_length=20.,horizontal_aperture=20.955,clipping_range=(.01,20.)),
                    offset=CameraCfg.OffsetCfg(pos=tuple(map(float,eye)),rot=quat,convention='opengl'),renderer_cfg=robots.RENDERER)
                values={f.name:getattr(cfg.scene,f.name) for f in fields(cfg.scene) if f.init}
                cls=make_configclass('FlangeFixtureSceneCfg',[('fixture_cam',CameraCfg,camera)],bases=(type(cfg.scene),));cfg.scene=cls(**values)
                return cfg
            return configure
        class NativeRobot(old_robot):
            def __init__(robot,**opts):opts.update(cam_width=640,cam_height=360,robot_contact_offset=.00015);super().__init__(**opts)
        scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=spawn,callback,NativeRobot
        try:super()._ensure_sim(goal)
        finally:scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=original,old_callback,old_robot
        import omni.physx
        from pxr import PhysxSchema,UsdPhysics
        self.physics=omni.physx.get_physx_simulation_interface();self._base.sim.set_setting('/physics/disableContactProcessing',False)
        self.subscription=self.physics.subscribe_full_contact_report_events(lambda *args:None)
        for prim in self._base.sim.stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    def _task_step(self):
        result=super()._task_step();states=self._publish_demo_states()
        from pxr import PhysicsSchemaTools
        global CONTACTS
        headers,contacts,_=self.physics.get_full_contact_report();CONTACTS=[];robot_contacts=[];pin_contact_samples=[]
        for h in headers:
            if h.num_contact_data:
                names=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (h.collider0,h.collider1)]
                if any('/upper_flange/' in p for p in names):CONTACTS.append(names)
                if any('Arm/' in p for p in names):robot_contacts.append(names)
                if any('RightArm/' in p for p in names) and any('/pin_' in p for p in names):
                    for index in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                        d=contacts[index]
                        pin_contact_samples.append(dict(colliders=names,position_m=list(map(float,d.position)),normal=list(map(float,d.normal)),impulse_Ns=list(map(float,d.impulse)),separation_m=float(d.separation),materials=[str(PhysicsSchemaTools.intToSdfPath(int(v))) for v in (d.material0,d.material1)]))
        from flange_assembly.geometry import score,rotation
        r=rotation(states['upper_flange'].quat_xyzw)
        root=np.asarray(states['upper_flange'].pos)-r@np.array(MOVING_BBOX)
        roll=math.atan2(r[2,1],r[1,1])
        from flange_assembly.policy import CURRENT_PHASES,CONTROL_STATE
        self.contact_trace.append(dict(pin_contact_last_physics_substep=pin_contact_samples,physics_dt_s=1/480,pre_step_hands=CONTROL_STATE.copy(),robot_contacts=robot_contacts,phases=CURRENT_PHASES.copy(),t=float(self._t),root=root.tolist(),roll_rad=roll,score=score(states['lower_flange'],states['upper_flange'],[states['pin_1'],states['pin_2']]),contacts=CONTACTS.copy(),objects={k:dict(pos=np.asarray(v.pos).tolist(),quat_xyzw=np.asarray(v.quat_xyzw).tolist()) for k,v in states.items()}))
        if getattr(self,'_witness_writer',None):self._witness_writer.stdin.write(self._fixture_image().tobytes())
        if int(self._t*20)%20==0:
            print('ASSEMBLY_STATE',round(self._t,2),{k:{'p':[round(float(x),5) for x in v.pos],'q':[round(float(x),5) for x in v.quat_xyzw],'v':[round(float(x),5) for x in v.lin_vel]} for k,v in states.items()},'CONTACTS',CONTACTS,flush=True)
        return result

    def close(self):
        try:
            folder=os.environ.get('FLANGE_DIAGNOSTICS_DIR')
            if folder and hasattr(self,'contact_trace'):
                out=Path(folder);out.mkdir(parents=True,exist_ok=True)
                (out/'contact-trace.json').write_text(json.dumps(self.contact_trace))
            if getattr(self,'_witness_writer',None):
                self._witness_writer.stdin.close();rc=self._witness_writer.wait(timeout=30);error=self._witness_writer.stderr.read().decode(errors='replace');self._witness_writer=None
                if folder:(Path(folder)/'fixture-video.log').write_text(error)
                if rc:raise RuntimeError('Diagnostic video encoder failed')
        finally:super().close()

def make_embodiment(**kwargs):return FlangeEmbodiment(**kwargs)
