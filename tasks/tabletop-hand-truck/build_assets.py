"""Original fixed-deployed Wesco220646 proxy: real envelope, estimated internals."""
from pathlib import Path
import math,json,numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
from yam_arena import ik
from cart_task.config import *
ROOT=Path(__file__).resolve().parent;path=ROOT/'assets/cart.usda';path.unlink(missing_ok=True)
s=Usd.Stage.CreateNew(str(path));root=UsdGeom.Xform.Define(s,'/Cart');s.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim());root.GetPrim().AddAppliedSchema('PhysxArticulationAPI');root.GetPrim().CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(False)
materials={}
for n,sf,df in [('Metal',.5,.4),('Rubber',.9,.7),('Polymer',.4,.3)]:
 m=UsdShade.Material.Define(s,'/Cart/Materials/'+n);a=UsdPhysics.MaterialAPI.Apply(m.GetPrim());a.CreateStaticFrictionAttr(sf);a.CreateDynamicFrictionAttr(df);a.CreateRestitutionAttr(0.);materials[n]=m
frame=UsdGeom.Xform.Define(s,'/Cart/frame');UsdPhysics.RigidBodyAPI.Apply(frame.GetPrim());UsdShade.MaterialBindingAPI.Apply(frame.GetPrim()).Bind(materials['Metal'],materialPurpose='physics')
parts=[]
def box(n,size,pos,mass,group,material='aluminum',inner=None,color=(.65,.68,.71),collision=True):
 size=np.array(size);p=UsdGeom.Cube.Define(s,'/Cart/frame/'+n);p.CreateSizeAttr(1.);p.AddTranslateOp().Set(tuple(pos));p.AddScaleOp().Set(tuple(size));p.CreateDisplayColorAttr([color])
 if collision:UsdPhysics.CollisionAPI.Apply(p.GetPrim())
 if material in ('polypropylene','rubber'):
  UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(materials['Rubber' if material=='rubber' else 'Polymer'],materialPurpose='physics')
 def ib(m,d):return m/12*np.diag([d[1]**2+d[2]**2,d[0]**2+d[2]**2,d[0]**2+d[1]**2])
 I=ib(mass,size)
 if inner is not None:
  inside=np.array(inner);factor=np.prod(size)/(np.prod(size)-np.prod(inside));I=ib(mass*factor,size)-ib(mass*(factor-1),inside)
 parts.append(dict(name=n,group=group,material=material,mass_kg=mass,centre_m=list(pos),inertia=I.tolist(),geometry=dict(type='hollow_box_outer_collision_envelope' if inner is not None else 'box',size_m=size.tolist(),inner_size_m=None if inner is None else list(inner)),collision=collision))
def dense(n,size,pos,rho,group,material='aluminum',**kw):box(n,size,pos,np.prod(size)*rho,group,material,**kw)
# Original open-grid toe envelope. Member layout/thickness are estimates, not CAD.
for sign in (-1,1):dense('toe_side_'+str(sign).replace('-','m'),(.012,NOSE_DEPTH,.006),(sign*(.381/2-.006),-NOSE_DEPTH/2,.009),2700,'toe')
for i,(y,dep) in enumerate([(-.0125,.025),(-NOSE_DEPTH+.006,.012),(-.070,.012),(-.15875,.012),(-.2475,.012)]):dense('toe_web'+str(i),(.357,dep,.006),(0,y,.009),2700,'toe')
for sign in (-1,1):dense('toe_rib'+str(sign).replace('-','m'),(.012,.30,.006),(sign*.10,-NOSE_DEPTH/2,.003),2700,'toe')
# Lower handhold ring and upper push-button housing, approximated solid polymer.
for i,z in enumerate((.2275,.2725)):dense('lower_bar'+str(i),(.30,.04,.015),(0,0,z),950,'braces','polypropylene',color=(.12,.14,.13))
for sign in (-1,1):dense('lower_end'+str(sign).replace('-','m'),(.03,.04,.030),(sign*.135,0,.25),950,'braces','polypropylene',color=(.12,.14,.13))
dense('upper_bar',(.30,.04,.04),(0,0,.5621),950,'braces','polypropylene',color=(.12,.14,.13))
dense('top_grip',(.28,.027,.027),(0,0,HEIGHT-.0135),1100,'grip','rubber',color=(.12,.14,.13))
for sign in (-1,1):
 dense('wheel_mount'+str(sign).replace('-','m'),(.02,.04,.08),(sign*RAIL_X,AXLE_Y/2,.08),2700,'hardware')
 dense('toe_hinge'+str(sign).replace('-','m'),(.03,.02,.025),(sign*RAIL_X,.005,.0245),2700,'hardware')
# Circular steel axle, estimated8mm diameter; wheel dimensions are published.
axle_width=WIDTH;am=math.pi*BORE_RADIUS**2*axle_width*7850
ax=UsdGeom.Cylinder.Define(s,'/Cart/frame/axle');ax.CreateAxisAttr('X');ax.CreateRadiusAttr(BORE_RADIUS);ax.CreateHeightAttr(axle_width);ax.AddTranslateOp().Set((0,AXLE_Y,RADIUS));ax.CreateDisplayColorAttr([(.5,.5,.5)]);UsdPhysics.CollisionAPI.Apply(ax.GetPrim())
parts.append(dict(name='axle',group='hardware',material='steel',mass_kg=am,centre_m=[0,AXLE_Y,RADIUS],inertia=np.diag([am*BORE_RADIUS**2/2,*([am*(3*BORE_RADIUS**2+axle_width**2)/12]*2)]).tolist(),geometry=dict(type='cylinder',radius_m=BORE_RADIUS,length_m=axle_width),collision=True))
# Stored elastic straps: visual proxies only; no hidden restraint or elastic force.
for sign in (-1,1):dense('stored_strap'+str(sign).replace('-','m'),(.006,.006,.50),(sign*.09,.035,.40),1100,'straps','rubber',color=(.14,.22,.10),collision=False)
# An effective common profile wall closes the chosen9lb unit-weight budget.
profiles=[('lower_rail'+str(sign).replace('-','m'),(RAIL_WIDTH,RAIL_DEPTH,.55),(sign*RAIL_X,0,.30)) for sign in (-1,1)]
profiles += [('extended_handle'+str(sign).replace('-','m'),(.016,.020,HEIGHT-.027-.45),(sign*RAIL_X,0,(HEIGHT-.027+.45)/2)) for sign in (-1,1)]
def hollow(t,d):
 inner=np.array(d)-[2*t,2*t,0.];return np.prod(d)-np.prod(inner),inner
budget=MASS-2*WHEEL_MASS-sum(p['mass_kg'] for p in parts);lo,hi=.0001,.0079
assert budget>0
for _ in range(70):
 t=(lo+hi)/2;v=sum(hollow(t,size)[0] for n,size,pos in profiles)
 if v*2700<budget:lo=t
 else:hi=t
wall=(lo+hi)/2
for n,size,pos in profiles:
 volume,inner=hollow(wall,size);box(n,size,pos,volume*2700,'frame_profiles',inner=inner)
fm=sum(p['mass_kg'] for p in parts);assert abs(fm+2*WHEEL_MASS-MASS)<1e-8
com=sum(p['mass_kg']*np.array(p['centre_m']) for p in parts)/fm;I=np.zeros((3,3))
for p in parts:
 d=np.array(p['centre_m'])-com;I+=np.array(p['inertia'])+p['mass_kg']*(np.eye(3)*(d@d)-np.outer(d,d))
e,V=np.linalg.eigh(I)
if np.linalg.det(V)<0:V[:,0]*=-1
q=ik.matrix_to_quat_xyzw(V);ma=UsdPhysics.MassAPI.Apply(frame.GetPrim());ma.CreateMassAttr(fm);ma.CreateCenterOfMassAttr(Gf.Vec3f(*com));ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*e));ma.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
wheelI=np.zeros(3)
for outer,inner,rho in [(RADIUS,HUB_RADIUS,1100),(HUB_RADIUS,BORE_RADIUS,950)]:
 m=math.pi*(outer**2-inner**2)*WHEEL_WIDTH*rho;wheelI+=np.array([m*(outer**2+inner**2)/2,*([m*(3*(outer**2+inner**2)+WHEEL_WIDTH**2)/12]*2)])
for side,sign in [('left',-1),('right',1)]:
 n='wheel_'+side;cx=sign*(WIDTH/2-WHEEL_WIDTH/2);w=UsdGeom.Xform.Define(s,'/Cart/'+n);w.AddTranslateOp().Set((cx,AXLE_Y,RADIUS));UsdPhysics.RigidBodyAPI.Apply(w.GetPrim());ma=UsdPhysics.MassAPI.Apply(w.GetPrim());ma.CreateMassAttr(WHEEL_MASS);ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*wheelI))
 tire=UsdGeom.Cylinder.Define(s,'/Cart/'+n+'/Tyre');tire.CreateAxisAttr('X');tire.CreateRadiusAttr(RADIUS);tire.CreateHeightAttr(WHEEL_WIDTH);tire.CreateDisplayColorAttr([(.05,.05,.05)])
 hub=UsdGeom.Cylinder.Define(s,'/Cart/'+n+'/PolyHub');hub.CreateAxisAttr('X');hub.CreateRadiusAttr(HUB_RADIUS);hub.CreateHeightAttr(WHEEL_WIDTH+.0001);hub.CreateDisplayColorAttr([(.65,.68,.65)])
 mesh=UsdGeom.Mesh.Define(s,'/Cart/'+n+'/Contact');pts=[]
 for x in (-WHEEL_WIDTH/2,WHEEL_WIDTH/2):
  for j in range(120):a=2*math.pi*j/120;pts.append((x,RADIUS*math.sin(a),RADIUS*math.cos(a)))
 faces=[list(range(119,-1,-1)),list(range(120,240))]+[[j,(j+1)%120,(j+1)%120+120,j+120] for j in range(120)];mesh.GetPrim().AddAppliedSchema('PhysxConvexHullCollisionAPI');mesh.GetPrim().CreateAttribute('physxConvexHullCollision:hullVertexLimit',Sdf.ValueTypeNames.Int).Set(255);mesh.CreatePointsAttr(pts);mesh.CreateFaceVertexCountsAttr([len(f) for f in faces]);mesh.CreateFaceVertexIndicesAttr([v for f in faces for v in f]);mesh.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull');UsdShade.MaterialBindingAPI.Apply(w.GetPrim()).Bind(materials['Rubber'],materialPurpose='physics')
 mark=UsdGeom.Cube.Define(s,'/Cart/'+n+'/RotationIndex');mark.CreateSizeAttr(1.);mark.AddTranslateOp().Set((sign*(WHEEL_WIDTH/2+.0002),0,.03));mark.AddScaleOp().Set((.0002,.006,.045));mark.CreateDisplayColorAttr([(1.,.45,.02)])
 inner_mark=UsdGeom.Cube.Define(s,'/Cart/'+n+'/RotationIndexInner');inner_mark.CreateSizeAttr(1.);inner_mark.AddTranslateOp().Set((-sign*(WHEEL_WIDTH/2+.0002),0,.03));inner_mark.AddScaleOp().Set((.0002,.006,.045));inner_mark.CreateDisplayColorAttr([(1.,.45,.02)])
 tread_mark=UsdGeom.Cube.Define(s,'/Cart/'+n+'/RotationIndexTread');tread_mark.CreateSizeAttr(1.);tread_mark.AddTranslateOp().Set((0,0,RADIUS+.0002));tread_mark.AddScaleOp().Set((WHEEL_WIDTH,.006,.0002));tread_mark.CreateDisplayColorAttr([(1.,.45,.02)])
 joint=UsdPhysics.RevoluteJoint.Define(s,'/Cart/joint_'+side);joint.CreateBody0Rel().SetTargets(['/Cart/frame']);joint.CreateBody1Rel().SetTargets(['/Cart/'+n]);joint.CreateAxisAttr('X');joint.CreateLocalPos0Attr(Gf.Vec3f(cx,AXLE_Y,RADIUS));joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateCollisionEnabledAttr(False);drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(2e-7);drive.CreateMaxForceAttr(.001);drive.CreateTargetVelocityAttr(0.);drive.CreateTargetPositionAttr(0.)
s.GetRootLayer().Save()
reference=dict(product='Wesco Mini Mover220646',url='https://catalog.wescomfg.com/item/platform-trucks-carts-dollies/plastic-platform-lightweight-folding-trucks/220646-1',drawing_url='https://catalog.wescomfg.com/Asset/220646.pdf',height_m=HEIGHT,width_m=WIDTH,length_m=LENGTH,total_mass_kg=MASS,older_weight_lb=7.25,selected_current_weight_lb=9.)
groups={}
for p in parts:groups[p['group']]=groups.get(p['group'],0)+p['mass_kg']
ledger=dict(reference=reference,scale=1.,layout=dict(cart_support='floor',support_height_m=GROUND,robot_platform_size_m=list(PLATFORM_SIZE),robot_platform_center_m=list(PLATFORM_CENTER),initial_axle_m=AXLE.tolist(),goal_axle_m=[.35,0.,GROUND+RADIUS],platform_shape='U',platform_parts={n:dict(size_m=list(v[0]),center_m=list(v[1])) for n,v in PLATFORM_PARTS.items()},grasp_points_local_m={n:v.tolist() for n,v in GRIP_LOCAL.items()},grip_actions=dict(GRIP_ACTION),tcp_advance_m={'left':.008,'right':.012},grasp_rotations_world={n:v.tolist() for n,v in GRIP_ROTS.items()},reset_settle_s=2.,robot_target_preload_z_m=TCP_PRELOAD_Z,roll_direction=ROLL_DIRECTION,user_authorized_reduced_table=True),derived=dict(height_m=HEIGHT,width_m=WIDTH,length_m=LENGTH,total_mass_kg=fm+2*WHEEL_MASS,wheel_diameter_m=2*RADIUS,wheel_width_m=WHEEL_WIDTH,wheel_mass_kg=WHEEL_MASS,wheel_inertia_kg_m2=wheelI.tolist(),frame_mass_kg=fm,frame_com_m=com.tolist(),frame_inertia_kg_m2=I.tolist(),frame_principal_inertia_kg_m2=e.tolist(),effective_profile_wall_m=wall,axle_to_frame_offset_m=AXLE_Y),part_groups=groups,parts=parts,payload=None,contact_model=dict(wheel_collision_sides=120,wheel_hull_vertex_limit=255,metal_static_dynamic=[.5,.4],tire_static_dynamic=[.9,.7],assumed_rubber_handle_static_dynamic=[.9,.7],polymer_and_paper_static_dynamic=[.4,.3],restitution=0.),assumptions=['Actual listed41in operating height,15.5in width,17.5in depth,15x12.5in nose and5x1.25in wheels at scale1:1;24in is folded height, not used for this task.','Current unit weight9lb is selected; manufacturer-linked drawing and older catalog say7.25lb. Neither value is a measured test-unit mass.','Axle offset63.5mm is inferred from overall depth minus toe depth minus tire radius, assuming the wheels define the rear extremity; no measured axle offset is supplied.','Profiles, open-grid toe, brace/hinge locations, handle and mounts are original approximate proxies from product references, not manufacturer CAD.','Fold-out wheel/nose linkage and telescoping handle are fixed in the fully deployed, latched configuration. Folding and latch behavior are not simulated.','Only total unit mass and dimensions are sourced. Component masses and inertia are estimated from geometry, densities2700aluminum/7850steel/950polymer/1100rubber kg/m3. Effective profile wall closes the9lb budget; mass distribution is unmeasured.','Wheel hub radius45mm,axle bore8mm and material distribution are assumptions. Tire deformation and bearing resistance are unmeasured.','Elastic straps are stored as visual noncolliding rigid proxies; no strap restrains the load or contributes an elastic force.','The cart is empty. Estimated friction and all hardware behavior are physically unqualified. Cart transport takes place on the supplied floor in front of a user-authorized reduced robot platform; robot bases and cameras remain calibrated.'])
(ROOT/'physical_design.json').write_text(json.dumps(ledger,indent=2)+'\n');print(json.dumps(ledger['derived'],indent=2))
