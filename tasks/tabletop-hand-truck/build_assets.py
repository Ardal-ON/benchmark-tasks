"""Build native passive-wheel geometry and the auditable product-based mass ledger."""
from pathlib import Path
import math,json,numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
from yam_arena import ik
from cart_task.config import MASS,WHEEL_MASS,RADIUS,WIDTH,AXLE_Y,SCALE
ROOT=Path(__file__).resolve().parent;path=ROOT/'assets/cart.usda';path.unlink(missing_ok=True)
s=Usd.Stage.CreateNew(str(path));root=UsdGeom.Xform.Define(s,'/Cart');s.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
root.GetPrim().AddAppliedSchema('PhysxArticulationAPI');root.GetPrim().CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(False)
materials={}
for name,sf,df in [('Metal',.5,.4),('Rubber',.9,.7)]:
 mat=UsdShade.Material.Define(s,'/Cart/Materials/'+name);api=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());api.CreateStaticFrictionAttr(sf);api.CreateDynamicFrictionAttr(df);api.CreateRestitutionAttr(0.);materials[name]=mat
parts=[]
def box(name,size,pos,rho,mass=None):
 size=np.array(size);pos=np.array(pos);m=float(np.prod(size)*rho) if mass is None else mass
 p=UsdGeom.Cube.Define(s,'/Cart/frame/'+name);p.CreateSizeAttr(1.);p.AddTranslateOp().Set(tuple(pos));p.AddScaleOp().Set(tuple(size));p.CreateDisplayColorAttr([(.66,.69,.72)]);UsdPhysics.CollisionAPI.Apply(p.GetPrim())
 I=m/12*np.diag([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2]);parts.append(dict(name=name,mass_kg=m,centre_m=pos.tolist(),inertia=I.tolist(),geometry=dict(type='box',size_m=size.tolist(),density_kg_m3=rho)))
def rod(name,length,pos,axis='Z',ro=.004,ri=.003,rho=2700):
 m=math.pi*(ro*ro-ri*ri)*length*rho;p=UsdGeom.Cylinder.Define(s,'/Cart/frame/'+name);p.CreateAxisAttr(axis);p.CreateRadiusAttr(ro);p.CreateHeightAttr(length);p.AddTranslateOp().Set(tuple(pos));p.CreateDisplayColorAttr([(.68,.71,.74)]);UsdPhysics.CollisionAPI.Apply(p.GetPrim())
 axial=.5*m*(ro*ro+ri*ri);side=m*(3*(ro*ro+ri*ri)+length*length)/12;diag=[side]*3;diag['XYZ'.index(axis)]=axial
 parts.append(dict(name=name,mass_kg=m,centre_m=list(pos),inertia=np.diag(diag).tolist(),geometry=dict(type='hollow_tube_collision_envelope',length_m=length,outer_radius_m=ro,inner_radius_m=ri,axis=axis,density_kg_m3=rho)))
f=UsdGeom.Xform.Define(s,'/Cart/frame');UsdPhysics.RigidBodyAPI.Apply(f.GetPrim());UsdShade.MaterialBindingAPI.Apply(f.GetPrim()).Bind(materials['Metal'],materialPurpose='physics')
for side,x in [('left',-.037),('right',.037)]:rod(side+'_rail',.284,(x,0.,.154))
for j,z in enumerate((.05,.13,.21,.296)):rod('cross'+str(j),.074,(0.,0.,z),'X')
box('nose',(.1125,.046875,.002),(0.,-.046875/2,.001),2700)
rod('axle',.103125,(0.,AXLE_Y,RADIUS),'X',.002,0.,7850)
remaining=MASS-2*WHEEL_MASS-sum(p['mass_kg'] for p in parts);thick=remaining/(2*2700*.02*AXLE_Y)
assert .002<thick<.02,thick
for side,x in [('left',-.037),('right',.037)]:box(side+'_axle_bracket',(thick,AXLE_Y,.02),(x,AXLE_Y/2,RADIUS),2700)
rod('grip_sleeve',.060,(0.,0.,.296),'X',.013,.004,350)
UsdGeom.Gprim(s.GetPrimAtPath('/Cart/frame/grip_sleeve')).CreateDisplayColorAttr([(.08,.08,.08)])
fm=sum(p['mass_kg'] for p in parts);com=sum(p['mass_kg']*np.array(p['centre_m']) for p in parts)/fm;I=np.zeros((3,3))
for p in parts:
 d=np.array(p['centre_m'])-com;I+=np.array(p['inertia'])+p['mass_kg']*(np.eye(3)*np.dot(d,d)-np.outer(d,d))
e,V=np.linalg.eigh(I)
if np.linalg.det(V)<0:V[:,0]*=-1
q=ik.matrix_to_quat_xyzw(V);api=UsdPhysics.MassAPI.Apply(f.GetPrim());api.CreateMassAttr(fm);api.CreateCenterOfMassAttr(Gf.Vec3f(*com));api.CreateDiagonalInertiaAttr(Gf.Vec3f(*e));api.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
for side,x in [('left',-1),('right',1)]:
 name='wheel_'+side;cx=x*(WIDTH/2-.00625);pos=(cx,AXLE_Y,RADIUS);w=UsdGeom.Xform.Define(s,'/Cart/'+name);w.AddTranslateOp().Set(pos);UsdPhysics.RigidBodyAPI.Apply(w.GetPrim());ma=UsdPhysics.MassAPI.Apply(w.GetPrim());ma.CreateMassAttr(WHEEL_MASS);ma.CreateDiagonalInertiaAttr(Gf.Vec3f(.5*WHEEL_MASS*RADIUS**2,*([WHEEL_MASS*(3*RADIUS**2+.0125**2)/12]*2)))
 tire=UsdGeom.Cylinder.Define(s,'/Cart/'+name+'/Tyre');tire.CreateAxisAttr('X');tire.CreateRadiusAttr(RADIUS);tire.CreateHeightAttr(.0125);tire.CreateDisplayColorAttr([(.035,.035,.035)])
 # A painted index stripe makes rotation observable in a physical/video trial.
 # It is purely visual, has no collision or separate body, and negligible paint mass.
 mark=UsdGeom.Cube.Define(s,'/Cart/'+name+'/RotationIndex');mark.CreateSizeAttr(1.);mark.AddTranslateOp().Set((x*.0064,0.,.014));mark.AddScaleOp().Set((.0002,.004,.018));mark.CreateDisplayColorAttr([(.95,.95,.95)])
 UsdShade.MaterialBindingAPI.Apply(w.GetPrim()).Bind(materials['Rubber'],materialPurpose='physics')
 hull=UsdGeom.Mesh.Define(s,'/Cart/'+name+'/Contact');pts=[(xx,RADIUS*math.cos(2*math.pi*i/64),RADIUS*math.sin(2*math.pi*i/64)) for xx in (-.00625,.00625) for i in range(64)];hull.CreatePointsAttr(pts);hull.CreateFaceVertexCountsAttr([4]*64+[64,64]);hull.CreateFaceVertexIndicesAttr([v for i in range(64) for v in (i,(i+1)%64,(i+1)%64+64,i+64)]+list(range(63,-1,-1))+list(range(64,128)));hull.CreateVisibilityAttr('invisible');hull.CreateSubdivisionSchemeAttr('none');UsdPhysics.CollisionAPI.Apply(hull.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(hull.GetPrim()).CreateApproximationAttr('convexHull')
 hub=UsdGeom.Cylinder.Define(s,'/Cart/'+name+'/Hub');hub.CreateAxisAttr('X');hub.CreateRadiusAttr(.012);hub.CreateHeightAttr(.013);hub.CreateDisplayColorAttr([(.6,.62,.64)])
 mark=UsdGeom.Cube.Define(s,'/Cart/'+name+'/Mark');mark.CreateSizeAttr(1.);mark.AddTranslateOp().Set((x*.0066,0.,.018));mark.AddScaleOp().Set((.0005,.004,.022));mark.CreateDisplayColorAttr([(.95,.8,.03)])
 j=UsdPhysics.RevoluteJoint.Define(s,'/Cart/joint_'+side);j.CreateBody0Rel().SetTargets(['/Cart/frame']);j.CreateBody1Rel().SetTargets(['/Cart/'+name]);j.CreateAxisAttr('X');j.CreateLocalPos0Attr(Gf.Vec3f(*pos));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateLocalRot0Attr(Gf.Quatf(1));j.CreateLocalRot1Attr(Gf.Quatf(1));j.CreateCollisionEnabledAttr(False);drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(0.);drive.CreateDampingAttr(2e-7);drive.CreateMaxForceAttr(.001);drive.CreateTargetVelocityAttr(0.)
s.GetRootLayer().Save()
ledger=dict(reference=dict(product='Magliner116-UA-1030',url='https://www.magliner.com/116-ua-1030',height_m=1.2192,width_m=.4699,length_m=.4572,total_mass_kg=24*.45359237,wheel_product='101030',wheel_url='https://www.magliner.com/10-inch-ballon-cushion-wheel-101030',wheel_diameter_m=.254,wheel_width_m=.0508,wheel_unit_mass_kg=3*.45359237),scale=SCALE,derived=dict(height_m=.3,width_m=WIDTH,length_m=.1125,wheel_diameter_m=2*RADIUS,wheel_width_m=.0125,nose_size_m=[.1125,.046875,.002],reference_scaled_base_mass_kg=MASS,total_mass_kg=fm+2*WHEEL_MASS,wheel_mass_kg=WHEEL_MASS,frame_mass_kg=fm,frame_com_m=com.tolist(),frame_inertia_kg_m2=I.tolist(),frame_principal_inertia_kg_m2=e.tolist(),bracket_thickness_m=thick),parts=parts,assumptions=['This is a built miniature replica, not a manufacturer-sold300mm industrial truck.','Manufacturer whole/wheel masses scaled with s^3 at unchanged density. Force/inertia equivalence to fullsize is not claimed.','Unpublished tube/bracket/nose thicknesses are deliberate build dimensions. Bracket thickness closes the derived frame mass budget using aluminum2700kg/m3.','Axle uses steel7850kg/m3. Tube collisions are outer envelopes but masses use hollow tube volumes.','Wheel uses actual scaled unit mass; cylinder inertia approximates cushion rubber, polyolefin hub and bearings. Collision uses64-sided convex hull. Passive zero-target viscous bearing drive damping2e-7 USD angular units, stiffness0, max torque.001Nm; no commanded wheel motion.','An added60mm-long26mm-OD grip sleeve (8mm bore, assumed foam density350kg/m3) adds mass beyond the scaled reference and increases overall height9mm.','Physical apparatus and friction/bearing parameters have not been measured.'])
(ROOT/'physical_design.json').write_text(json.dumps(ledger,indent=2)+'\n');print(json.dumps(ledger['derived'],indent=2))
