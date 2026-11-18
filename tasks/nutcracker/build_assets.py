"""Generate custom, simplified rigid nutcracker and split-shell assets."""
from pathlib import Path
import math,json,hashlib
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf
R=Path(__file__).resolve().parent
(R/'assets').mkdir(exist_ok=True)
def stage(name,mass=None):
 s=Usd.Stage.CreateNew(str(R/'assets'/f'{name}.usda'));UsdGeom.SetStageMetersPerUnit(s,1.);UsdGeom.SetStageUpAxis(s,'Z');p=UsdGeom.Xform.Define(s,'/Asset').GetPrim();s.SetDefaultPrim(p)
 if mass is not None:UsdPhysics.RigidBodyAPI.Apply(p);UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
 return s
def mat(s,name,color):
 m=UsdShade.Material.Define(s,'/Asset/Materials/'+name);sh=UsdShade.Shader.Define(s,'/Asset/Materials/'+name+'/Surface');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.7);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m
def box(s,name,pos,size,color,collide=True):
 p=UsdGeom.Cube.Define(s,'/Asset/'+name);p.CreateSizeAttr(1.);p.AddTranslateOp().Set(Gf.Vec3d(*pos));p.AddScaleOp().Set(Gf.Vec3f(*size));UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(mat(s,name,color));
 if collide:UsdPhysics.CollisionAPI.Apply(p.GetPrim())
 return p
def cylinder(s,name,pos,radius,height,color,collide=True):
 p=UsdGeom.Cylinder.Define(s,'/Asset/'+name);p.CreateAxisAttr('Z');p.CreateRadiusAttr(radius);p.CreateHeightAttr(height);p.AddTranslateOp().Set(Gf.Vec3d(*pos));UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(mat(s,name,color))
 if collide:UsdPhysics.CollisionAPI.Apply(p.GetPrim())
 return p
def sphere(s,name,pos,radius,color):
 p=UsdGeom.Sphere.Define(s,'/Asset/'+name);p.CreateRadiusAttr(radius);p.AddTranslateOp().Set(Gf.Vec3d(*pos));UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(mat(s,name,color));UsdPhysics.CollisionAPI.Apply(p.GetPrim());return p
def half(s,name,sign):
 # Convex semi-ellipsoid with flat break plane y=0; origin is half centroid at +/-8 mm.
 pts=[];faces=[];N=20;M=10
 for j in range(M+1):
  phi=math.pi*j/M
  for i in range(N+1):
   theta=math.pi*i/N
   pts.append(Gf.Vec3f(.018*math.sin(phi)*math.cos(theta),sign*(.016*math.sin(phi)*math.sin(theta)-.008),.018*math.cos(phi)))
 for j in range(M):
  for i in range(N):
   a=j*(N+1)+i;faces.extend([a,a+1,a+N+2,a+N+1])
 mesh=UsdGeom.Mesh.Define(s,'/Asset/'+name);mesh.CreatePointsAttr(pts);mesh.CreateFaceVertexCountsAttr([4]*(N*M));mesh.CreateFaceVertexIndicesAttr(faces);mesh.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat(s,name,(.43,.23,.09)));UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
 # Light kernel printed on inner face: visual approximation, remains attached to each shell half.
 box(s,'Kernel', (0.,-sign*.0078,0.),(.021,.0005,.022),(.86,.70,.44),False)
# A tabletop figurine: red jacket, blue legs, cylindrical-looking head and hat.
# One rigid lower jaw and the rearward sleeve/hand form the movable lever.
s=stage('fixture');box(s,'Base',(0.,0.,.006),(.13,.13,.012),(.16,.14,.10));s.GetRootLayer().Save()
s=stage('upper',.45)
box(s,'LeftBoot',(-.028,0.,-.103),(.040,.055,.025),(.04,.04,.05),False)
box(s,'RightBoot',(.028,0.,-.103),(.040,.055,.025),(.04,.04,.05),False)
cylinder(s,'LeftLeg',(-.027,0.,-.062),.015,.065,(.10,.17,.42))
cylinder(s,'RightLeg',(.027,0.,-.062),.015,.065,(.10,.17,.42))
cylinder(s,'Jacket',(0.,0.,-.0695),.068,.049,(.66,.05,.035))
# Recess the collar around the moving jaw's full sweep.
box(s,'CollarBack',(0.,-.046,-.0295),(.090,.040,.031),(.66,.05,.035))
box(s,'CollarLeft',(-.050,.018,-.0295),(.026,.055,.031),(.66,.05,.035))
box(s,'CollarRightInner',(.038,.018,-.0295),(.002,.055,.031),(.66,.05,.035))
box(s,'CollarRightOuter',(.054,.018,-.0295),(.018,.055,.031),(.66,.05,.035))
box(s,'Belt',(0.,.069,-.060),(.095,.004,.012),(.04,.04,.05),False)
box(s,'Buckle',(0.,.072,-.060),(.020,.003,.014),(.90,.67,.08),False)
cylinder(s,'Head',(0.,.005,.0745),.060,.080,(.91,.68,.47))
box(s,'MouthSideL',(-.037,.038,.018),(.014,.060,.034),(.91,.68,.47))
box(s,'MouthSideR',(.0345,.038,.018),(.009,.060,.034),(.91,.68,.47))
# Author the elliptic hat as an unscaled convex mesh: PhysX cylinder scaling
# would otherwise produce a collision/visual mismatch at the gripper.
points=[Gf.Vec3f(.0423*math.cos(i*math.tau/32),.005+.047*math.sin(i*math.tau/32),z) for z in (.111,.149) for i in range(32)]
hat=UsdGeom.Mesh.Define(s,'/Asset/Hat');hat.CreatePointsAttr(points)
hat.CreateFaceVertexCountsAttr([4]*32+[32,32]);hat.CreateFaceVertexIndicesAttr([k for i in range(32) for k in (i,(i+1)%32,(i+1)%32+32,i+32)]+list(reversed(range(32)))+list(range(32,64)))
hat.CreateSubdivisionSchemeAttr('none');UsdShade.MaterialBindingAPI.Apply(hat.GetPrim()).Bind(mat(s,'Hat',(.06,.10,.20)))
UsdPhysics.CollisionAPI.Apply(hat.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(hat.GetPrim()).CreateApproximationAttr('convexHull')
cylinder(s,'HatBrim',(0.,.005,.111),.052,.006,(.04,.06,.10),False)
box(s,'HatBand',(0.,.053,.125),(.050,.004,.008),(.85,.62,.10),False)
box(s,'HatBadge',(0.,.056,.133),(.013,.003,.016),(.95,.70,.12),False)
box(s,'CoatGoldL',(-.023,.069,-.028),(.005,.004,.055),(.90,.62,.09),False)
box(s,'CoatGoldR',(.023,.069,-.028),(.005,.004,.055),(.90,.62,.09),False)
box(s,'BeardL',(-.052,.055,.042),(.016,.006,.035),(.91,.90,.84),False)
box(s,'BeardR',(.052,.055,.042),(.016,.006,.035),(.91,.90,.84),False)
box(s,'UpperLip',(0.,.040,.034),(.060,.050,.008),(.93,.80,.65))
for x in (-.022,.022):box(s,'Eye'+('L' if x<0 else 'R'),(x,.060,.094),(.010,.003,.010),(.03,.03,.03),False)
box(s,'Nose',(0.,.063,.074),(.014,.015,.018),(.83,.52,.36),False)
box(s,'MoustacheL',(-.018,.060,.052),(.036,.008,.009),(.12,.08,.045),False)
box(s,'MoustacheR',(.018,.060,.052),(.036,.008,.009),(.12,.08,.045),False)
cylinder(s,'RightSleeve',(.079,.020,-.027),.014,.078,(.62,.04,.03))
cylinder(s,'RightShoulder',(.079,.020,.013),.016,.009,(.88,.66,.15),False)
cylinder(s,'RightCuff',(.079,.020,-.067),.015,.008,(.93,.84,.65),False)
cylinder(s,'RightHand',(.079,.020,-.073),.013,.022,(.92,.68,.45))
cylinder(s,'LeftSleeve',(-.079,.020,-.027),.014,.078,(.62,.04,.03))
cylinder(s,'LeftShoulder',(-.079,.020,.013),.016,.009,(.88,.66,.15),False)
cylinder(s,'LeftCuff',(-.079,.020,-.067),.015,.008,(.93,.84,.65),False)
cylinder(s,'LeftHand',(-.079,.020,-.073),.013,.022,(.92,.68,.45))
# Reinforced physical hinge parts, with actual external envelopes.
for side,x in (('L',-.033),('R',.033)):
 box(s,'HingeCheek'+side,(x,.020,0.),(.006,.060,.025),(.55,.57,.59))
for side,x in (('L',-.0215),('R',.0215)):box(s,'HingeBackplate'+side,(x,-.013,0.),(.029,.006,.025),(.55,.57,.59))
s.GetRootLayer().Save()
s=stage('lower',.225)
box(s,'LowerJaw',(0.,.045,0.),(.0596,.060,.008),(.91,.73,.54))
for side,x in (('L',-.0278),('R',.0278)):box(s,'JawGuide'+side,(x,.040,.010),(.004,.050,.012),(.55,.57,.59))
for i,y in enumerate((.025,.040,.055)):box(s,'JawTooth'+str(i),(0.,y,.0055),(.058,.002,.003),(.90,.88,.79))
for name,x in (('L',-.014),('R',.014)):box(s,'FrontTooth'+name,(x,.071,.012),(.010,.005,.016),(.90,.88,.79))
box(s,'JawPivot',(0.,-.025,0.),(.028,.060,.012),(.45,.31,.18))
box(s,'HingeBoss',(0.,.020,0.),(.0596,.012,.012),(.55,.57,.59))
cylinder(s,'HingeAxle',(0.,.020,0.),.003,.092,(.55,.57,.59)).CreateAxisAttr('X')
cylinder(s,'StopHub',(.042,.020,0.),.007,.004,(.55,.57,.59)).CreateAxisAttr('X')
# Centered, separate rear lever. Its rigid link continues through the body
# to the lower jaw; overlap with the body is filtered as an internal mechanism.
box(s,'LeverLink',(0.,-.035,-.012),(.012,.180,.012),(.46,.28,.14))
box(s,'StopPad',(.042,-.010,-.0195),(.004,.010,.003),(.55,.57,.59))
# Outboard crank shares the jaw's keyed hinge shaft. Stay outside all fixed
# cheeks/backplate/spine; the relieved right cheek/coat envelopes clear it.
crank=box(s,'StopCrank',(.042,.005,-.00975),(.004,.03578,.006),(.55,.57,.59))
crank.AddRotateXOp().Set(math.degrees(math.atan2(.0195,.030)))
ops=crank.GetOrderedXformOps();crank.SetXformOpOrder([ops[0],ops[2],ops[1]])
box(s,'RearLever',(0.,-.120,-.010),(.018,.018,.100),(.46,.28,.14))
box(s,'LeverGrip',(0.,-.120,.040),(.080,.024,.024),(.72,.50,.27))
s.GetRootLayer().Save()
for name,sign in [('shell_a',1),('shell_b',-1)]:
 s=stage(name,.007);half(s,'Shell',sign);s.GetRootLayer().Save()
(R/'assets/hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((R/'assets').glob('*.usda'))},indent=2)+'\n')
