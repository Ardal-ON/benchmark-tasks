"""Reproducible original CAD-like meshes and open compound colliders (SI units)."""
from pathlib import Path
import json, math, hashlib
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
from ratchet_task.config import *
ROOT=Path(__file__).resolve().parent
STEEL=(.46,.49,.53); BRACKET=(.17,.29,.40); DARK=(.12,.14,.16)
def stage(name,mass=None):
    s=Usd.Stage.CreateNew(str(ROOT/'assets'/f'{name}.usda'))
    UsdGeom.SetStageMetersPerUnit(s,1.); UsdGeom.SetStageUpAxis(s,'Z')
    p=UsdGeom.Xform.Define(s,'/Asset').GetPrim();s.SetDefaultPrim(p)
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(p);UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
    return s

def material(s,name,rgb,metal=.0,rough=.4):
    path='/Asset/Materials/'+name
    mat=UsdShade.Material.Define(s,path);shader=UsdShade.Shader.Define(s,path+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    for key,typ,val in [('diffuseColor',Sdf.ValueTypeNames.Color3f,Gf.Vec3f(*rgb)),('metallic',Sdf.ValueTypeNames.Float,metal),('roughness',Sdf.ValueTypeNames.Float,rough)]:shader.CreateInput(key,typ).Set(val)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    return mat

def mesh(s,name,verts,faces,rgb=STEEL,collide=False,visible=True):
    p=UsdGeom.Mesh.Define(s,'/Asset/'+name)
    p.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in verts]);p.CreateFaceVertexCountsAttr([len(f) for f in faces]);p.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);p.CreateSubdivisionSchemeAttr('none')
    p.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(material(s,name.replace('/','_'),rgb,.85 if rgb==STEEL else .25,.28))
    if collide:
        UsdPhysics.CollisionAPI.Apply(p.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(p.GetPrim()).CreateApproximationAttr('convexHull')
    if not visible:p.CreateVisibilityAttr('invisible')
    return p

def prism(s,name,xy,z0,z1,rgb=STEEL,collide=True,visible=True):
    n=len(xy);v=[(*p,z) for z in [z0,z1] for p in xy]
    f=[list(reversed(range(n))),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    return mesh(s,name,v,f,rgb,collide,visible)

def ring(s,name,rin,rout,z0,z1,n=48,rgb=STEEL,rect=None,collide=True):
    for i in range(n):
        a,b=2*np.pi*np.array([i,i+1])/n
        def outer(t):
            r=rout if rect is None else min(rect[0]/max(abs(np.cos(t)),1e-12),rect[1]/max(abs(np.sin(t)),1e-12))
            return r*np.array([np.cos(t),np.sin(t)])
        xy=[rin*np.array([np.cos(a),np.sin(a)]),outer(a),outer(b),rin*np.array([np.cos(b),np.sin(b)])]
        prism(s,f'{name}/Sector{i:02}',xy,z0,z1,rgb,collide)

def box(s,name,p,size,rgb=DARK,collide=True):
    o=UsdGeom.Cube.Define(s,'/Asset/'+name);o.CreateSizeAttr(1.);o.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));o.AddScaleOp().Set(Gf.Vec3f(*size));o.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdShade.MaterialBindingAPI.Apply(o.GetPrim()).Bind(material(s,name.replace('/','_'),rgb,.35,.45))
    if collide:UsdPhysics.CollisionAPI.Apply(o.GetPrim())
    return o

def bolt():
    s=stage('bolt',BOLT_MASS)
    # Bevelled hex head; collision uses the exact head envelope.
    n=6;r=HEAD_AF/np.sqrt(3)
    xy=r*np.array([[np.cos(a),np.sin(a)] for a in np.arange(n)*np.pi/3+np.pi/6])
    prism(s,'HeadContact',xy,0,HEAD_HEIGHT,STEEL,True,False)
    verts=[]
    for z,scale in [(0,.92),(.0004,1),(HEAD_HEIGHT-.0005,1),(HEAD_HEIGHT,.90)]:verts.extend([(*p*scale,z) for p in xy])
    faces=[list(reversed(range(6))),list(range(18,24))]+[[j*6+i,j*6+(i+1)%6,(j+1)*6+(i+1)%6,(j+1)*6+i] for j in range(3) for i in range(6)]
    mesh(s,'MachinedHead',verts,faces)
    # Smooth minor-diameter collider; helical ridge is visual. Physical thread uses a native PhysX joint constraint.
    angles=np.arange(48)*2*np.pi/48
    prism(s,'ShankContact',.00365*np.c_[np.cos(angles),np.sin(angles)],-BOLT_LENGTH,-.00005,STEEL,True,False)
    # Continuous thread surface profile sampled axially, with a real spiral crest.
    rings=256;n=64;verts=[]
    for z in np.linspace(-BOLT_LENGTH,0,rings):
        for a in np.arange(n)*2*np.pi/n:
            phase=((z/PITCH-a/(2*np.pi))%1)
            r=.00325+.00075*max(0,1-abs(phase-.5)/.42)
            r=min(r,.0029+.0011*min(1,(z+BOLT_LENGTH)/.001))
            verts.append((r*np.cos(a),r*np.sin(a),z))
    faces=[[j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i] for j in range(rings-1) for i in range(n)]
    mesh(s,'ThreadVisual',verts,faces);s.GetRootLayer().Save()

def build():
    (ROOT/'assets').mkdir(exist_ok=True);bolt()
    s=stage('fixture');box(s,'Base',[0,0,.006],[.18,.16,.012])
    ring(s,'Receiver',.00435,.045,.012,.090,n=48,rgb=DARK,rect=(.045,.04))
    ring(s,'Bracket',.0046,.06,.090,.093,n=48,rgb=BRACKET,rect=(.07,.045))
    s.GetRootLayer().Save()
    s=stage('bolt_holder');ring(s,'Holder',.0044,.014,0,.022,n=32,rgb=BRACKET);box(s,'Foot',[0,0,.002],[.044,.044,.004]);s.GetRootLayer().Save()
    s=stage('socket',.040)
    # Hexagonal physical cavity: inner AF 13.4mm, 0.2mm clearance per face.
    n=6;rin=.0134/np.sqrt(3);rout=.0105
    for i in range(n):
        a,b=np.pi/6+2*np.pi*np.array([i,i+1])/n
        xy=[rin*np.array([np.cos(a),np.sin(a)]),rout*np.array([np.cos(a),np.sin(a)]),rout*np.array([np.cos(b),np.sin(b)]),rin*np.array([np.cos(b),np.sin(b)])]
        prism(s,f'HexWall{i}',xy,0,.009)
    ring(s,'UpperBarrel',.0049,.0105,.009,.022,n=48)
    # 3/8in drive square; cavity remains open.
    inner=.0097/2;outer=.0105
    box(s,'DriveFront',[0,(outer+inner)/2,.025],[2*outer,outer-inner,.006],STEEL)
    box(s,'DriveBack',[0,-(outer+inner)/2,.025],[2*outer,outer-inner,.006],STEEL)
    box(s,'DriveLeft',[(outer+inner)/2,0,.025],[outer-inner,2*inner,.006],STEEL)
    box(s,'DriveRight',[-(outer+inner)/2,0,.025],[outer-inner,2*inner,.006],STEEL)
    s.GetRootLayer().Save()
    s=stage('walls')
    from asset_geometry import add_neck_walls
    add_neck_walls(s)
    s.GetRootLayer().Save()
    s=stage('tool_cradle');box(s,'Foot',[0,0,.004],[.23,.065,.008]);
    for x in [-.12,0]:
        for y in [-.026,.026]:box(s,f'Support{str(x).replace("-","N").replace(".","D")}{str(y).replace("-","N").replace(".","D")}',[x,y,.016],[.018,.012,.024],BRACKET)
    s.GetRootLayer().Save()
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'assets').glob('*.usda'))}
    (ROOT/'assets/generated-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    (ROOT/'assets/generated-provenance.json').write_text(json.dumps({'source':'Original parametric geometry','builder':'build_assets.py','units':'m, kg','thread_pitch_m':PITCH,'bolt_underhead_length_m':BOLT_LENGTH,'bolt_head_across_flats_m':HEAD_AF,'simplifications':['Thread ridge is visual; native PhysX helical constraint supplies force behavior','Smooth minor-diameter shank collider','Socket/receiver holes preserved by compound convex pieces']},indent=2)+'\n')
if __name__=='__main__':build()

def build_tool():
    """Two-body ratchet. Socket is the drive body's permanently assembled tool."""
    s=stage('handle',.378)
    p=UsdGeom.Xform.Define(s,'/Asset/Visual').GetPrim();p.GetReferences().AddReference('ratchet_visual.usda','/RatchetVisual/Handle')
    from asset_geometry import add_handle_colliders
    report,_=add_handle_colliders(s,ROOT/'assets/ratchet_visual.usda')
    (ROOT/'assets/handle-collision-provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    s.GetRootLayer().Save()
    s=stage('drive',.086)
    p=UsdGeom.Xform.Define(s,'/Asset/DriveVisual').GetPrim();p.GetReferences().AddReference('ratchet_visual.usda','/RatchetVisual/Drive')
    socket=UsdGeom.Xform.Define(s,'/Asset/Socket');socket.GetPrim().GetReferences().AddReference('socket.usda','/Asset');socket.AddTranslateOp().Set(Gf.Vec3d(0,0,-.028))
    # The socket is permanently part of this rigid drive; strip nested body schemas.
    socket.GetPrim().RemoveAPI(UsdPhysics.RigidBodyAPI);socket.GetPrim().RemoveAPI(UsdPhysics.MassAPI)
    # Root-layer delete list is required to suppress schemas inherited by reference.
    from pxr import Sdf
    socket.GetPrim().SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit([]))
    s.GetRootLayer().Save()
if __name__=='__main__' and (ROOT/'assets/ratchet_visual.usda').exists():build_tool()

if __name__=='__main__':
    for p in (ROOT/'assets').glob('*.usda'):p.write_text(p.read_text().rstrip()+'\n')
    (ROOT/'assets/generated-hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'assets').glob('*.usda'))},indent=2)+'\n')
