"""Original analytic GF d32 fixed-flange/pipe envelopes; no downloaded CAD.

Asset generation needs NumPy/SciPy. Runtime only reads the generated USD files.
Holes use32-sided inscribed polygons (at most0.034mm radial conservatism).
"""
import math,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial import Delaunay
from flange_assembly.specs import PIPE_LENGTH,ELBOW_INTERSECTION,MOVING_MASS,MOVING_COM,HOLE_XY
ROOT=Path(__file__).parent/'assets'
ROOT.mkdir(exist_ok=True)
FACES=((0,2,1),(3,4,5),(0,1,4,3),(1,2,5,4),(2,0,3,5))
COLOR=(.24,.27,.29)
def mesh(name,points,faces,color=COLOR):
    pts=', '.join('('+','.join(f'{v:.9f}' for v in p)+')' for p in points)
    return f'''    def Mesh "{name}" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"])
    {{
        point3f[] points = [{pts}]
        int[] faceVertexCounts = [{','.join(str(len(f)) for f in faces)}]
        int[] faceVertexIndices = [{','.join(str(i) for f in faces for i in f)}]
        uniform token subdivisionScheme = "none"
        uniform token physics:approximation = "convexHull"
        color3f[] primvars:displayColor = [({','.join(map(str,color))})]
    }}\n'''
def ring(prefix,inner,outer,z0,z1,n=32,axis='Z',origin=(0.,0.,0.)):
    text=''
    for i in range(n):
        a,b=2*math.pi*i/n,2*math.pi*(i+1)/n
        pts=[(r*math.cos(t),r*math.sin(t),z) for z in (z0,z1) for r,t in ((inner,a),(outer,a),(outer,b),(inner,b))]
        if axis=='Y':pts=[(x,z,y) for x,y,z in pts]
        pts=[tuple(v+o for v,o in zip(p,origin)) for p in pts]
        text+=mesh(f'{prefix}_{i}',pts,((0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)))
    return text
# Plate material triangulation: exact polygon area catches accidental missing wedges.
circles=[(0.,0.,.0136,48)]+[(x,y,.007,32) for x,y in HOLE_XY]
points=[(.0575*math.cos(a),.0575*math.sin(a)) for a in np.linspace(0,2*math.pi,128,endpoint=False)]
for x,y,r,n in circles:points += [(x+r*math.cos(a),y+r*math.sin(a)) for a in np.linspace(0,2*math.pi,n,endpoint=False)]
points=np.asarray(points);plate='';area=0.;count=0
for ids in Delaunay(points).simplices:
    tri=points[ids]
    valid=True
    for x,y,r,n in circles:
        center=np.array([x,y]);rr=r*math.cos(math.pi/n)
        cross=[np.cross(tri[(i+1)%3]-tri[i],center-tri[i]) for i in range(3)]
        if min(cross)>=-1e-12 or max(cross)<=1e-12:valid=False;break
        for i in range(3):
            a,b=tri[i],tri[(i+1)%3];ab=b-a;t=np.clip(np.dot(center-a,ab)/np.dot(ab,ab),0,1)
            if np.linalg.norm(a+t*ab-center)<rr-1e-10:valid=False;break
        if not valid:break
    if not valid:continue
    area+=abs(float(np.cross(tri[1]-tri[0],tri[2]-tri[0])))/2
    plate+=mesh(f'Plate_{count}',[(x,y,z) for z in (0.,.014) for x,y in tri],FACES);count+=1
expected=128*.0575**2*math.sin(2*math.pi/128)/2-sum(n*r*r*math.sin(2*math.pi/n)/2 for x,y,r,n in circles)
assert abs(area-expected)<1e-10,(area,expected)
def asset(name,contents,mass=None):
    schema=' (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "MaterialBindingAPI"])' if mass else ' (prepend apiSchemas = ["MaterialBindingAPI"])'
    attrs=f'    float physics:mass = {mass}\n' if mass else ''
    if name=='moving_pipe':attrs+=f'    point3f physics:centerOfMass = {MOVING_COM}\n'
    color=(.44,.47,.49) if name=='v_rest' else (.24,.27,.29)
    attrs+='    rel material:binding = </Asset/Looks/Surface>\n'
    attrs+=f'''    def Scope "Looks" {{
        def Material "Surface" {{
            token outputs:surface.connect = </Asset/Looks/Surface/Shader.outputs:surface>
            def Shader "Shader" {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = ({','.join(map(str,color))})
                float inputs:metallic = {0.5 if name=='v_rest' else 0.}
                float inputs:roughness = 0.45
                token outputs:surface
            }}
        }}
    }}
'''
    (ROOT/(name+'.usda')).write_text('#usda 1.0\n( defaultPrim="Asset"\n metersPerUnit=1\n upAxis="Z" )\ndef Xform "Asset"'+schema+'\n{\n'+attrs+contents+'}\n')
for name,length in (('fixed_pipe',.2),('moving_pipe',PIPE_LENGTH)):
    # Socket region begins3mm behind face; the cut length comes from specs.py.
    contents=plate+ring('Socket',.016,.0205,.014,.025)+ring('Pipe',.0136,.016,.003,length+.003,48)
    if name=='moving_pipe':
        # GF721100108 elbow:39mm socket-to-intersection,40mm OD.
        # Two intersecting annular envelopes simplify the internal elbow passage.
        contents+=ring('ElbowAxial',.016,.020,ELBOW_INTERSECTION-.039,ELBOW_INTERSECTION)
        contents+=ring('ElbowBranch',.016,.020,0.,.039,axis='Y',origin=(0.,0.,ELBOW_INTERSECTION))
        contents+=ring('Branch',.0136,.016,.017,.217,48,axis='Y',origin=(0.,0.,ELBOW_INTERSECTION))
    asset(name,contents,MOVING_MASS if name=='moving_pipe' else None)
# Two45-degree smooth V rests; notch vertex147.3726mm above tabletop.
vertex=.17-.016*math.sqrt(2);height=vertex+.03
parts=''
for side in (-1,1):
    yz=[(0.,0.),(side*.03,0.),(side*.03,height),(0.,vertex)]
    pts=[(x,y,z) for x in (-.015,.015) for y,z in yz]
    parts+=mesh('Rest_'+str(side).replace('-','m'),pts,((0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)),(.44,.47,.49))
asset('v_rest',parts)
report={'flange_code':'721730208','pipe_code':'161017108','moving_mass_kg':MOVING_MASS,'elbow_code':'721100108','branch_cut_length_m':.2,'plate_convex_triangles':count,'plate_area_m2':area,'expected_polygon_area_m2':expected,'hole_diameter_mm':14,'max_hole_radial_conservatism_mm':7*(1-math.cos(math.pi/32)),'moving_local_bbox_mm':[[-57.5,-57.5,0],[57.5,217,1000*(ELBOW_INTERSECTION+.020)]],'fixed_local_bbox_mm':[[-57.5,-57.5,0],[57.5,57.5,203]],'v_vertex_above_table_mm':vertex*1000,'assets':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ROOT.glob('*.usda')) if p.stem in ('fixed_pipe','moving_pipe','v_rest')}}
(ROOT/'analytic-geometry.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
