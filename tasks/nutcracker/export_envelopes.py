"""Export the authored millimetre outer envelopes for physical fabrication."""
import csv
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics
root=Path(__file__).resolve().parent
with (root/'fabrication-envelope.csv').open('w') as stream:
    writer=csv.writer(stream)
    writer.writerow(['body','part','shape','center_x_mm','center_y_mm','center_z_mm',
                     'size_x_mm','size_y_mm','size_z_mm','colliding'])
    for body in ('fixture','upper','lower'):
        stage=Usd.Stage.Open(str(root/'assets'/f'{body}.usda'))
        for prim in stage.GetDefaultPrim().GetChildren():
            if not prim.IsA(UsdGeom.Gprim):continue
            bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default']).ComputeLocalBound(prim).ComputeAlignedRange()
            center=bbox.GetMidpoint(); size=bbox.GetSize()
            writer.writerow([body,prim.GetName(),prim.GetTypeName(),
                *[round(v*1000,3) for v in center],*[round(v*1000,3) for v in size],
                prim.HasAPI(UsdPhysics.CollisionAPI)])
