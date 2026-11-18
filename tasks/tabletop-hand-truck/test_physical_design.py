"""Independent mass-budget and native asset checks."""
import json,math,unittest
from pathlib import Path
import numpy as np
from pxr import Usd,UsdPhysics
ROOT=Path(__file__).resolve().parent
D=json.loads((ROOT/'physical_design.json').read_text())
class PhysicalDesign(unittest.TestCase):
 def test_product_scale_and_mass(self):
  s=.300/(48*.0254)
  self.assertAlmostEqual(D['derived']['reference_scaled_base_mass_kg'],24*.45359237*s**3)
  self.assertAlmostEqual(D['derived']['wheel_mass_kg'],3*.45359237*s**3)
  self.assertAlmostEqual(D['derived']['wheel_diameter_m'],10*.0254*s)
 def test_every_frame_part_mass_from_volume(self):
  for p in D['parts']:
   g=p['geometry']
   if g['type']=='box':v=math.prod(g['size_m'])
   else:v=math.pi*(g['outer_radius_m']**2-g['inner_radius_m']**2)*g['length_m']
   self.assertAlmostEqual(p['mass_kg'],v*g['density_kg_m3'],places=12)
 def test_combined_com_and_parallel_axis_inertia(self):
  mass=sum(p['mass_kg'] for p in D['parts']);com=sum(p['mass_kg']*np.array(p['centre_m']) for p in D['parts'])/mass
  I=np.zeros((3,3))
  for p in D['parts']:
   r=np.array(p['centre_m'])-com;I+=np.array(p['inertia'])+p['mass_kg']*((r@r)*np.eye(3)-np.outer(r,r))
  self.assertAlmostEqual(mass,D['derived']['frame_mass_kg']);np.testing.assert_allclose(com,D['derived']['frame_com_m'],atol=1e-12);np.testing.assert_allclose(I,D['derived']['frame_inertia_kg_m2'],atol=1e-12)
 def test_authored_mass_matches_ledger(self):
  stage=Usd.Stage.Open(str(ROOT/'assets/cart.usda'));actual=[]
  for p in stage.Traverse():
   if p.HasAPI(UsdPhysics.MassAPI):actual.append(UsdPhysics.MassAPI(p).GetMassAttr().Get())
  self.assertEqual(len(actual),3);np.testing.assert_allclose(sorted(actual),sorted([D['derived']['frame_mass_kg'],D['derived']['wheel_mass_kg'],D['derived']['wheel_mass_kg']]),rtol=1e-6)
 def test_bearings_cannot_propel_wheels(self):
  stage=Usd.Stage.Open(str(ROOT/'assets/cart.usda'));joints=[p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
  self.assertEqual(len(joints),2)
  for p in joints:
   for name in ('drive:angular:physics:stiffness','drive:angular:physics:targetVelocity','drive:angular:physics:targetPosition'):self.assertFalse(p.GetAttribute(name).Get())
if __name__=='__main__':unittest.main()
