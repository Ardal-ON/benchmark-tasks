import json,unittest
from pathlib import Path
import numpy as np
from pxr import Usd,UsdPhysics,UsdShade
ROOT=Path(__file__).resolve().parent;D=json.loads((ROOT/'physical_design.json').read_text())
class PhysicalDesign(unittest.TestCase):
 def test_actual_product_not_scaled(self):
  self.assertEqual(D['scale'],1.)
  for field,x in [('height_m',41*.0254),('width_m',15.5*.0254),('length_m',17.5*.0254),('total_mass_kg',9*.45359237)]:self.assertAlmostEqual(D['derived'][field],x)
  self.assertGreater(D['derived']['wheel_mass_kg'],0.);self.assertAlmostEqual(D['derived']['wheel_diameter_m'],.127)
 def test_component_mass_budget(self):
  self.assertAlmostEqual(sum(D['part_groups'].values())+2*D['derived']['wheel_mass_kg'],D['derived']['total_mass_kg'])
  self.assertAlmostEqual(sum(p['mass_kg'] for p in D['parts']),D['derived']['frame_mass_kg'])
 def test_compound_com_and_inertia(self):
  parts=D['parts'];mass=sum(p['mass_kg'] for p in parts);com=sum(p['mass_kg']*np.array(p['centre_m']) for p in parts)/mass;I=np.zeros((3,3))
  for p in parts:
   r=np.array(p['centre_m'])-com;I+=np.array(p['inertia'])+p['mass_kg']*((r@r)*np.eye(3)-np.outer(r,r))
  np.testing.assert_allclose(com,D['derived']['frame_com_m'],atol=1e-12);np.testing.assert_allclose(I,D['derived']['frame_inertia_kg_m2'],atol=1e-12);self.assertTrue(np.all(np.linalg.eigvalsh(I)>0))
 def test_native_masses_and_passive_bearings(self):
  s=Usd.Stage.Open(str(ROOT/'assets/cart.usda'));actual=[UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in s.Traverse() if p.HasAPI(UsdPhysics.MassAPI)]
  np.testing.assert_allclose(sorted(actual),sorted([D['derived']['frame_mass_kg']]+[D['derived']['wheel_mass_kg']]*2),rtol=1e-6)
  for p in s.Traverse():
   if p.IsA(UsdPhysics.RevoluteJoint):
    for key in ('stiffness','targetPosition','targetVelocity'):self.assertEqual(p.GetAttribute('drive:angular:physics:'+key).Get(),0.)
 def test_modeled_rubber_handle_uses_rubber_contact(self):
  stage=Usd.Stage.Open(str(ROOT/'assets/cart.usda'))
  handle=stage.GetPrimAtPath('/Cart/frame/top_grip')
  material,_=UsdShade.MaterialBindingAPI(handle).ComputeBoundMaterial(materialPurpose='physics')
  self.assertEqual(str(material.GetPath()),'/Cart/Materials/Rubber')
  self.assertAlmostEqual(UsdPhysics.MaterialAPI(material).GetStaticFrictionAttr().Get(),.9,places=6)
 def test_telescoping_profiles_fit(self):
  parts={p['name']:p for p in D['parts']};outer=parts['lower_rail1'];inner=parts['extended_handle1']
  self.assertGreater(outer['geometry']['inner_size_m'][0],inner['geometry']['size_m'][0])
  self.assertGreater(outer['geometry']['inner_size_m'][1],inner['geometry']['size_m'][1])
  np.testing.assert_allclose(outer['centre_m'][:2],inner['centre_m'][:2])
  top=outer['centre_m'][2]+outer['geometry']['size_m'][2]/2;bottom=inner['centre_m'][2]-inner['geometry']['size_m'][2]/2
  self.assertGreaterEqual(top-bottom,.12)
 def test_wheel_collision_preserves_roundness(self):
  stage=Usd.Stage.Open(str(ROOT/'assets/cart.usda'))
  p=stage.GetPrimAtPath('/Cart/wheel_left/Contact')
  from pxr import UsdGeom
  count=len(UsdGeom.Mesh(p).GetPointsAttr().Get())
  self.assertGreaterEqual(p.GetAttribute('physxConvexHullCollision:hullVertexLimit').Get(),count)
  self.assertLess(.0635*(1-np.cos(np.pi/(count/2))),25e-6)
 def test_jaw_closing_axis_is_perpendicular_to_crossbar(self):
  from cart_task.config import GRIP_ROTS
  for rotation in GRIP_ROTS.values():self.assertLess(abs(rotation[1,0]),1e-12)
 def test_empty_cart_has_no_payload(self):
  self.assertIsNone(D['payload'])
  self.assertNotIn("Box('load'",(ROOT/'task.py').read_text())
if __name__=='__main__':unittest.main()
