import unittest
import numpy as np
from chain_task.topology import linked,CrossingAudit,segment_distance
from chain_task.geometry import layout,loop,eye_loop,STAND_POS,EYE_OFFSET,rx,rz,RADIUS,GATE_OFFSET

def circles(center=1.):
 a=np.linspace(0,2*np.pi,65);return np.column_stack([np.cos(a),np.sin(a),a*0]),np.column_stack([center+np.cos(a),a*0,np.sin(a)])
class TopologyTests(unittest.TestCase):
 def test_linked_and_unlinked(self):
  a,b=circles();self.assertTrue(linked(a,b));_,c=circles(3.);self.assertFalse(linked(a,c));self.assertFalse(linked(a,a+[0,0,.1]));self.assertFalse(linked(a,a+[2.1,0,0]))
 def test_tangent_and_coplanar_conservative(self):
  a,b=circles(2.);self.assertFalse(linked(a,b));self.assertFalse(linked(a,a));self.assertFalse(linked(a,a+[.5,0,0]))
 def test_vertex_crossing_and_rigid_transform(self):
  a,b=circles();r=rz(.7)@rx(.4);p=np.array([.3,-.1,.8]);self.assertTrue(linked(a@r.T+p,b@r.T+p));self.assertTrue(linked(a[::-1],b[::-1]))
 def test_closed_loop_crossing_invalidates(self):
  a,b=circles();h=CrossingAudit();h.update({'eye':a,'link':b+[4,0,0]},{'eye':True,'link':True});h.update({'eye':a,'link':b},{'eye':True,'link':True});self.assertTrue(h.invalid)
  h=CrossingAudit();h.update({'eye':a,'link':b+[4,0,0]},{'eye':True,'link':True});h.update({'eye':a,'link':b},{'eye':True,'link':False});h.update({'eye':a,'link':b},{'eye':True,'link':True});self.assertFalse(h.invalid)
 def test_remote_gate_open_does_not_authorize_later_tunneling(self):
  a,b=circles();h=CrossingAudit();h.update({'eye':a,'link':b+[5,0,0]},{'eye':True,'link':False});h.update({'eye':a,'link':b+[5,0,0]},{'eye':True,'link':True});h.update({'eye':a,'link':b},{'eye':True,'link':True});self.assertTrue(h.invalid)
 def test_original_carabiner_hole_and_suspended_link(self):
  eye=eye_loop();r=rz(np.deg2rad(-10))@rx(np.pi/2);p=STAND_POS+EYE_OFFSET+[0,0,-.025+.00475-.030];g=p+r@GATE_OFFSET;self.assertTrue(linked(eye,loop(p,r,g,r)))
 def test_layout_reproducibility_and_two_extra_seeds(self):
  for seed in [17,23,41]:
   a,b=layout(seed),layout(seed)
   for k in a:np.testing.assert_array_equal(a[k]['position'],b[k]['position']);np.testing.assert_array_equal(a[k]['rotation'],b[k]['rotation'])
 def test_segment_distances(self):
  self.assertAlmostEqual(segment_distance([0,0,0],[1,0,0],[.5,-1,0],[.5,1,0]),0.);self.assertAlmostEqual(segment_distance([0,0,0],[1,0,0],[0,1,0],[1,1,0]),1.)
if __name__=='__main__':unittest.main()
