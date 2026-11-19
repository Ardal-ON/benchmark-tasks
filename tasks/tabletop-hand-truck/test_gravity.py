import unittest,numpy as np
from cart_task.gravity import torque,potential
class GravityModel(unittest.TestCase):
 def test_gravity_torque_matches_potential_work(self):
  for q in (np.array([.3,1.2,1.5,.2,.3,-.4]),np.array([-.5,2.9,1.8,.1,-.1,1.5])):
   numeric=np.array([(potential(q+np.eye(6)[i]*1e-5)-potential(q-np.eye(6)[i]*1e-5))/2e-5 for i in range(6)])
   np.testing.assert_allclose(torque(q),numeric,atol=1e-7)
 def test_near_vertical_yaw_has_negligible_gravity_torque(self):
  self.assertLess(abs(torque(np.array([.5,2.,1.,.3,.2,0.]))[0]),1e-4) # rounded URDF axis
if __name__=='__main__':unittest.main()
