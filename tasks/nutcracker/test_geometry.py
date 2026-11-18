import unittest
import numpy as np
from nutcracker_task.geometry import rim_opening,fixture_stable,table_bounds_ok,supported,grippers_clear
class GeometryTests(unittest.TestCase):
    def test_shear_is_not_opening(self):
        I=np.eye(3);a=np.array([0.,.008,0.]);b=np.array([0.,-.008,0.])
        self.assertAlmostEqual(rim_opening(a+[.01,0,0],I,b,I),0.)
        self.assertAlmostEqual(rim_opening(a+[0,.003,0],I,b,I),.003)
    def test_common_rotation_is_not_a_crack(self):
        t=.8;c,s=np.cos(t),np.sin(t);r=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        self.assertAlmostEqual(rim_opening(r@[0.,.008,0.],r,r@[0.,-.008,0.],r),0.)
    def test_normal_jaw_rotation_keeps_mount_stable(self):
        O=np.array([.32,-.04,.8675]);P=np.array([0,.02,0]);t=-np.pi/4
        r=np.array([[1,0,0],[0,np.cos(t),-np.sin(t)],[0,np.sin(t),np.cos(t)]])
        poses=dict(upper=O,lower=O+P-r@P);rot=dict(upper=np.eye(3),lower=r)
        self.assertTrue(fixture_stable(poses,rot,O,np.eye(3)))
        poses['lower']=poses['lower']+[.011,0,0]
        self.assertFalse(fixture_stable(poses,rot,O,np.eye(3)))
    def test_off_table_or_airborne_cannot_earn_containment(self):
        self.assertFalse(table_bounds_ok([np.array([1.,0,.8])]))
        self.assertFalse(supported([100,100],0))
        self.assertTrue(supported([0,100],0))
        self.assertFalse(supported([0,100],25))
    def test_root_clear_but_surface_contact_fails(self):
        # Tall figure at origin: TCP is 200 mm from origin but touches top.
        hand=dict(tcp=np.array([0.,0.,.2]),fingers=np.array([-.047,-.047]))
        ok,d=grippers_clear({'left':hand},{'figure':np.zeros(3)},
            {'figure':np.eye(3)},[('figure',np.array([0.,0.,.1]),np.array([.05,.05,.1]))])
        self.assertFalse(ok);self.assertLess(d,.08)
if __name__=='__main__':unittest.main()
