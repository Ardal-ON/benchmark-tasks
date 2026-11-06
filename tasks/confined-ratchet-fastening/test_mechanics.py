import math,unittest
from ratchet_task.mechanics import Clutch,Screw
class Mechanics(unittest.TestCase):
    def test_clutch_backlash_return_and_drive(self):
        c=Clutch();c.reset()
        self.assertEqual(c.step(math.radians(4),.1),.0002)
        self.assertGreater(c.step(math.radians(15),.1),.5)
        self.assertLess(abs(c.step(math.radians(-15),-.1)),.003)
        self.assertLess(c.step(math.radians(-11),.1),.003)
    def test_helix_pitch_and_direction(self):
        s=Screw();s.capture(.1,0)
        for theta in [-2*math.pi,2*math.pi]:
            force,torque,_=s.forces(.1+s.pitch*theta/(2*math.pi),theta,0,0,.01)
            self.assertAlmostEqual(force,0);self.assertAlmostEqual(torque,0)
    def test_helix_restoring_power(self):
        s=Screw();s.capture(0,0)
        f,t,_=s.forces(.0001,0,.01,0,.01)
        self.assertLess(f,0);self.assertLess(f*.01,0)
        self.assertGreater(t,0)
    def test_no_capture_no_force(self):self.assertEqual(Screw().forces(0,0,0,1,0),(0.,0.,0.))
    def test_seating_and_unscrewing_release_preload(self):
        s=Screw();s.capture(0,0);s.forces(0,0,0,0,0)
        self.assertAlmostEqual(s.forces(0,-.25,0,0,0)[2],.5)
        self.assertEqual(s.forces(0,.1,0,0,.001)[2],0.)
if __name__=='__main__':unittest.main()
