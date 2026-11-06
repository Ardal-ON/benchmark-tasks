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
            self.assertAlmostEqual(s.expected_z(theta),.1+s.pitch*theta/(2*math.pi))
    def test_capture_reference_nonzero_angle(self):
        s=Screw();s.capture(.1,7.)
        self.assertAlmostEqual(s.expected_z(7.),.1)
        self.assertAlmostEqual(s.expected_z(7.-2*math.pi),.09875)
    def test_uncaptured_rejects_prediction(self):
        with self.assertRaises(RuntimeError):Screw().expected_z(0)
    def test_implicit_clutch_preserves_capacity(self):
        c=Clutch();c.reset();self.assertAlmostEqual(c.step(20,0,1/480),c.capacity)
if __name__=='__main__':unittest.main()
