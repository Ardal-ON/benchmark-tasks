"""Reject dragging, spinning, carrying and an unreleased or unstable finish."""
import unittest
from cart_task.scoring import rolling_complete,stage_score
class CartRules(unittest.TestCase):
 def rolling(self,s=.15,d=.15,t=4.8):
  return rolling_complete(dict.fromkeys(('wheel_left','wheel_right'),s),dict.fromkeys(('wheel_left','wheel_right'),d),dict.fromkeys(('wheel_left','wheel_right'),t),.03125)
 def finish(self,**kw):
  a=dict(failed=False,tipped=True,rolling=True,axle=(.31,.10,.77875),angle=0.,clear=True,stable=True,hold=19);a.update(kw);return stage_score(**a)
 def test_real_roll(self):self.assertTrue(self.rolling())
 def test_drag_spin_and_carry_rejected(self):
  for kw in ({'t':0.},{'t':10.},{'s':0.},{'d':0.},{'t':-4.8}):self.assertFalse(self.rolling(**kw))
 def test_both_wheels_must_roll(self):
  self.assertFalse(rolling_complete({'wheel_left':.15,'wheel_right':.1},dict.fromkeys(('wheel_left','wheel_right'),.15),dict.fromkeys(('wheel_left','wheel_right'),4.8),.03125))
 def test_tip_stage_is_only_quarter(self):self.assertEqual(self.finish(rolling=False),(0.25,0))
 def test_destination_without_release_is_half(self):self.assertEqual(self.finish(clear=False),(.5,0))
 def test_finish_needs_full_second_of_stability(self):
  self.assertEqual(self.finish(hold=18),(.5,19));self.assertEqual(self.finish(),(1.,20));self.assertEqual(self.finish(stable=False),(.5,0))
 def test_wrong_destination_cannot_pass(self):self.assertEqual(self.finish(axle=(.36,.1,.77875)),(.25,0))
 def test_fault_erases_all_stages(self):self.assertEqual(self.finish(failed=True),(0.,0))
 def test_tilted_cart_cannot_finish(self):self.assertEqual(self.finish(angle=.1),(.5,0))
if __name__=='__main__':unittest.main()
