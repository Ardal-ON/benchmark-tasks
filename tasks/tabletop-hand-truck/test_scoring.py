"""Reject dragging, spinning, carrying and an unreleased or unstable finish."""
import unittest
from cart_task.scoring import rolling_complete,stage_score,pose_stable,load_bearing_contact,supported_motion,support_lost
class CartRules(unittest.TestCase):
 def rolling(self,s=.15,d=.15,t=.15/.0635):
  return rolling_complete(dict.fromkeys(('wheel_left','wheel_right'),s),dict.fromkeys(('wheel_left','wheel_right'),d),dict.fromkeys(('wheel_left','wheel_right'),t),.0635)
 def finish(self,**kw):
  a=dict(failed=False,tipped=True,rolling=True,axle=(.35,0.,.051),angle=0.,clear=True,stable=True,hold=19);a.update(kw);return stage_score(**a)
 def test_real_roll(self):self.assertTrue(self.rolling())
 def test_drag_spin_and_carry_rejected(self):
  for kw in ({'t':0.},{'t':10.},{'s':0.},{'d':0.},{'t':-4.8}):self.assertFalse(self.rolling(**kw))
 def test_both_wheels_must_roll(self):
  self.assertFalse(rolling_complete({'wheel_left':.15,'wheel_right':.1},dict.fromkeys(('wheel_left','wheel_right'),.15),dict.fromkeys(('wheel_left','wheel_right'),.15/.0635),.0635))
 def test_tip_stage_is_only_quarter(self):self.assertEqual(self.finish(rolling=False),(0.25,0))
 def test_destination_without_release_is_half(self):self.assertEqual(self.finish(clear=False),(.5,0))
 def test_finish_needs_full_second_of_stability(self):
  self.assertEqual(self.finish(hold=18),(.5,19));self.assertEqual(self.finish(),(1.,20));self.assertEqual(self.finish(stable=False),(.5,0))
 def test_wrong_destination_cannot_pass(self):self.assertEqual(self.finish(axle=(.40,0.,.051)),(.25,0))
 def test_spin_in_place_is_not_parked(self):
  import numpy as np,math
  a=math.radians(1.5);R=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
  self.assertFalse(pose_stable([0,0,0],[0,0,0],np.eye(3),R))
  self.assertTrue(pose_stable([0,0,0],[0,0,0],np.eye(3),np.eye(3)))
 def test_fault_erases_all_stages(self):self.assertEqual(self.finish(failed=True),(0.,0))
 def test_tilted_cart_cannot_finish(self):self.assertEqual(self.finish(angle=.1),(.5,0))
 def test_proximity_and_zero_normal_impulse_are_not_support(self):
  self.assertFalse(load_bearing_contact(.01,[0,0,1],[0,0,1]))
  self.assertFalse(load_bearing_contact(0,[1,0,0],[0,0,1]))
  self.assertFalse(load_bearing_contact(0,[0,0,0],[0,0,1]))
  self.assertTrue(load_bearing_contact(0,[0,0,.01],[0,0,1]))
  self.assertTrue(load_bearing_contact(0,[0,0,-.01],[0,0,1]))
 def test_unloaded_spin_and_airborne_travel_get_no_credit(self):
  names=('wheel_left','wheel_right')
  d,t=supported_motion(dict.fromkeys(names,20),dict.fromkeys(names,.15),dict.fromkeys(names,10.))
  self.assertEqual(sum(d.values())+sum(t.values()),0.)
  self.assertFalse(rolling_complete(d,dict.fromkeys(names,.15),t,.0635))
  gd,gt=supported_motion(dict.fromkeys(names,0),dict.fromkeys(names,.15),dict.fromkeys(names,.15/.0635))
  self.assertTrue(rolling_complete(gd,dict.fromkeys(names,.15),gt,.0635))
 def test_one_lifted_wheel_stops_both_rolling_counters(self):
  names=('wheel_left','wheel_right')
  for ages in ({'wheel_left':5,'wheel_right':0},{'wheel_left':0,'wheel_right':5}):
   d,t=supported_motion(ages,dict.fromkeys(names,.15),dict.fromkeys(names,3.))
   self.assertEqual(sum(d.values())+sum(t.values()),0.)
 def test_reverse_rocking_cannot_make_net_progress(self):
  self.assertFalse(self.rolling(s=-.15,d=-.15,t=-.15/.0635))
  self.assertFalse(self.rolling(s=0,d=0,t=0))
 def test_parked_nose_plate_support_is_not_airborne(self):
  ages={'wheel_left':121,'wheel_right':121}
  self.assertFalse(support_lost(ages,0,True))
  self.assertTrue(support_lost(ages,5,True))
  self.assertTrue(support_lost(ages,0,False))
  self.assertFalse(support_lost({'wheel_left':0,'wheel_right':121},1000,False))
if __name__=='__main__':unittest.main()
