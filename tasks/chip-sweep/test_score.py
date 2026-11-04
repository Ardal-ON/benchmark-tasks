import unittest
from types import SimpleNamespace
import numpy as np
from chip_sweep.scoring import History,evaluate,collected
TOP=.7475
def state(p,size,still=True):
 p=np.array(p);h=np.array(size)/2
 return SimpleNamespace(pos=p,low=p-h,high=p+h,resting=lambda:still)
def objects():
 o={f'chip_{i}':state((.22+i*.015,0,TOP+.015),(.012,.008,.006)) for i in range(5)}
 o['brush']=state((.22875,.27,TOP+.0555),(.2475,.115,.035));return o
class Scores(unittest.TestCase):
 def test_placement_requires_brush_contact(self):self.assertEqual(evaluate(objects(),History(open_clear=True),TOP),0)
 def test_full_and_partial(self):
  h=History(touched={f'chip_{i}' for i in range(5)},open_clear=True)
  self.assertEqual(evaluate(objects(),h,TOP),1.)
  h.open_clear=False;self.assertEqual(evaluate(objects(),h,TOP),.8)
  h.touched.remove('chip_4');self.assertEqual(evaluate(objects(),h,TOP),.64)
 def test_direct_hand_contact_invalid(self):
  h=History(touched={f'chip_{i}' for i in range(5)},open_clear=True)
  h.contact('/World/envs/env_0/chip_1','/World/envs/env_0/right_robot/finger')
  self.assertEqual(evaluate(objects(),h,TOP),0)
 def test_must_be_fully_inside_and_resting(self):
  self.assertFalse(collected(state((.305,0,TOP+.015),(.012,.008,.006)),TOP))
  self.assertFalse(collected(state((.25,0,TOP+.021),(.012,.008,.006)),TOP))
  self.assertFalse(collected(state((.25,0,TOP+.015),(.012,.008,.006),False),TOP))
 def test_wrong_contact_does_not_count(self):
  h=History();h.contact('/World/brush','/World/table');self.assertFalse(h.touched)
  h.contact('/World/brush','/World/chip_3');self.assertEqual(h.touched,{'chip_3'})
 def test_drop_invalid(self):
  o=objects();o['chip_3']=state((.2,0,TOP-.1),(.012,.008,.006));h=History();h.update(o,TOP);self.assertTrue(h.invalid)
if __name__=='__main__':unittest.main()
