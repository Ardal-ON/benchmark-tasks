import unittest
from nutcracker_task.scoring import Rubric

class ObservableRubricTests(unittest.TestCase):
    def setUp(self):
        self.r=Rubric()
        self.sample=dict(opening_m=0.,opposing_jaw_contact=False,
            robot_shell_contact=False,seated=False,fixture_stable=True,
            contained=True,clear=True)
    def frame(self, **changes):return self.r.update(**(self.sample|changes))
    def witness(self):self.frame(opposing_jaw_contact=True,compression=True,clear=False)
    def test_intact_seat_needs_two_seconds(self):
        for _ in range(39):self.assertEqual(self.frame(seated=True),0.)
        self.assertEqual(self.frame(seated=True),.25)
    def test_opening_after_jaw_compression_survives_release(self):
        self.witness();self.frame(opening_m=.003,clear=False)
        for _ in range(39):self.assertEqual(self.frame(opening_m=.003),0.)
        self.assertEqual(self.frame(opening_m=.003),1.)
    def test_documented_crack_can_close(self):
        self.witness();self.frame(opening_m=.004,clear=False)
        for _ in range(40):score=self.frame()
        self.assertEqual(score,1.)
    def test_no_native_break_event_needed(self):
        self.witness()
        for _ in range(40):score=self.frame(opening_m=.004)
        self.assertEqual(score,1.)
    def test_loading_robot_crush_and_later_jaws_fail(self):
        self.frame(opening_m=.004,robot_shell_contact=True)
        self.frame()
        for _ in range(50):score=self.frame(opening_m=.004,compression=True,opposing_jaw_contact=True)
        self.assertEqual(score,0.)
    def test_gripper_pry_cannot_later_requalify_full_width(self):
        self.witness();self.frame(opening_m=.0006)
        self.frame(opening_m=.004,robot_shell_contact=True)
        for _ in range(50):score=self.frame(opening_m=.004,compression=True,opposing_jaw_contact=True)
        self.assertEqual(score,0.)
    def test_precracked_before_witness_fails(self):
        self.frame(opening_m=.0006);self.witness()
        for _ in range(50):score=self.frame(opening_m=.004)
        self.assertEqual(score,0.)
    def test_subthreshold_not_full(self):
        self.witness()
        for _ in range(50):score=self.frame(opening_m=.0029)
        self.assertEqual(score,0.)
    def test_invalid_final_states_reset_hold(self):
        for condition in ('fixture_stable','contained','clear'):
            self.r=Rubric();self.witness()
            for _ in range(39):self.frame(opening_m=.004)
            self.assertEqual(self.frame(opening_m=.004,**{condition:False}),0.)
            self.assertEqual(self.frame(opening_m=.004),0.)
    def test_hairline_or_reclosed_crack_not_intact(self):
        self.frame(opening_m=.0006)
        for _ in range(50):score=self.frame(seated=True)
        self.assertEqual(score,0.)
    def test_delayed_reveal_allowed(self):
        self.witness()
        for _ in range(100):self.frame()
        for _ in range(40):score=self.frame(opening_m=.004)
        self.assertEqual(score,1.)
    def test_touch_without_closure_not_witness(self):
        self.frame(opposing_jaw_contact=True)
        for _ in range(40):score=self.frame(opening_m=.004)
        self.assertEqual(score,0.)
    def test_exit_reentry_or_foreign_object_poison_witness(self):
        for bad in ('mouth_custody','foreign_contact'):
            self.r=Rubric();self.witness();self.frame(**{bad:bad!='mouth_custody'})
            for _ in range(40):score=self.frame(opening_m=.004,compression=True,opposing_jaw_contact=True)
            self.assertEqual(score,0.)
    def test_physics_rate_transient_exit_cannot_hide_between_control_frames(self):
        self.witness()
        self.r.observe(opening_m=0.,opposing_jaw_contact=False,robot_shell_contact=False,mouth_custody=False)
        self.r.observe(opening_m=0.,opposing_jaw_contact=False,robot_shell_contact=False,mouth_custody=True)
        self.r.observe(opening_m=.004,opposing_jaw_contact=True,robot_shell_contact=False,compression=True)
        for _ in range(40):score=self.frame(opening_m=.004,observe_evidence=False)
        self.assertEqual(score,0.)
    def test_same_sample_compression_cannot_retroactively_qualify_opening(self):
        for _ in range(40):score=self.frame(opening_m=.004,compression=True,opposing_jaw_contact=True)
        self.assertEqual(score,0.)
    def test_unseated_intact_not_partial(self):
        for _ in range(50):score=self.frame()
        self.assertEqual(score,0.)
if __name__=='__main__':unittest.main()
