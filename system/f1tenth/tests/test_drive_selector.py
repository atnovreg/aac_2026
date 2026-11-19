import unittest

from system.f1tenth.drive_selector import DriveSelector, HardwareLimits, OperatorInput


class DriveSelectorTest(unittest.TestCase):
    def setUp(self):
        self.gate = DriveSelector(HardwareLimits(hardware_config()))

    def feed(self, t, operator=OperatorInput(), speed=1.0, manual=(1.0, .1)):
        self.gate.operator(operator, t)
        self.gate.feedback(-3800.0, 117, 0, t)
        self.gate.manual(*manual, t)
        self.gate.autonomy(speed, .1, t)

    def arm(self):
        self.feed(1.0)
        self.feed(1.05, OperatorInput(arm=True))
        self.feed(1.1)

    def test_held_inputs_at_start_cannot_arm(self):
        self.feed(1.0, OperatorInput(arm=True, auto=True))
        self.assertEqual(self.gate.output(1.0).erpm, 0.)

    def test_manual_override_is_prioritized_and_auto_requires_new_request(self):
        self.arm()
        self.feed(1.15, OperatorInput(auto=True))
        out = self.gate.output(1.15)
        self.assertEqual(out.mode, 'auto')
        self.assertEqual(out.erpm, -3800.)
        self.assertAlmostEqual(out.servo, .35)
        self.feed(1.20, OperatorInput(manual=True, auto=True))
        out = self.gate.output(1.20)
        self.assertEqual(out.mode, 'manual')
        self.assertEqual(out.erpm, -4614.)
        self.assertAlmostEqual(out.servo, .56)
        self.feed(1.25, OperatorInput(auto=True))
        self.assertEqual(self.gate.output(1.25).erpm, 0.)

    def test_bad_or_stale_auto_output_stops_and_requires_release(self):
        self.arm()
        self.feed(1.15, OperatorInput(auto=True))
        self.assertEqual(self.gate.output(1.15).mode, 'auto')
        self.gate.autonomy(0., 0., 1.16)
        self.assertEqual(self.gate.output(1.16).erpm, 0.)
        self.feed(1.17, OperatorInput(auto=True))
        self.assertEqual(self.gate.output(1.17).erpm, 0.)
        self.feed(1.18)
        self.feed(1.19, OperatorInput(auto=True))
        self.assertEqual(self.gate.output(1.19).mode, 'auto')
        for t in [1.29, 1.39, 1.49]:
            self.gate.operator(OperatorInput(auto=True), t)
            self.gate.feedback(-3800., 117, 0, t)
        self.assertEqual(self.gate.output(1.49).erpm, 0.)

    def test_motor_caps_faults_and_operator_stop(self):
        self.arm()
        self.feed(1.15, OperatorInput(auto=True), speed=3.)
        self.assertEqual(self.gate.output(1.15).erpm, 0.)
        self.feed(1.20)
        self.feed(1.25, OperatorInput(auto=True), speed=2.2)
        self.assertEqual(self.gate.output(1.25).erpm, -8300.)
        self.gate.feedback(-8600., 117, 0, 1.26)
        self.assertEqual(self.gate.output(1.26).erpm, 0.)
        self.feed(1.27, OperatorInput(manual=True))
        self.gate.feedback(-1000., 117, 1, 1.27)
        self.assertEqual(self.gate.output(1.27).erpm, 0.)
        self.feed(1.28, OperatorInput(stop=True, manual=True, auto=True))
        self.assertEqual(self.gate.output(1.28).erpm, 0.)


def hardware_config():
    return {
        'wheel_erpm_per_mps': -3800., 'auto_speed_mps': 2.2, 'auto_erpm_limit': 8300.,
        'auto_servo_gain': -1., 'auto_servo_center': .45, 'controller_id': 117,
        'manual_erpm_gain': -4614., 'manual_erpm_offset': 0., 'manual_erpm_limit': 10000.,
        'manual_servo_gain': 1., 'manual_servo_center': .46, 'servo_min': .2, 'servo_max': .8,
    }
