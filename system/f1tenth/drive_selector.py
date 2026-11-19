"""Physical-unit command selection shared by the AGX gate and offline tests."""
from dataclasses import dataclass, fields
from math import isfinite


@dataclass(frozen=True)
class OperatorInput:
    arm: bool = False
    stop: bool = False
    manual: bool = False
    auto: bool = False


@dataclass(frozen=True)
class DriveOutput:
    mode: str
    erpm: float
    servo: float


class HardwareLimits:
    def __init__(self, config):
        for name in (
            'wheel_erpm_per_mps', 'auto_speed_mps', 'auto_erpm_limit',
            'auto_servo_gain', 'auto_servo_center', 'controller_id',
            'manual_erpm_gain', 'manual_erpm_offset', 'manual_erpm_limit',
            'manual_servo_gain', 'manual_servo_center', 'servo_min', 'servo_max',
        ):
            value = config[name]
            if type(value) not in (int, float) or not isfinite(value):
                raise ValueError(f'Invalid drive calibration {name}')
            setattr(self, name, value)
        if not (-5000 <= self.wheel_erpm_per_mps < 0
                and 0 < self.auto_speed_mps <= 2.3
                and 0 < self.auto_erpm_limit <= 8700
                and self.auto_erpm_limit <= self.manual_erpm_limit <= 10000
                and 0 <= self.servo_min < self.servo_max <= 1
                and self.servo_min < self.auto_servo_center < self.servo_max
                and self.servo_min < self.manual_servo_center < self.servo_max
                and self.auto_servo_gain != 0 and self.manual_servo_gain != 0
                and self.controller_id == int(self.controller_id)):
            raise ValueError('Drive calibration falls outside the indoor operating envelope')


def clamp(value, low, high):
    return max(low, min(high, value))


class DriveSelector:
    """PS5 selection and speed watchdog, independent of the MPC process."""

    JOY_TIMEOUT = .30
    FEEDBACK_TIMEOUT = .25
    COMMAND_TIMEOUT = .25
    MAX_AUTO_STEER_RAD = .30

    def __init__(self, limits):
        self.limits = limits
        self.joy = self.wheel = self.manual_command = self.auto_command = None
        self.armed = self.saw_neutral = self.auto_blocked = False

    def disarm(self):
        self.armed = self.saw_neutral = False
        self.auto_blocked = True

    @staticmethod
    def fresh(item, now, limit):
        return item is not None and 0 <= now - item[0] <= limit

    def operator(self, controls, now):
        previous = self.joy[1] if self.fresh(self.joy, now, self.JOY_TIMEOUT) else None
        if previous is None or controls is None or controls.stop:
            self.disarm()
        self.joy = (now, controls) if controls is not None else None
        if controls is None or controls.stop:
            return
        if not (controls.arm or controls.auto or controls.manual):
            self.saw_neutral = True
        if (self.saw_neutral and previous is not None and controls.arm
                and not previous.arm and not (controls.auto or controls.manual)):
            self.armed = True
        if not controls.auto:
            self.auto_blocked = False
        elif controls.manual:
            self.auto_blocked = True

    def feedback(self, erpm, controller_id, fault_code, now):
        self.wheel = ((now, erpm) if isfinite(erpm)
                      and controller_id == self.limits.controller_id and fault_code == 0 else None)
        if self.wheel is None:
            self.disarm()

    def manual(self, speed, steering, now):
        self.manual_command = (now, speed, steering) if isfinite(speed) and isfinite(steering) else None

    def autonomy(self, speed, steering, now):
        self.auto_command = ((now, speed, steering)
                             if isfinite(speed) and isfinite(steering)
                             and 0 < speed <= self.limits.auto_speed_mps + 1e-6
                             and abs(steering) <= self.MAX_AUTO_STEER_RAD + 1e-6 else None)
        if self.auto_command is None and self.joy is not None and self.joy[1].auto:
            self.auto_blocked = True

    def output(self, now):
        limits = self.limits
        stop = DriveOutput('stop', 0., limits.manual_servo_center)
        if (not self.fresh(self.joy, now, self.JOY_TIMEOUT)
                or not self.fresh(self.wheel, now, self.FEEDBACK_TIMEOUT)):
            self.disarm()
            return stop
        _, operator = self.joy
        if not self.armed:
            return stop
        if operator.manual and self.fresh(self.manual_command, now, self.COMMAND_TIMEOUT):
            _, speed, angle = self.manual_command
            target = speed*limits.manual_erpm_gain + limits.manual_erpm_offset if speed else 0.
            servo = limits.manual_servo_center + limits.manual_servo_gain*angle
            return DriveOutput('manual', clamp(target, -limits.auto_erpm_limit, limits.auto_erpm_limit),
                               clamp(servo, limits.servo_min, limits.servo_max))
        if not operator.auto or operator.manual or self.auto_blocked:
            return stop
        if (not self.fresh(self.auto_command, now, self.COMMAND_TIMEOUT)
                or self.wheel[1] < -limits.auto_erpm_limit
                or self.wheel[1] > 200.):
            self.auto_blocked = True
            return stop
        _, speed, angle = self.auto_command
        target = max(speed*limits.wheel_erpm_per_mps, -limits.auto_erpm_limit)
        servo = limits.auto_servo_center + limits.auto_servo_gain*angle
        return DriveOutput('auto', target, clamp(servo, limits.servo_min, limits.servo_max))
