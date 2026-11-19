"""Connect PS5, MPCC, and VESC with a single owner of final commands."""
from pathlib import Path
import time

import yaml

from .drive_selector import DriveSelector, HardwareLimits, OperatorInput


def load_config(calibration_path, manual_path):
    config = yaml.safe_load(Path(calibration_path).read_text())
    manual = yaml.safe_load(Path(manual_path).read_text())
    mapping = manual['/vesc/ackermann_to_vesc']['ros__parameters']
    driver = manual['/vesc/vesc_driver']['ros__parameters']
    config.update(
        manual_erpm_gain=mapping['speed_to_erpm_gain'],
        manual_erpm_offset=mapping['speed_to_erpm_offset'],
        manual_erpm_limit=min(-driver['speed_min'], driver['speed_max']),
        manual_servo_gain=mapping['steering_angle_to_servo_gain'],
        manual_servo_center=mapping['steering_angle_to_servo_offset'],
        servo_min=driver['servo_min'], servo_max=driver['servo_max'],
    )
    control = manual['/manual_control']['ros__parameters']
    buttons = [control[name] for name in ('arm_button', 'stop_button', 'deadman_button')]
    buttons.append(config['auto_button'])
    if len(set(buttons)) != len(buttons) or any(type(index) is not int or index < 0 for index in buttons):
        raise ValueError('PS5 drive buttons must be distinct nonnegative indices')
    return HardwareLimits(config), buttons


def main():
    from ackermann_msgs.msg import AckermannDriveStamped
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Float64, String
    from vesc_msgs.msg import VescStateStamped

    rclpy.init()
    node = rclpy.create_node('aac_drive_selector')
    try:
        limits, buttons = load_config(
            node.declare_parameter('calibration_file', '').value,
            node.declare_parameter('manual_config', '').value,
        )
        hardware = node.declare_parameter('hardware', False).value
        topics = '/vesc/commands' if hardware else '/drive/preview'
        motor = node.create_publisher(Float64, f'{topics}/motor/speed', 1)
        servo = node.create_publisher(Float64, f'{topics}/servo/position', 1)
        mode = node.create_publisher(String, '/drive/mode', 1)
        selector = DriveSelector(limits)
        previous_mode = None

        def source_time(header, timeout):
            stamp = header.stamp.sec*1_000_000_000 + header.stamp.nanosec
            age = node.get_clock().now().nanoseconds - stamp
            if stamp > 0 and -50_000_000 <= age <= int(timeout*1e9):
                return time.monotonic() - max(0., age/1e9)
            return None

        def joy(msg):
            controls = None
            stamp = source_time(msg.header, selector.JOY_TIMEOUT)
            if len(msg.buttons) > max(buttons) and stamp is not None:
                controls = OperatorInput(*(bool(msg.buttons[index]) for index in buttons))
            selector.operator(controls, stamp if stamp is not None else time.monotonic())

        def wheel(msg):
            stamp = source_time(msg.header, selector.FEEDBACK_TIMEOUT)
            if stamp is not None:
                state = msg.state
                selector.feedback(state.speed, state.controller_id, state.fault_code, stamp)

        def command(msg, receiver):
            stamp = source_time(msg.header, selector.COMMAND_TIMEOUT)
            if msg.header.frame_id == 'base_link' and stamp is not None:
                receiver(msg.drive.speed, msg.drive.steering_angle, stamp)
            else:
                receiver(float('nan'), 0., time.monotonic())

        def publish():
            nonlocal previous_mode
            output = selector.output(time.monotonic())
            motor.publish(Float64(data=output.erpm))
            servo.publish(Float64(data=output.servo))
            mode.publish(String(data=output.mode))
            if output.mode != previous_mode:
                node.get_logger().info(f'Drive mode {output.mode}')
                previous_mode = output.mode

        node.create_subscription(Joy, '/manual/joy', joy, 1)
        node.create_subscription(VescStateStamped, '/vesc/sensors/core', wheel, 1)
        node.create_subscription(AckermannDriveStamped, '/manual/ackermann_drive',
                                 lambda msg: command(msg, selector.manual), 1)
        node.create_subscription(AckermannDriveStamped, '/mpcc/drive_request',
                                 lambda msg: command(msg, selector.autonomy), 1)
        node.create_timer(.02, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
        node.get_logger().info(f'Drive selector ready, output namespace {topics}')
        try:
            rclpy.spin(node)
        finally:
            if rclpy.ok():
                motor.publish(Float64(data=0.))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
