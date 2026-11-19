"""Run the indoor MINCURV MPCC and optionally publish speed setpoints."""
from collections import Counter
from contextlib import nullcontext
import csv
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import yaml

from navigation.localization3d.soft_runtime import SoftController
from system.f1tenth.drive_selector import DriveSelector
from system.f1tenth.workspace import find_workspace
from .controller import default_config, state_bounds
from .ros_input import check_time, stamp_ns
from .simulation import assess_pose, input_increment_violation
from .slash_model import load_parameters
from .track import Track


def speed_setpoint(prediction, dt, steering, limit):
    """Use the state predicted one LiDAR cycle ahead as a velocity reference."""
    times = np.arange(len(prediction)) * dt
    speed = np.interp(.10, times, prediction[:, 3])
    return (float(np.clip(speed, 0., limit)),
            float(np.clip(steering, -DriveSelector.MAX_AUTO_STEER_RAD, DriveSelector.MAX_AUTO_STEER_RAD)))


class ShadowCsv:
    def __init__(self, stream):
        self.writer = csv.writer(stream)
        self.writer.writerow([
            "stamp_ns", "received_ns", "x_m", "y_m", "yaw_rad", "vx_mps",
            "vy_mps", "yaw_rate_rad_s", "s_m", "accepted", "reason",
            "solver_status", "solve_cycle_ms", "drive_normalized",
            "steering_rad", "progress_speed_mps",
        ])

    def write(self, message, now, *, diagnostics=None, control=None, reason=""):
        result = ("", "", "", "", "") if diagnostics is None else (
            diagnostics["status"], 1000 * diagnostics["cycle_time_s"], *control,
        )
        self.writer.writerow([
            stamp_ns(message.header), now, *message.state,
            int(diagnostics is not None), reason, *result,
        ])


class Shadow:
    """Previous input is internal shadow history, initially zero at each restart."""

    def __init__(self, track_csv, build_dir, speed_limit=0.):
        track_csv = Path(track_csv)
        validation = json.loads(track_csv.with_name("validation.json").read_text())
        if not validation["geometry_valid"] or "vehicle-centre offsets" not in validation.get("boundary_semantics", ""):
            raise ValueError("Expected a validated indoor track with vehicle-centre offset widths")
        config = default_config()
        selected = find_workspace(__file__) / "src/aac_2026/system/f1tenth/config/selected_run.yaml"
        updates = yaml.safe_load(selected.read_text())
        config["weights"].update(updates.pop("weights", {}))
        config.update(updates)
        config.update(model_parameters=load_parameters(), vehicle_width=0., safety_scaling=1.,
                      soft_profile=str(track_csv.with_name("soft_profile.npz")), runtime_backend="python")
        if speed_limit:
            config["speed_max"] = min(2.3, speed_limit + .1)
            config["model_parameters"]["steering_limit"] = min(
                config["model_parameters"]["steering_limit"], DriveSelector.MAX_AUTO_STEER_RAD)
        self.track = Track.from_csv(track_csv)
        self.controller = SoftController(self.track, config, build_dir)
        self.lower, self.upper = state_bounds(self.track, config)
        self.tolerance = config["constraint_tolerance"]
        self.stamp = self.clock = None
        self.reset()

    def reset(self):
        if self.controller.prediction is not None:
            self.controller.solver.reset()
        self.controller.prediction = self.controller.increments = None
        self.previous = np.zeros(3)
        self.last_state = None

    def update(self, message, now, map_frame):
        try:
            stamp = stamp_ns(message.header)
            if self.clock is not None and now < self.clock:
                self.reset()
                self.stamp = None
            self.clock = now
            check_time(stamp, now)
            if self.stamp is not None and stamp <= self.stamp:
                raise ValueError("Duplicate or out-of-order feedback")
            if self.stamp is not None and stamp - self.stamp > 500_000_000:
                self.reset()
            self.stamp = stamp
            if message.header.frame_id != map_frame:
                raise ValueError(f"Expected solver state in {map_frame}")
            state = np.array(message.state, dtype=float)
            if state.shape != (7,) or not np.isfinite(state).all():
                raise ValueError("Expected seven finite states")
            # The OCP bounds progress to a single lap and yaw to a bounded interval.
            state[6] %= self.track.length
            state[2] = math.remainder(state[2], math.tau)
            if self.last_state is not None and (
                abs(state[6] - self.last_state[6]) > self.track.length / 2
                or abs(state[2] - self.last_state[2]) > math.pi
            ):
                self.reset()
            if np.any(state < self.lower[:7]) or np.any(state > self.upper[:7]):
                raise ValueError("Measurement outside Slash model state bounds")
            progress, _, violation = assess_pose(self.track, state, state[6], 0.)
            if violation > self.tolerance or abs(progress - state[6]) > .05:
                raise ValueError("Measurement outside the track corridor or inconsistent progress")
            control, diagnostics = self.controller.solve(state, self.previous)
            status = diagnostics["status"]
            if status or not np.isfinite(control).all() or not np.isfinite(diagnostics["residuals"]).all():
                raise ValueError(f"acados status={status}, residuals={diagnostics['residuals']}")
            if (np.any(control < self.lower[7:10] - self.tolerance)
                    or np.any(control > self.upper[7:10] + self.tolerance)
                    or input_increment_violation(control, self.previous, self.controller.config) > self.tolerance):
                raise ValueError("Solver input constraints exceeded")
            prediction = self.controller.prediction[1:]
            if (not np.isfinite(prediction).all() or np.any(prediction < self.lower - self.tolerance)
                    or np.any(prediction > self.upper + self.tolerance)):
                raise ValueError("Prediction outside model state bounds")
            ref = self.track.reference(prediction[:, 6])
            delta = prediction[:, :2] - ref["center"]
            offset = ref["tangent"][:, 0] * delta[:, 1] - ref["tangent"][:, 1] * delta[:, 0]
            if (np.any(offset < -ref["width_right"] - self.tolerance)
                    or np.any(offset > ref["width_left"] + self.tolerance)):
                raise ValueError("Prediction outside track corridor")
            self.previous = control.copy()
            self.last_state = state
            return diagnostics
        except (RuntimeError, ValueError):
            self.reset()
            raise


def main():
    import rclpy
    from aac_msgs.msg import MpccStateStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    rclpy.init()
    node = Node("mpcc_shadow")
    received = accepted = 0
    rejections = Counter()
    try:
        path = node.declare_parameter("track_csv", "").value
        if not path:
            raise ValueError("Set track_csv to the indoor MINCURV track used by mpcc_input")
        frame = node.declare_parameter("map_frame", "indoor_map").value
        log_path = node.declare_parameter("log_csv", "").value
        speed_limit = node.declare_parameter("drive_speed_limit", 0.).value
        publisher = None
        if speed_limit != 0.:
            if not math.isfinite(speed_limit) or not 0. < speed_limit <= 2.3:
                raise ValueError("drive_speed_limit must be in (0, 2.3] m/s")
            from ackermann_msgs.msg import AckermannDriveStamped
            publisher = node.create_publisher(AckermannDriveStamped, "/mpcc/drive_request", 1)

        def publish_request(header, speed=0., steering=0.):
            if publisher is not None:
                message = AckermannDriveStamped()
                message.header.stamp = header.stamp
                message.header.frame_id = "base_link"
                message.drive.speed = speed
                message.drive.steering_angle = steering
                publisher.publish(message)

        output = open(log_path, "x", newline="", buffering=1) if log_path else nullcontext()
        with output as stream, TemporaryDirectory(prefix="aac-mpcc-shadow-") as directory:
            results = ShadowCsv(stream) if stream is not None else None
            solver = Shadow(path, directory, speed_limit)

            def receive(message):
                nonlocal received, accepted
                received += 1
                now = node.get_clock().now().nanoseconds
                try:
                    stats = solver.update(message, now, frame)
                    check_time(stamp_ns(message.header), node.get_clock().now().nanoseconds)
                except (RuntimeError, ValueError) as exc:
                    solver.reset()
                    publish_request(message.header)
                    rejections[str(exc).split(", residuals=")[0]] += 1
                    if results is not None:
                        results.write(message, now, reason=str(exc))
                    node.get_logger().warning(str(exc), throttle_duration_sec=2.)
                    return
                accepted += 1
                if publisher is not None:
                    publish_request(message.header, *speed_setpoint(
                        solver.controller.prediction, solver.controller.config["dt"],
                        solver.previous[1], speed_limit))
                if results is not None:
                    results.write(message, now, diagnostics=stats, control=solver.previous)
                node.get_logger().info(f"stamp_ns={solver.stamp} acados status={stats['status']} "
                                       f"cycle_ms={1000 * stats['cycle_time_s']:.1f}")

            node.create_subscription(MpccStateStamped, "/mpcc/state", receive, 1)
            node.get_logger().info(f"Shadow solver ready for /mpcc/state on {path}")
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print("Shadow summary " + json.dumps(dict(received=received, accepted=accepted,
                                                rejected_by_reason=dict(rejections))), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
