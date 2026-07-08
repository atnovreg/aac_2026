#!/usr/bin/env python3
"""Print live MPPI cornering diagnostics from /mppi/debug_status."""

import argparse
import json
import math
import statistics
import sys
import time
from collections import deque

import rclpy
from std_msgs.msg import String


def as_float(data, name, default=math.nan):
    try:
        return float(data.get(name, default))
    except (TypeError, ValueError):
        return default


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes", "on")
    return bool(value)


def mean(values, default=math.nan):
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return default
    return statistics.fmean(clean)


def percentile(values, q, default=math.nan):
    clean = sorted(value for value in values if math.isfinite(value))
    if not clean:
        return default
    if len(clean) == 1:
        return clean[0]
    position = (len(clean) - 1) * q
    low = int(math.floor(position))
    high = int(math.ceil(position))
    if low == high:
        return clean[low]
    fraction = position - low
    return clean[low] * (1.0 - fraction) + clean[high] * fraction


class MppiWatcher:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.samples = deque()
        self.last_print_time = 0.0
        self.parse_errors = 0
        self.subscription = node.create_subscription(
            String, args.debug_topic, self.on_debug_status, 10)

    def on_debug_status(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.parse_errors += 1
            return

        now = time.monotonic()
        self.samples.append((now, data))
        self.prune(now)

    def prune(self, now):
        cutoff = now - self.args.window
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def maybe_print(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_print_time < self.args.period:
            return

        self.last_print_time = now
        self.prune(now)
        print(self.summary(), flush=True)

    def summary(self):
        if not self.samples:
            return (
                f"waiting for {self.args.debug_topic} "
                f"(parse_errors={self.parse_errors})")

        data_samples = [data for _, data in self.samples]
        latest = data_samples[-1]
        corner_samples = [
            data for data in data_samples
            if abs(as_float(data, "curvature_heading_change_deg", 0.0)) >=
            self.args.corner_threshold_deg
        ]

        if not corner_samples:
            speed = as_float(latest, "speed", 0.0)
            rollouts = int(as_float(latest, "rollouts", 0.0))
            profile = latest.get("tuning_profile", "unknown")
            ff_gain = as_float(latest, "steering_feedforward_command_gain", 0.0)
            speed_floor = as_float(latest, "tracking_min_command_speed", 0.0)
            return (
                f"samples={len(data_samples)} no_corner "
                f"profile={profile} ff_gain={ff_gain:.2f} floor={speed_floor:.2f} "
                f"speed={speed:.2f} rollouts={rollouts} "
                f"hint=wait_for_corner")

        corner_angles = [
            abs(as_float(data, "curvature_heading_change_deg", 0.0))
            for data in corner_samples
        ]
        speeds = [as_float(data, "speed") for data in corner_samples]
        corner_targets = [
            as_float(
                data,
                "effective_speed_target",
                as_float(data, "corner_speed_target"))
            for data in corner_samples
        ]
        lateral_targets = [
            as_float(data, "lateral_accel_speed_target") for data in corner_samples
        ]
        lateral_limits = [
            as_float(data, "lateral_accel_limit") for data in corner_samples
        ]
        lateral_accels = [
            as_float(data, "command_lateral_accel_estimate") for data in corner_samples
        ]
        steering_targets_abs = [
            abs(as_float(data, "steering_target_deg")) for data in corner_samples
        ]
        steering_errors_abs = [
            abs(as_float(data, "steering_error_deg")) for data in corner_samples
        ]
        selected_best_gaps = [
            as_float(data, "selected_minus_best_sampled_cost") for data in corner_samples
        ]
        steering_response_ratios = []
        for data in corner_samples:
            target = abs(as_float(data, "steering_target_deg"))
            steering = abs(as_float(data, "steering_deg"))
            if (
                math.isfinite(target) and
                target >= self.args.steering_threshold_deg and
                math.isfinite(steering)
            ):
                steering_response_ratios.append(min(2.0, steering / target))
        speeds_before_limit = [
            as_float(data, "command_speed_before_limit") for data in corner_samples
        ]
        feature_active_ratio = sum(
            1 for data in corner_samples
            if as_bool(data.get("use_corner_speed_as_speed_target", False)) and
            as_bool(data.get("use_lateral_accel_speed_limit", False)) and
            as_bool(data.get("limit_command_speed_to_corner", False))) / len(corner_samples)
        limited_ratio = sum(
            1 for data in corner_samples
            if as_bool(data.get("command_speed_limited", False))) / len(corner_samples)
        floor_applied_ratio = sum(
            1 for data in corner_samples
            if as_bool(data.get("command_speed_floor_applied", False))) / len(corner_samples)
        tracking_speed_floors = [
            as_float(data, "tracking_speed_floor") for data in corner_samples
        ]

        max_corner = max(corner_angles)
        avg_speed = mean(speeds, 0.0)
        finite_targets = [value for value in corner_targets if math.isfinite(value)]
        min_target = min(finite_targets) if finite_targets else avg_speed
        avg_before_limit = mean(speeds_before_limit, avg_speed)
        avg_tracking_floor = mean(tracking_speed_floors, 0.0)
        avg_lateral_target = mean(lateral_targets, min_target)
        avg_lateral_limit = mean(lateral_limits, math.nan)
        lateral_accel_p95 = percentile(lateral_accels, 0.95, 0.0)
        avg_over_target = avg_speed - min_target
        steering_target_p95 = percentile(steering_targets_abs, 0.95, 0.0)
        steering_error_p95 = percentile(steering_errors_abs, 0.95, 0.0)
        steering_response_avg = mean(steering_response_ratios, 1.0)
        steering_lag_ratio = (
            sum(
                1 for response in steering_response_ratios
                if response < self.args.late_response_threshold) /
            len(steering_response_ratios)
            if steering_response_ratios else 0.0)
        selected_best_gap_p95 = percentile(selected_best_gaps, 0.95, 0.0)
        lateral_active_ratio = sum(
            1 for target, lateral_target in zip(corner_targets, lateral_targets)
            if math.isfinite(target) and
            math.isfinite(lateral_target) and
            lateral_target <= target + 0.03) / len(corner_samples)
        rollouts = int(as_float(latest, "rollouts", 0.0))
        profile = latest.get("tuning_profile", "unknown")
        ff_gain = as_float(latest, "steering_feedforward_command_gain", 0.0)
        hint = self.hint(
            limited_ratio,
            floor_applied_ratio,
            avg_over_target,
            avg_before_limit,
            avg_speed,
            min_target,
            max_corner,
            lateral_active_ratio,
            feature_active_ratio,
            steering_target_p95,
            steering_error_p95,
            steering_response_avg,
            steering_lag_ratio,
            selected_best_gap_p95)

        summary = (
            f"samples={len(data_samples)} corner_samples={len(corner_samples)} "
            f"profile={profile} ff_gain={ff_gain:.2f} "
            f"max_corner={max_corner:.1f}deg "
            f"cost_target_min={min_target:.2f}mps "
            f"floor_avg={avg_tracking_floor:.2f}mps "
            f"lat_target_avg={avg_lateral_target:.2f}mps "
            f"lat_a_p95={lateral_accel_p95:.2f}mps2 "
            f"lat_active={lateral_active_ratio * 100.0:.0f}% "
            f"steer_target_p95={steering_target_p95:.1f}deg "
            f"steer_err_p95={steering_error_p95:.1f}deg "
            f"steer_resp={steering_response_avg * 100.0:.0f}% "
            f"steer_lag={steering_lag_ratio * 100.0:.0f}% "
            f"opt_gap_p95={selected_best_gap_p95:.2f} "
            f"features={feature_active_ratio * 100.0:.0f}% "
            f"speed_avg={avg_speed:.2f}mps "
            f"before_limit_avg={avg_before_limit:.2f}mps "
            f"limited={limited_ratio * 100.0:.0f}% "
            f"floor={floor_applied_ratio * 100.0:.0f}% "
            f"rollouts={rollouts} hint={hint}")
        if self.args.commands:
            command = self.command_for_hint(hint, avg_lateral_limit)
            if command:
                summary += f"\n  command: {command}"
        return summary

    def hint(
            self,
            limited_ratio,
            floor_applied_ratio,
            avg_over_target,
            avg_before_limit,
            avg_speed,
            min_target,
            max_corner,
            lateral_active_ratio,
            feature_active_ratio,
            steering_target_p95,
            steering_error_p95,
            steering_response_avg,
            steering_lag_ratio,
            selected_best_gap_p95):
        if feature_active_ratio < 0.5:
            return "corner_features_disabled_restart_or_apply_early"
        if selected_best_gap_p95 > self.args.optimizer_gap_threshold:
            return "optimizer_gap_increase_iterations_or_lower_temperature"
        if avg_speed < 0.10 and min_target > 0.15 and floor_applied_ratio < 0.2:
            return "tracking_floor_disabled_or_restart"
        if (
            avg_over_target > -0.20 and
            steering_target_p95 >= self.args.steering_threshold_deg and
            steering_error_p95 >= self.args.steering_threshold_deg and
            (steering_response_avg < 0.70 or steering_lag_ratio >= 0.45)
        ):
            return "steering_lag_try_aggressive_or_raise_steering_rate"
        if limited_ratio < 0.2 and avg_before_limit > min_target + 0.20:
            return "try_aggressive_or_lower_corner_speed_margin"
        if limited_ratio > 0.6 and avg_over_target > 0.18:
            return "limit_active_raise_max_accel_or_lower_corner_speed_margin"
        if max_corner > 25.0 and avg_over_target > 0.16 and lateral_active_ratio < 0.4:
            return "lower_lateral_accel_limit_or_try_slow_corner"
        if max_corner > 25.0 and avg_over_target > 0.12:
            return "try_slow_corner_for_tighter_turns"
        if limited_ratio > 0.8 and avg_over_target < -0.05:
            return "too_slow_raise_limits_or_use_stable"
        return "looks_reasonable_verify_vehicle_response"

    def command_for_hint(self, hint, lateral_accel_limit=math.nan):
        node = self.args.node
        if hint == "corner_features_disabled_restart_or_apply_early":
            return f"ros2 run mppi_planner_node mppi_tune.py early --node {node} --verify"
        if hint == "optimizer_gap_increase_iterations_or_lower_temperature":
            return (
                f"ros2 param set {node} iteration_count 3 && "
                f"ros2 param set {node} temperature 0.35")
        if hint == "tracking_floor_disabled_or_restart":
            return (
                f"ros2 param set {node} tracking_min_command_speed 0.25 && "
                f"ros2 run mppi_planner_node mppi_runtime_check.py "
                f"--node {node} --expect-profile early "
                f"--min-steering-feedforward-command-gain 0.10")
        if hint == "steering_lag_try_aggressive_or_raise_steering_rate":
            return (
                f"ros2 run mppi_planner_node mppi_tune.py aggressive --node {node} --verify && "
                f"ros2 param set {node} max_steering_rate 5.0 && "
                f"ros2 param set {node} steering_feedforward_command_gain 0.65")
        if hint == "try_aggressive_or_lower_corner_speed_margin":
            return f"ros2 run mppi_planner_node mppi_tune.py aggressive --node {node} --verify"
        if hint == "limit_active_raise_max_accel_or_lower_corner_speed_margin":
            return (
                f"ros2 param set {node} max_accel 2.5 && "
                f"ros2 param set {node} corner_speed_margin 0.02")
        if hint == "lower_lateral_accel_limit_or_try_slow_corner":
            next_limit = 0.35
            if math.isfinite(lateral_accel_limit):
                next_limit = max(0.25, lateral_accel_limit * 0.8)
            return (
                f"ros2 run mppi_planner_node mppi_tune.py slow_corner --node {node} --verify && "
                f"ros2 param set {node} lateral_accel_limit {next_limit:.2f}")
        if hint == "try_slow_corner_for_tighter_turns":
            return f"ros2 run mppi_planner_node mppi_tune.py slow_corner --node {node} --verify"
        if hint == "too_slow_raise_limits_or_use_stable":
            next_limit = 0.55
            if math.isfinite(lateral_accel_limit):
                next_limit = min(0.80, lateral_accel_limit * 1.2)
            return (
                f"ros2 run mppi_planner_node mppi_tune.py stable --node {node} --verify && "
                f"ros2 param set {node} lateral_accel_limit {next_limit:.2f} && "
                f"ros2 param set {node} corner_speed_margin 0.10")
        return ""


def main():
    parser = argparse.ArgumentParser(
        description="Watch MPPI debug_status and summarize live cornering behavior.")
    parser.add_argument(
        "--debug-topic",
        default="/mppi/debug_status",
        help="Debug status topic. Default: /mppi/debug_status")
    parser.add_argument(
        "--node",
        default="/mppi_planner_node",
        help="Target MPPI node used in recommended commands. Default: /mppi_planner_node")
    parser.add_argument(
        "--commands",
        action="store_true",
        help="Print a recommended ros2 command below each tuning hint.")
    parser.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="Rolling window in seconds. Default: 5.0")
    parser.add_argument(
        "--period",
        type=float,
        default=1.0,
        help="Print period in seconds. Default: 1.0")
    parser.add_argument(
        "--corner-threshold-deg",
        type=float,
        default=12.0,
        help="Minimum curvature heading change to count as a corner. Default: 12.0")
    parser.add_argument(
        "--steering-threshold-deg",
        type=float,
        default=8.0,
        help="Minimum steering target counted as meaningful corner steering. Default: 8.")
    parser.add_argument(
        "--optimizer-gap-threshold",
        type=float,
        default=1.0,
        help="Selected-vs-best-sampled cost gap that suggests more MPPI iterations. Default: 1.0")
    parser.add_argument(
        "--late-response-threshold",
        type=float,
        default=0.75,
        help="Steering response ratio below this is counted as late. Default: 0.75")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Exit after this many seconds. Default: 0 means run until interrupted.")
    args = parser.parse_args()

    rclpy.init(args=None)
    node = rclpy.create_node("mppi_watch")
    watcher = MppiWatcher(node, args)
    start = time.monotonic()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            watcher.maybe_print()
            if args.duration > 0.0 and time.monotonic() - start >= args.duration:
                watcher.maybe_print(force=True)
                break
    except KeyboardInterrupt:
        watcher.maybe_print(force=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
