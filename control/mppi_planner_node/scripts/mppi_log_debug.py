#!/usr/bin/env python3
"""Record MPPI debug_status JSON messages to a CSV file."""

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from std_msgs.msg import String


DEFAULT_FIELDS = (
    "stamp",
    "tuning_profile",
    "speed",
    "steering_deg",
    "target_speed",
    "max_speed",
    "max_accel",
    "max_steering_rate",
    "iteration_count",
    "temperature",
    "corner_speed_target",
    "lateral_accel_speed_target",
    "effective_speed_target",
    "tracking_min_command_speed",
    "tracking_speed_floor",
    "command_speed_before_limit",
    "command_speed_limited",
    "command_speed_floor_applied",
    "curvature_heading_change_deg",
    "curvature_heading_change_signed_deg",
    "curvature_lookahead_actual_distance",
    "curvature_estimate",
    "command_lateral_accel_estimate",
    "heading_lookahead_actual_distance",
    "path_progress",
    "path_heading_deg",
    "lookahead_heading_deg",
    "steering_target_deg",
    "steering_error_deg",
    "selected_cost",
    "best_sampled_rollout_cost",
    "selected_minus_best_sampled_cost",
    "best_rollout_cost",
    "worst_rollout_cost",
    "rollouts",
    "trajectory_points",
    "heading_weight",
    "heading_lookahead_weight",
    "heading_lookahead_distance",
    "steering_rate_weight",
    "accel_weight",
    "steering_feedforward_lookahead_distance",
    "steering_feedforward_weight",
    "steering_feedforward_command_gain",
    "curvature_speed_weight",
    "curvature_speed_gain",
    "lateral_accel_limit",
    "use_lateral_accel_speed_limit",
    "use_corner_speed_as_speed_target",
    "limit_command_speed_to_corner",
    "corner_speed_margin",
)


class DebugLogger:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.start_time = time.monotonic()
        self.count = 0
        self.parse_errors = 0
        self.fields = list(DEFAULT_FIELDS)
        self.output_path = self.make_output_path(args.output)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.output_path.open("w", newline="")
        self.writer = csv.DictWriter(self.file, fieldnames=self.fields, extrasaction="ignore")
        self.writer.writeheader()
        self.subscription = node.create_subscription(
            String, args.debug_topic, self.on_debug_status, 10)

    def make_output_path(self, output):
        if output:
            return Path(output).expanduser()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return Path(f"/tmp/mppi_debug_{stamp}.csv")

    def on_debug_status(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.parse_errors += 1
            return

        row = {field: "" for field in self.fields}
        row["stamp"] = f"{time.time():.6f}"
        for field in self.fields:
            if field == "stamp":
                continue
            value = data.get(field, "")
            if isinstance(value, float) and not math.isfinite(value):
                value = ""
            row[field] = value

        self.writer.writerow(row)
        self.count += 1
        if self.args.flush_every > 0 and self.count % self.args.flush_every == 0:
            self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()

    def done(self):
        if self.args.duration <= 0.0:
            return False
        return time.monotonic() - self.start_time >= self.args.duration


def main():
    parser = argparse.ArgumentParser(
        description="Record /mppi/debug_status JSON to CSV for post-drive tuning.")
    parser.add_argument(
        "--debug-topic",
        default="/mppi/debug_status",
        help="Debug status topic. Default: /mppi/debug_status")
    parser.add_argument(
        "-o",
        "--output",
        default="",
        help="CSV output path. Default: /tmp/mppi_debug_<timestamp>.csv")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Stop after this many seconds. Default: 0 means run until interrupted.")
    parser.add_argument(
        "--flush-every",
        type=int,
        default=10,
        help="Flush file after this many rows. Default: 10")
    args = parser.parse_args()

    rclpy.init(args=None)
    node = rclpy.create_node("mppi_log_debug")
    logger = DebugLogger(node, args)
    print(f"logging {args.debug_topic} to {logger.output_path}", flush=True)

    try:
        while rclpy.ok() and not logger.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        logger.close()
        node.destroy_node()
        rclpy.shutdown()

    print(
        f"wrote {logger.count} rows to {logger.output_path} "
        f"(parse_errors={logger.parse_errors})",
        flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
