#!/usr/bin/env python3
"""Run an isolated MPPI cornering probe without touching vehicle command topics."""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from mppi_tune import PROFILES, apply_profile_atomically, profile_with_name


def installed_prefix():
    script_path = Path(__file__).resolve()
    if len(script_path.parents) >= 3:
        return script_path.parents[2]
    return None


def default_executable():
    prefix = installed_prefix()
    if prefix is None:
        return ""
    return str(prefix / "lib" / "mppi_planner_node" / "mppi_planner_exe")


def default_config():
    prefix = installed_prefix()
    if prefix is None:
        return ""
    return str(prefix / "share" / "mppi_planner_node" / "config" / "mppi_planner.yaml")


def make_probe_config(source_config, node_name):
    if not source_config:
        return None
    source_path = Path(source_config)
    if not source_path.exists():
        return None

    text = source_path.read_text()
    text = text.replace("mppi_planner_node:", f"{node_name}:", 1)
    temp = tempfile.NamedTemporaryFile(
        mode="w", prefix="mppi_corner_probe_", suffix=".yaml", delete=False)
    temp.write(text)
    temp.close()
    return temp.name


def start_planner(args, node_name, topic_prefix, config_path):
    command = [
        args.executable,
        "--ros-args",
    ]
    if config_path:
        command.extend(["--params-file", config_path])
    command.extend([
        "-r", f"__node:={node_name}",
        "-p", f"local_path_topic:={topic_prefix}/local_path",
        "-p", f"command_topic:={topic_prefix}/ackermann_auto",
        "-p", f"trajectory_topic:={topic_prefix}/optimal_trajectory",
        "-p", f"marker_topic:={topic_prefix}/trajectory_markers",
        "-p", f"debug_topic:={topic_prefix}/debug_status",
        "-p", f"max_visualized_rollouts:={args.max_visualized_rollouts}",
    ])
    for override in args.set_param:
        command.extend(["-p", override])

    return subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True)


def apply_profile(args, node_name):
    if args.profile == "config":
        return 0
    print(f"Applying profile '{args.profile}' to /{node_name}", flush=True)
    return apply_profile_atomically(
        f"/{node_name}", profile_with_name(args.profile), args.profile_timeout)


def make_path(node, args):
    path = PathMsg()
    path.header.frame_id = "base_footprint"
    path.header.stamp = node.get_clock().now().to_msg()
    heading = math.radians(args.heading_deg)
    cos_heading = math.cos(heading)
    sin_heading = math.sin(heading)
    for index in range(args.points):
        base_x = args.spacing * index
        base_y = args.curvature * base_x * base_x
        x = cos_heading * base_x - sin_heading * base_y
        y = sin_heading * base_x + cos_heading * base_y
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def has_valid_probe_output(received):
    trajectory = received.get("trajectory")
    markers = received.get("markers")
    debug_msg = received.get("debug")
    cmd = received.get("cmd")

    if cmd is None or trajectory is None or markers is None or debug_msg is None:
        return False
    if not trajectory.poses:
        return False

    namespaces = Counter(marker.ns for marker in markers.markers)
    if namespaces.get("mppi_rollouts", 0) <= 0:
        return False
    if namespaces.get("mppi_best_sampled", 0) <= 0:
        return False
    if namespaces.get("mppi_optimal", 0) <= 0:
        return False
    if namespaces.get("mppi_diagnostics", 0) <= 0:
        return False

    try:
        debug = json.loads(debug_msg.data)
    except json.JSONDecodeError:
        return False
    return int(debug.get("rollouts", 0)) > 0


def collect_probe(args, topic_prefix):
    rclpy.init(args=None)
    node = rclpy.create_node("mppi_corner_probe_client")
    received = {"cmd": None, "markers": None, "trajectory": None, "debug": None}

    path_pub = node.create_publisher(PathMsg, f"{topic_prefix}/local_path", 10)
    node.create_subscription(
        AckermannDriveStamped,
        f"{topic_prefix}/ackermann_auto",
        lambda msg: received.__setitem__("cmd", msg),
        10)
    node.create_subscription(
        MarkerArray,
        f"{topic_prefix}/trajectory_markers",
        lambda msg: received.__setitem__("markers", msg),
        10)
    node.create_subscription(
        PathMsg,
        f"{topic_prefix}/optimal_trajectory",
        lambda msg: received.__setitem__("trajectory", msg),
        10)
    node.create_subscription(
        String,
        f"{topic_prefix}/debug_status",
        lambda msg: received.__setitem__("debug", msg),
        10)

    deadline = time.monotonic() + args.duration
    valid_since = None
    while time.monotonic() < deadline:
        path_pub.publish(make_path(node, args))
        rclpy.spin_once(node, timeout_sec=0.05)
        if has_valid_probe_output(received):
            if valid_since is None:
                valid_since = time.monotonic()
            if time.monotonic() - valid_since >= args.settle_time:
                break
        else:
            valid_since = None

    node.destroy_node()
    rclpy.shutdown()
    return received


def evaluate(received, require_corner_features):
    issues = []
    summary = {}
    debug = {}
    debug_msg = received["debug"]

    if debug_msg is None:
        issues.append("no debug_status received")
    else:
        try:
            debug = json.loads(debug_msg.data)
        except json.JSONDecodeError as exc:
            issues.append(f"debug_status is not valid JSON: {exc}")
            debug = {}

    cmd = received["cmd"]
    if cmd is None:
        issues.append("no command received")
    else:
        debug_speed = as_float(debug.get("speed"))
        debug_steering_deg = as_float(debug.get("steering_deg"))
        summary["speed"] = (
            debug_speed if math.isfinite(debug_speed) else float(cmd.drive.speed))
        summary["steering"] = (
            debug_steering_deg * math.pi / 180.0
            if math.isfinite(debug_steering_deg)
            else float(cmd.drive.steering_angle))

    trajectory = received["trajectory"]
    if trajectory is None:
        issues.append("no optimal trajectory received")
    else:
        summary["trajectory_poses"] = len(trajectory.poses)

    markers = received["markers"]
    if markers is None:
        issues.append("no rollout markers received")
    else:
        namespaces = Counter(marker.ns for marker in markers.markers)
        line_point_max = Counter()
        for marker in markers.markers:
            if marker.action == Marker.DELETEALL:
                continue
            if marker.type == Marker.LINE_STRIP:
                line_point_max[marker.ns] = max(
                    line_point_max[marker.ns],
                    len(marker.points))
        summary["marker_namespaces"] = dict(sorted(namespaces.items()))
        summary["line_point_max"] = dict(sorted(line_point_max.items()))
        if namespaces.get("mppi_rollouts", 0) <= 0:
            issues.append("missing mppi_rollouts markers")
        if namespaces.get("mppi_best_sampled", 0) <= 0:
            issues.append("missing mppi_best_sampled marker")
        if namespaces.get("mppi_optimal", 0) <= 0:
            issues.append("missing mppi_optimal marker")
        if namespaces.get("mppi_diagnostics", 0) <= 0:
            issues.append("missing mppi_diagnostics markers")
        for namespace in ("mppi_rollouts", "mppi_best_sampled", "mppi_optimal"):
            if line_point_max.get(namespace, 0) < 2:
                issues.append(f"{namespace} line strip has fewer than 2 points")
        texts = [
            marker.text for marker in markers.markers
            if marker.ns == "mppi_diagnostics" and marker.text
        ]
        if texts:
            summary["diagnostic_text"] = texts[0]
        else:
            issues.append("missing mppi_diagnostics text marker")

    if debug:
        summary["debug"] = {
            key: debug.get(key)
            for key in (
                "corner_speed_target",
                "tuning_profile",
                "max_accel",
                "max_steering_rate",
                "iteration_count",
                "temperature",
                "lateral_accel_speed_target",
                "lateral_accel_limit",
                "use_lateral_accel_speed_limit",
                "effective_speed_target",
                "tracking_min_command_speed",
                "tracking_speed_floor",
                "curvature_lookahead_actual_distance",
                "curvature_estimate",
                "command_lateral_accel_estimate",
                "steering_target_deg",
                "steering_error_deg",
                "steering_rate_weight",
                "steering_feedforward_lookahead_distance",
                "steering_feedforward_weight",
                "steering_feedforward_command_gain",
                "use_corner_speed_as_speed_target",
                "command_speed_before_limit",
                "command_speed_limited",
                "command_speed_floor_applied",
                "best_sampled_rollout_cost",
                "selected_minus_best_sampled_cost",
                "rollouts",
            )
        }
        if require_corner_features and debug.get("use_corner_speed_as_speed_target") is not True:
            issues.append("use_corner_speed_as_speed_target is not active")
        if require_corner_features and debug.get("use_lateral_accel_speed_limit") is not True:
            issues.append("use_lateral_accel_speed_limit is not active")
        corner_target = debug.get("corner_speed_target")
        effective_target = debug.get("effective_speed_target")
        target_speed = debug.get("target_speed")
        if (
            require_corner_features and
            isinstance(corner_target, (int, float)) and
            isinstance(target_speed, (int, float))):
            if corner_target >= target_speed:
                issues.append("corner_speed_target did not drop below target_speed")
        if (
            require_corner_features and
            isinstance(effective_target, (int, float)) and
            isinstance(target_speed, (int, float))):
            if effective_target >= target_speed:
                issues.append("effective_speed_target did not drop below target_speed")

    return summary, issues


def as_float(value, default=math.nan):
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def param_override(text):
    if ":=" in text:
        return text
    if "=" not in text:
        raise argparse.ArgumentTypeError("parameter override must be NAME=VALUE")
    name, value = text.split("=", 1)
    if not name.strip() or not value.strip():
        raise argparse.ArgumentTypeError("parameter override must be NAME=VALUE")
    return f"{name.strip()}:={value.strip()}"


def main():
    parser = argparse.ArgumentParser(
        description="Safely probe MPPI corner response on isolated topics.")
    parser.add_argument("--executable", default=default_executable())
    parser.add_argument("--config", default=default_config())
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.5,
        help="Seconds to keep collecting after the first valid output. Default: 0.5")
    parser.add_argument("--points", type=int, default=32)
    parser.add_argument("--spacing", type=float, default=0.15)
    parser.add_argument("--curvature", type=float, default=0.18)
    parser.add_argument(
        "--heading-deg",
        type=float,
        default=0.0,
        help="Rotate the synthetic path in the body frame. Default: 0.")
    parser.add_argument(
        "--set-param",
        action="append",
        type=param_override,
        default=[],
        metavar="NAME=VALUE",
        help="Extra private-node parameter override. May be repeated.")
    parser.add_argument("--max-visualized-rollouts", type=int, default=12)
    parser.add_argument(
        "--profile",
        choices=["config"] + sorted(PROFILES.keys()),
        default="config",
        help="Apply a tuning profile to the isolated node before probing. Default: config")
    parser.add_argument(
        "--profile-timeout",
        type=float,
        default=8.0,
        help="Seconds to wait while applying --profile. Default: 8.0")
    parser.add_argument(
        "--no-require-corner-features",
        action="store_true",
        help="Do not fail if corner speed target features are disabled.")
    parser.add_argument("--keep-node", action="store_true")
    args = parser.parse_args()

    if not args.executable or not Path(args.executable).exists():
        print(f"MPPI executable not found: {args.executable}", file=sys.stderr)
        return 1

    node_name = f"mppi_corner_probe_{os.getpid()}"
    topic_prefix = f"/{node_name}"
    probe_config = make_probe_config(args.config, node_name)
    process = start_planner(args, node_name, topic_prefix, probe_config)

    try:
        time.sleep(2.0)
        if apply_profile(args, node_name) != 0:
            print(f"Failed to apply profile '{args.profile}'", file=sys.stderr)
            return 1
        received = collect_probe(args, topic_prefix)
        require_corner_features = (
            not args.no_require_corner_features and args.profile != "baseline")
        summary, issues = evaluate(received, require_corner_features)
        summary["profile"] = args.profile
        print(json.dumps(summary, indent=2, sort_keys=True))
        if issues:
            print("MPPI corner probe: FAIL", file=sys.stderr)
            for issue in issues:
                print(f"- {issue}", file=sys.stderr)
            return 1
        print("MPPI corner probe: OK")
        return 0
    finally:
        if not args.keep_node:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        if probe_config:
            try:
                os.unlink(probe_config)
            except OSError:
                pass


if __name__ == "__main__":
    sys.exit(main())
