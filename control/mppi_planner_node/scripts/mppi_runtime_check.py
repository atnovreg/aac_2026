#!/usr/bin/env python3
"""Check whether the running MPPI node is using the latest cornering features."""

import argparse
import os
import subprocess
import sys
import time


REQUIRED_PARAMETERS = (
    "tuning_profile",
    "steering_feedforward_weight",
    "steering_feedforward_lookahead_distance",
    "steering_feedforward_command_gain",
    "tracking_min_command_speed",
    "use_corner_speed_as_speed_target",
    "use_lateral_accel_speed_limit",
    "lateral_accel_limit",
    "limit_command_speed_to_corner",
    "visualize_rollouts",
)

REQUIRED_TRUE_PARAMETERS = (
    "use_corner_speed_as_speed_target",
    "use_lateral_accel_speed_limit",
    "limit_command_speed_to_corner",
    "visualize_rollouts",
)


def run(command, timeout=5.0):
    try:
        return subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(
            command,
            124,
            stdout=f"Timed out after {timeout:.1f}s")


def find_pids(node_name):
    completed = run(["pgrep", "-af", f"__node:={node_name.lstrip('/')}"])
    pids = []
    for line in completed.stdout.splitlines():
        parts = line.split(maxsplit=1)
        if not parts:
            continue
        if "mppi_planner_exe" not in line:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue

        try:
            actual = os.readlink(f"/proc/{pid}/exe")
        except OSError:
            continue
        if "mppi_planner_exe" in actual:
            pids.append(pid)
    return pids


def check_executable(pids, expected_executable):
    issues = []
    for pid in pids:
        proc_exe = f"/proc/{pid}/exe"
        try:
            actual = os.readlink(proc_exe)
        except OSError as exc:
            issues.append(f"pid {pid}: cannot read executable: {exc}")
            continue

        if actual.endswith(" (deleted)"):
            issues.append(f"pid {pid}: running deleted executable: {actual}")
            continue

        if expected_executable and os.path.realpath(actual) != os.path.realpath(expected_executable):
            issues.append(
                f"pid {pid}: executable mismatch: {actual} != {expected_executable}")
    return issues


def check_parameters(node_name, parameters, timeout, retries):
    issues = []
    values = {}
    for parameter in parameters:
        completed = None
        for attempt in range(max(0, retries) + 1):
            completed = run(["ros2", "param", "get", node_name, parameter], timeout=timeout)
            if completed.returncode == 0 and "Timed out" not in completed.stdout:
                break
            if attempt < max(0, retries):
                time.sleep(0.25)
        output = completed.stdout.strip()
        values[parameter] = output
        if completed.returncode != 0 or "Parameter not set" in output:
            issues.append(f"{parameter}: not available on {node_name}")
    return values, issues


def parse_param_value(output):
    if "value is:" not in output:
        return ""
    return output.rsplit("value is:", 1)[1].strip()


def as_bool_value(output):
    value = parse_param_value(output).lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    return None


def as_float_value(output):
    try:
        return float(parse_param_value(output))
    except ValueError:
        return None


def check_parameter_values(parameter_values, args):
    issues = []
    if args.expect_profile:
        actual = parse_param_value(parameter_values.get("tuning_profile", ""))
        if actual != args.expect_profile:
            issues.append(
                f"tuning_profile: expected {args.expect_profile}, got {actual or 'unknown'}")

    if not args.allow_disabled_corner_features:
        for name in REQUIRED_TRUE_PARAMETERS:
            value = as_bool_value(parameter_values.get(name, ""))
            if value is not True:
                actual = parse_param_value(parameter_values.get(name, "")) or "unknown"
                issues.append(f"{name}: expected true, got {actual}")

    gain = as_float_value(parameter_values.get("steering_feedforward_command_gain", ""))
    if gain is not None and gain < args.min_steering_feedforward_command_gain:
        issues.append(
            "steering_feedforward_command_gain: "
            f"expected >= {args.min_steering_feedforward_command_gain:.2f}, got {gain:.2f}")

    return issues


def main():
    parser = argparse.ArgumentParser(
        description="Check MPPI runtime freshness before driving.")
    parser.add_argument(
        "--node",
        default="/mppi_planner_node",
        help="MPPI node name. Default: /mppi_planner_node")
    parser.add_argument(
        "--expected-executable",
        default="/ros_ws/install/mppi_planner_node/lib/mppi_planner_node/mppi_planner_exe",
        help="Expected installed executable path inside the container.")
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Only print failures.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for each ROS CLI query. Default: 10.0")
    parser.add_argument(
        "--retries",
        type=int,
        default=1,
        help="Retry each ROS parameter query this many times. Default: 1.")
    parser.add_argument(
        "--expect-profile",
        default="",
        help="Fail unless tuning_profile matches this value.")
    parser.add_argument(
        "--allow-disabled-corner-features",
        action="store_true",
        help="Do not require corner speed, lateral limit, command limiter, and rollout visualization to be enabled.")
    parser.add_argument(
        "--min-steering-feedforward-command-gain",
        type=float,
        default=0.0,
        help="Fail if steering_feedforward_command_gain is below this value. Default: 0.")
    parser.add_argument(
        "--check-parameters-on-executable-error",
        action="store_true",
        help="Continue querying ROS parameters even if the running executable is stale or mismatched.")
    args = parser.parse_args()

    issues = []
    pids = find_pids(args.node)
    if not pids:
        issues.append(f"no running mppi_planner_exe found for {args.node}")
    else:
        issues.extend(check_executable(pids, args.expected_executable))

    parameter_values = {}
    should_check_parameters = not issues or args.check_parameters_on_executable_error
    if should_check_parameters:
        parameter_values, parameter_issues = check_parameters(
            args.node, REQUIRED_PARAMETERS, args.timeout, args.retries)
        issues.extend(parameter_issues)
        if not parameter_issues:
            issues.extend(check_parameter_values(parameter_values, args))

    if not args.quiet:
        print(f"node={args.node}")
        print(f"pids={pids if pids else 'none'}")
        if parameter_values:
            for name, value in parameter_values.items():
                print(f"{name}: {value}")
        else:
            print("parameters: skipped because executable check failed")

    if issues:
        print("MPPI runtime check: FAIL", file=sys.stderr)
        for issue in issues:
            print(f"- {issue}", file=sys.stderr)
        print(
            "Restart the MPPI launch pane, then run this check again.",
            file=sys.stderr)
        return 1

    if not args.quiet:
        print("MPPI runtime check: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
