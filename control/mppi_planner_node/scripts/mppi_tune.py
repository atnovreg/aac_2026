#!/usr/bin/env python3
"""Apply runtime MPPI tuning presets."""

import argparse
import subprocess
import sys


PROFILES = {
    "baseline": {
        "target_speed": 0.9,
        "max_speed": 1.1,
        "tracking_min_command_speed": 0.10,
        "heading_weight": 3.0,
        "heading_lookahead_distance": 1.2,
        "heading_lookahead_weight": 2.5,
        "steering_feedforward_lookahead_distance": 1.2,
        "steering_feedforward_weight": 0.0,
        "steering_feedforward_command_gain": 0.0,
        "terminal_heading_weight": 5.0,
        "curvature_lookahead_distance": 1.8,
        "curvature_speed_weight": 0.8,
        "curvature_speed_gain": 0.8,
        "curvature_min_speed": 0.45,
        "lateral_accel_limit": 0.60,
        "use_lateral_accel_speed_limit": False,
        "use_corner_speed_as_speed_target": False,
        "limit_command_speed_to_corner": False,
        "corner_speed_margin": 0.05,
        "steering_noise_std": 0.34,
        "steering_rate_weight": 0.08,
        "max_steering_rate": 4.0,
        "progress_weight": 0.7,
    },
    "early": {
        "heading_weight": 3.5,
        "heading_lookahead_distance": 1.5,
        "heading_lookahead_weight": 3.0,
        "steering_feedforward_lookahead_distance": 1.4,
        "steering_feedforward_weight": 0.25,
        "steering_feedforward_command_gain": 0.20,
        "tracking_min_command_speed": 0.25,
        "terminal_heading_weight": 6.0,
        "curvature_speed_weight": 1.2,
        "curvature_speed_gain": 1.0,
        "lateral_accel_limit": 0.45,
        "use_lateral_accel_speed_limit": True,
        "use_corner_speed_as_speed_target": True,
        "limit_command_speed_to_corner": True,
        "corner_speed_margin": 0.05,
        "steering_noise_std": 0.40,
        "progress_weight": 0.5,
    },
    "aggressive": {
        "target_speed": 0.8,
        "heading_weight": 4.0,
        "heading_lookahead_distance": 1.8,
        "heading_lookahead_weight": 4.0,
        "steering_feedforward_lookahead_distance": 1.6,
        "steering_feedforward_weight": 0.45,
        "steering_feedforward_command_gain": 0.60,
        "tracking_min_command_speed": 0.25,
        "terminal_heading_weight": 7.0,
        "curvature_lookahead_distance": 2.0,
        "curvature_speed_weight": 1.5,
        "curvature_speed_gain": 1.2,
        "curvature_min_speed": 0.35,
        "lateral_accel_limit": 0.35,
        "use_lateral_accel_speed_limit": True,
        "use_corner_speed_as_speed_target": True,
        "limit_command_speed_to_corner": True,
        "corner_speed_margin": 0.02,
        "steering_noise_std": 0.45,
        "steering_rate_weight": 0.05,
        "max_steering_rate": 5.0,
        "progress_weight": 0.35,
    },
    "stable": {
        "target_speed": 0.85,
        "heading_weight": 2.8,
        "heading_lookahead_distance": 1.1,
        "heading_lookahead_weight": 1.5,
        "steering_feedforward_lookahead_distance": 1.0,
        "steering_feedforward_weight": 0.10,
        "steering_feedforward_command_gain": 0.10,
        "tracking_min_command_speed": 0.18,
        "terminal_heading_weight": 4.0,
        "curvature_speed_weight": 0.4,
        "curvature_speed_gain": 0.5,
        "lateral_accel_limit": 0.55,
        "use_lateral_accel_speed_limit": True,
        "use_corner_speed_as_speed_target": True,
        "limit_command_speed_to_corner": True,
        "corner_speed_margin": 0.08,
        "steering_noise_std": 0.28,
        "steering_rate_weight": 0.12,
        "max_steering_rate": 3.0,
        "progress_weight": 0.8,
    },
    "slow_corner": {
        "target_speed": 0.7,
        "max_speed": 0.9,
        "heading_lookahead_distance": 1.6,
        "heading_lookahead_weight": 3.2,
        "steering_feedforward_lookahead_distance": 1.4,
        "steering_feedforward_weight": 0.35,
        "steering_feedforward_command_gain": 0.50,
        "tracking_min_command_speed": 0.18,
        "curvature_lookahead_distance": 2.0,
        "curvature_speed_weight": 1.6,
        "curvature_speed_gain": 1.4,
        "curvature_min_speed": 0.30,
        "lateral_accel_limit": 0.30,
        "use_lateral_accel_speed_limit": True,
        "use_corner_speed_as_speed_target": True,
        "limit_command_speed_to_corner": True,
        "corner_speed_margin": 0.0,
        "steering_noise_std": 0.38,
        "progress_weight": 0.45,
    },
}


def service_name_for_node(node):
    normalized = node if node.startswith("/") else f"/{node}"
    return f"{normalized.rstrip('/')}/set_parameters_atomically"


def print_profile_commands(node, profile):
    for name, value in profile.items():
        command = ["ros2", "param", "set", node, name, str(value).lower()]
        print(" ".join(command), flush=True)


def profile_with_name(name):
    profile = dict(PROFILES[name])
    profile["tuning_profile"] = name
    return profile


def make_parameter_msg(name, value):
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

    parameter = Parameter()
    parameter.name = name
    parameter.value = ParameterValue()
    if isinstance(value, bool):
        parameter.value.type = ParameterType.PARAMETER_BOOL
        parameter.value.bool_value = value
    elif isinstance(value, int):
        parameter.value.type = ParameterType.PARAMETER_INTEGER
        parameter.value.integer_value = value
    elif isinstance(value, str):
        parameter.value.type = ParameterType.PARAMETER_STRING
        parameter.value.string_value = value
    else:
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = float(value)
    return parameter


def apply_profile_atomically(node, profile, timeout):
    try:
        import rclpy
        from rcl_interfaces.srv import SetParametersAtomically
    except ImportError as exc:
        print(f"Atomic mode needs rclpy and rcl_interfaces: {exc}", file=sys.stderr)
        return 127

    rclpy.init(args=None)
    client_node = rclpy.create_node("mppi_tune_client")
    try:
        service_name = service_name_for_node(node)
        client = client_node.create_client(SetParametersAtomically, service_name)
        if not client.wait_for_service(timeout_sec=timeout):
            print(
                f"Timed out after {timeout:.1f}s waiting for "
                f"{service_name}.",
                file=sys.stderr)
            return 124

        request = SetParametersAtomically.Request()
        request.parameters = [
            make_parameter_msg(name, value) for name, value in profile.items()
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=timeout)
        if not future.done():
            print(
                f"Timed out after {timeout:.1f}s while applying profile to {node}.",
                file=sys.stderr)
            return 124

        response = future.result()
        if response is None:
            print(f"Failed to apply profile to {node}: empty response.", file=sys.stderr)
            return 1
        if not response.result.successful:
            reason = response.result.reason or "node rejected the parameter update"
            print(f"Failed to apply profile to {node}: {reason}", file=sys.stderr)
            if "not declared" in reason.lower():
                print(
                    "The running MPPI node is probably stale. Restart the MPPI launch pane "
                    "and run mppi_runtime_check.py before tuning.",
                    file=sys.stderr)
            return 1
        return 0
    finally:
        client_node.destroy_node()
        rclpy.shutdown()


def run_param_set(node, name, value, dry_run, timeout):
    command = ["ros2", "param", "set", node, name, str(value).lower()]
    print(" ".join(command), flush=True)
    if dry_run:
        return 0
    try:
        completed = subprocess.run(command, check=False, timeout=timeout)
    except subprocess.TimeoutExpired:
        print(
            f"Timed out after {timeout:.1f}s while setting {name} on {node}.",
            file=sys.stderr)
        return 124
    return completed.returncode


def min_command_gain(profile_name):
    try:
        return float(PROFILES.get(profile_name, {}).get("steering_feedforward_command_gain", 0.0))
    except (TypeError, ValueError):
        return 0.0


def verify_profile(node, profile_name, timeout, dry_run=False):
    command = [
        "ros2", "run", "mppi_planner_node", "mppi_runtime_check.py",
        "--node", node,
        "--expect-profile", profile_name,
        "--min-steering-feedforward-command-gain",
        f"{min_command_gain(profile_name):.2f}",
        "--timeout", f"{timeout:.1f}",
        "--retries", "0",
    ]
    if profile_name == "baseline":
        command.append("--allow-disabled-corner-features")

    print(" ".join(command), flush=True)
    if dry_run:
        return 0
    verify_timeout = max(30.0, timeout * 12.0)
    try:
        completed = subprocess.run(command, check=False, timeout=verify_timeout)
    except subprocess.TimeoutExpired:
        print(
            f"Timed out after {verify_timeout:.1f}s while verifying {profile_name} on {node}.",
            file=sys.stderr)
        return 124
    return completed.returncode


def main():
    parser = argparse.ArgumentParser(
        description="Apply MPPI path tracking tuning presets while the node is running.")
    parser.add_argument(
        "profile",
        nargs="?",
        choices=sorted(PROFILES.keys()),
        help="Preset to apply. Use early first, aggressive if corners are still late, stable if oscillating.")
    parser.add_argument(
        "--profile",
        dest="profile_option",
        choices=sorted(PROFILES.keys()),
        help="Alias for the positional profile, useful in drive-trial checklists.")
    parser.add_argument(
        "--node",
        default="/mppi_planner_node",
        help="Target node name. Default: /mppi_planner_node")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print ros2 param commands without applying them.")
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="Sequential mode only: keep applying later parameters if one set command fails.")
    parser.add_argument(
        "--sequential",
        action="store_true",
        help="Use individual ros2 param set commands instead of the atomic parameter service.")
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Seconds to wait for each ros2 param set command. Default: 8.0")
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Run mppi_runtime_check.py after applying the profile.")
    args = parser.parse_args()

    selected_profile = args.profile or args.profile_option
    if args.profile and args.profile_option and args.profile != args.profile_option:
        parser.error("positional profile and --profile must match when both are provided")
    if not selected_profile:
        parser.error("the following arguments are required: profile")

    profile = profile_with_name(selected_profile)
    if args.dry_run or not args.sequential:
        print_profile_commands(args.node, profile)

    if args.dry_run:
        if args.verify:
            verify_profile(args.node, selected_profile, args.timeout, dry_run=True)
        mode = "sequential" if args.sequential else "atomic"
        print(f"Dry run complete for profile '{selected_profile}' in {mode} mode.")
        return 0

    if not args.sequential:
        rc = apply_profile_atomically(args.node, profile, args.timeout)
        if rc == 0:
            print(f"Applied MPPI tuning profile '{selected_profile}' to {args.node}.")
            if args.verify:
                return verify_profile(args.node, selected_profile, args.timeout)
        return rc

    failures = 0
    for name, value in profile.items():
        rc = run_param_set(args.node, name, value, args.dry_run, args.timeout)
        if rc != 0:
            failures += 1
            if not args.keep_going:
                return rc

    if failures:
        print(f"Finished with {failures} failed parameter update(s).", file=sys.stderr)
        return 1

    print(f"Applied MPPI tuning profile '{selected_profile}' to {args.node}.")
    if args.verify:
        return verify_profile(args.node, selected_profile, args.timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
