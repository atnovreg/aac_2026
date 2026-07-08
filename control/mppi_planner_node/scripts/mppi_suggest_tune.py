#!/usr/bin/env python3
"""Turn an MPPI drive debug CSV into a concrete next tuning action."""

import argparse
import csv
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

from mppi_analyze_log import analyze


def clamp(value, low, high):
    return max(low, min(value, high))


def command_line(command):
    return " ".join(command)


def ros_param_set(node, name, value):
    return ["ros2", "param", "set", node, name, str(value).lower()]


def tune_profile(node, profile, timeout, verify=True):
    command = [
        "ros2", "run", "mppi_planner_node", "mppi_tune.py", profile,
        "--node", node,
        "--timeout", f"{timeout:.1f}",
    ]
    if verify:
        command.append("--verify")
    return command


def next_lateral_limit(current, fallback, scale, low=0.25, high=0.80):
    if not math.isfinite(current):
        current = fallback
    return clamp(current * scale, low, high)


def make_plan(result, args):
    hint = result.get("hint", "")
    node = args.node
    timeout = args.timeout
    commands = []
    diagnosis = ""

    lateral_limit = result.get("lateral_limit_avg", math.nan)
    speed_over = result.get("speed_over_target_p95", 0.0)
    steering_response = result.get("steering_response_ratio_avg", 1.0)
    steering_error = result.get("steering_error_abs_p95", 0.0)
    early_lag = result.get("early_steering_lag_ratio", 0.0)
    target_min = result.get("target_min", math.nan)

    if hint == "empty_log":
        diagnosis = "CSV is empty; record a drive trial before tuning."
    elif hint == "wait_for_corner":
        diagnosis = "No corner samples were detected; drive through a corner or lower --corner-threshold-deg."
    elif hint == "corner_features_disabled_restart_or_apply_early":
        diagnosis = "Corner-speed features are missing or disabled on the running node."
        commands = [
            tune_profile(node, "early", timeout),
        ]
    elif hint == "optimizer_gap_increase_iterations_or_lower_temperature":
        diagnosis = (
            "MPPI sampled a better rollout than the final selected trajectory; "
            "make the weighted update commit more strongly.")
        commands = [
            ros_param_set(node, "iteration_count", 3),
            ros_param_set(node, "temperature", 0.35),
        ]
    elif hint == "tracking_floor_disabled_or_restart":
        diagnosis = "The controller is still outputting near-zero speed in corners; enable the tracking crawl floor."
        commands = [
            ros_param_set(node, "tracking_min_command_speed", 0.25),
            tune_profile(node, "early", timeout),
        ]
    elif hint == "steering_lag_try_aggressive_or_raise_steering_rate":
        diagnosis = (
            "Steering response is lagging the target, especially near corner entry; "
            "make the controller look further and allow faster steering changes.")
        commands = [
            tune_profile(node, "aggressive", timeout),
            ros_param_set(node, "max_steering_rate", 5.0),
            ros_param_set(node, "steering_feedforward_command_gain", 0.65),
            ros_param_set(node, "steering_rate_weight", 0.04),
            ros_param_set(node, "heading_lookahead_weight", 4.0),
        ]
    elif hint == "try_aggressive_or_lower_corner_speed_margin":
        diagnosis = "The speed command is staying high before the corner limiter engages."
        commands = [
            tune_profile(node, "aggressive", timeout),
            ros_param_set(node, "corner_speed_margin", 0.02),
        ]
    elif hint == "limit_active_raise_max_accel_or_lower_corner_speed_margin":
        diagnosis = "The corner limiter is active, but acceleration limits prevent speed from dropping fast enough."
        commands = [
            ros_param_set(node, "max_accel", 2.5),
            ros_param_set(node, "corner_speed_margin", 0.02),
        ]
    elif hint == "lower_lateral_accel_limit_or_try_slow_corner":
        next_limit = next_lateral_limit(lateral_limit, 0.45, 0.80, low=0.25, high=0.55)
        diagnosis = "The vehicle is still too fast in corners and the lateral-accel target is not dominant enough."
        commands = [
            tune_profile(node, "slow_corner", timeout),
            ros_param_set(node, "lateral_accel_limit", f"{next_limit:.2f}"),
            ros_param_set(node, "corner_speed_margin", 0.0),
        ]
    elif hint == "try_slow_corner_for_tighter_turns":
        diagnosis = "Tight turns are still above target; try the slower corner-entry preset."
        commands = [
            tune_profile(node, "slow_corner", timeout),
        ]
    elif hint == "too_slow_raise_limits_or_use_stable":
        next_limit = next_lateral_limit(lateral_limit, 0.45, 1.20, low=0.35, high=0.80)
        diagnosis = "The controller is slowing more than needed; back off to a steadier profile."
        commands = [
            tune_profile(node, "stable", timeout),
            ros_param_set(node, "lateral_accel_limit", f"{next_limit:.2f}"),
            ros_param_set(node, "corner_speed_margin", 0.10),
        ]
    else:
        diagnosis = "The log looks reasonable numerically; use RViz rollouts and vehicle response to decide."
        if (
            args.assume_late and
            math.isfinite(target_min) and
            target_min > 0.45 and
            speed_over > 0.08
        ):
            diagnosis = "Metrics are close, but you marked the drive as late; reduce the corner target one step."
            commands = [
                tune_profile(node, "slow_corner", timeout),
            ]
        elif (
            args.assume_late and
            (
                (steering_response < 0.85 and steering_error > args.steering_threshold_deg) or
                early_lag >= 0.35
            )
        ):
            diagnosis = "Metrics are close, but steering response still looks late."
            commands = [
                tune_profile(node, "aggressive", timeout),
                ros_param_set(node, "max_steering_rate", 5.0),
                ros_param_set(node, "steering_feedforward_command_gain", 0.65),
            ]

    return diagnosis, commands


def print_summary(result):
    print(f"rows={result.get('rows', 0)} corner_rows={result.get('corner_rows', 0)}")
    if result.get("profiles"):
        profiles = ", ".join(
            f"{name}:{count}" for name, count in result["profiles"].items())
        print(f"profiles={profiles}")
    print(f"hint={result.get('hint', '')}")
    if result.get("corner_rows", 0) <= 0:
        return

    keys = (
        "max_corner_deg",
        "speed_avg",
        "target_min",
        "speed_over_target_p95",
        "command_lateral_accel_p95",
        "limited_ratio",
        "lateral_active_ratio",
        "steering_target_abs_p95",
        "steering_error_abs_p95",
        "steering_response_ratio_avg",
        "early_steering_response_ratio_avg",
        "early_steering_lag_ratio",
        "steering_response_delay_p95",
        "max_accel_last",
        "max_steering_rate_last",
        "iteration_count_last",
        "temperature_last",
        "steering_rate_weight_last",
        "steering_feedforward_command_gain_last",
        "selected_best_cost_gap_p95",
    )
    for key in keys:
        value = result.get(key, math.nan)
        if key.endswith("_ratio"):
            print(f"{key}={value * 100.0:.1f}%")
        else:
            print(f"{key}={value:.3f}")


def apply_commands(commands, timeout):
    for command in commands:
        print(f"apply: {command_line(command)}", flush=True)
        try:
            completed = subprocess.run(command, check=False, timeout=timeout)
        except subprocess.TimeoutExpired:
            print(f"command timed out after {timeout:.1f}s", file=sys.stderr)
            return 124
        if completed.returncode != 0:
            print(f"command failed rc={completed.returncode}", file=sys.stderr)
            return completed.returncode
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Suggest the next MPPI tuning command from a drive-trial CSV.")
    parser.add_argument("csv_path", help="CSV file from mppi_log_debug.py or mppi_drive_trial.py")
    parser.add_argument("--node", default="/mppi_planner_node")
    parser.add_argument("--corner-threshold-deg", type=float, default=12.0)
    parser.add_argument("--steering-threshold-deg", type=float, default=8.0)
    parser.add_argument("--optimizer-gap-threshold", type=float, default=1.0)
    parser.add_argument("--early-window-sec", type=float, default=1.0)
    parser.add_argument("--corner-event-gap-sec", type=float, default=0.5)
    parser.add_argument("--late-response-threshold", type=float, default=0.75)
    parser.add_argument("--timeout", type=float, default=8.0)
    parser.add_argument(
        "--assume-late",
        action="store_true",
        help="Bias the fallback recommendation toward earlier corner entry.")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Apply the suggested commands instead of only printing them.")
    args = parser.parse_args()

    path = Path(args.csv_path).expanduser()
    if not path.exists():
        print(f"CSV file not found: {path}", file=sys.stderr)
        return 1

    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))

    analyze_args = SimpleNamespace(
        corner_threshold_deg=args.corner_threshold_deg,
        steering_threshold_deg=args.steering_threshold_deg,
        optimizer_gap_threshold=args.optimizer_gap_threshold,
        early_window_sec=args.early_window_sec,
        corner_event_gap_sec=args.corner_event_gap_sec,
        late_response_threshold=args.late_response_threshold)
    result = analyze(rows, analyze_args)
    diagnosis, commands = make_plan(result, args)

    print_summary(result)
    print(f"diagnosis={diagnosis}")
    if commands:
        print("commands:")
        for command in commands:
            print(f"  {command_line(command)}")
    else:
        print("commands: none")

    if args.apply and commands:
        return apply_commands(commands, args.timeout)
    if args.apply and not commands:
        print("No commands to apply.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
