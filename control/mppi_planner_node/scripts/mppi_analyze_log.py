#!/usr/bin/env python3
"""Analyze an MPPI debug CSV log and print tuning suggestions."""

import argparse
import csv
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


def as_float(row, name, default=math.nan):
    try:
        value = row.get(name, "")
        if value == "":
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes", "on")
    return bool(value)


def finite(values):
    return [value for value in values if math.isfinite(value)]


def mean(values, default=math.nan):
    clean = finite(values)
    if not clean:
        return default
    return statistics.fmean(clean)


def bool_ratio(rows, name):
    values = [
        as_bool(row.get(name))
        for row in rows
        if row.get(name, "") != ""
    ]
    if not values:
        return 0.0
    return sum(1 for value in values if value) / len(values)


def percentile(values, q, default=math.nan):
    clean = sorted(finite(values))
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


def target_for_row(row):
    target = as_float(row, "effective_speed_target")
    if math.isfinite(target):
        return target
    return as_float(row, "corner_speed_target")


def ensure_analysis_args(args):
    defaults = {
        "corner_threshold_deg": 12.0,
        "steering_threshold_deg": 8.0,
        "optimizer_gap_threshold": 1.0,
        "early_window_sec": 1.0,
        "corner_event_gap_sec": 0.5,
        "late_response_threshold": 0.75,
    }
    for name, value in defaults.items():
        if not hasattr(args, name):
            setattr(args, name, value)
    return args


def row_time(row, index):
    stamp = as_float(row, "stamp")
    if math.isfinite(stamp):
        return stamp
    return index / 30.0


def steering_response_ratio(row, steering_threshold_deg):
    target = abs(as_float(row, "steering_target_deg"))
    steering = abs(as_float(row, "steering_deg"))
    if (
        math.isfinite(target) and
        target >= steering_threshold_deg and
        math.isfinite(steering)
    ):
        return min(2.0, steering / target)
    return math.nan


def corner_events(rows, args):
    events = []
    current = []
    last_time = math.nan
    for index, row in enumerate(rows):
        is_corner = (
            abs(as_float(row, "curvature_heading_change_deg", 0.0)) >=
            args.corner_threshold_deg
        )
        current_time = row_time(row, index)
        if not is_corner:
            if current:
                events.append(current)
                current = []
            last_time = current_time
            continue

        if (
            current and
            math.isfinite(current_time) and
            math.isfinite(last_time) and
            current_time - last_time > args.corner_event_gap_sec
        ):
            events.append(current)
            current = []

        current.append((index, row, current_time))
        last_time = current_time

    if current:
        events.append(current)
    return events


def early_steering_metrics(rows, args):
    events = corner_events(rows, args)
    early_rows = []
    response_delays = []

    for event in events:
        start_time = event[0][2]
        event_responses = []
        event_duration = 0.0
        found_response = False
        for _, row, current_time in event:
            if math.isfinite(start_time) and math.isfinite(current_time):
                elapsed = current_time - start_time
            else:
                elapsed = 0.0
            event_duration = max(event_duration, elapsed)
            response = steering_response_ratio(row, args.steering_threshold_deg)
            if math.isfinite(response):
                event_responses.append((elapsed, response))
                if elapsed <= args.early_window_sec:
                    early_rows.append(row)
                if not found_response and response >= args.late_response_threshold:
                    response_delays.append(elapsed)
                    found_response = True

        if event_responses and not found_response:
            response_delays.append(event_duration)

    early_responses = [
        steering_response_ratio(row, args.steering_threshold_deg)
        for row in early_rows
    ]
    early_errors = [abs(as_float(row, "steering_error_deg")) for row in early_rows]
    late_flags = [
        response < args.late_response_threshold
        for response in early_responses
        if math.isfinite(response)
    ]
    return {
        "corner_event_count": len(events),
        "early_corner_rows": len(early_rows),
        "early_steering_response_ratio_avg": mean(early_responses, 1.0),
        "early_steering_error_abs_p95": percentile(early_errors, 0.95, 0.0),
        "early_steering_lag_ratio": (
            sum(1 for flag in late_flags if flag) / len(late_flags)
            if late_flags else 0.0),
        "steering_response_delay_avg": mean(response_delays, 0.0),
        "steering_response_delay_p95": percentile(response_delays, 0.95, 0.0),
    }


def last_float(rows, name, default=math.nan):
    for row in reversed(rows):
        value = as_float(row, name)
        if math.isfinite(value):
            return value
    return default


def command_for_hint(hint, node, lateral_accel_limit):
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
    if hint == "too_slow_raise_limits_or_use_stable":
        next_limit = 0.55
        if math.isfinite(lateral_accel_limit):
            next_limit = min(0.80, lateral_accel_limit * 1.2)
        return (
            f"ros2 run mppi_planner_node mppi_tune.py stable --node {node} --verify && "
            f"ros2 param set {node} lateral_accel_limit {next_limit:.2f} && "
            f"ros2 param set {node} corner_speed_margin 0.10")
    return ""


def analyze(rows, args):
    args = ensure_analysis_args(args)
    corner_rows = [
        row for row in rows
        if abs(as_float(row, "curvature_heading_change_deg", 0.0)) >=
        args.corner_threshold_deg
    ]

    result = {
        "rows": len(rows),
        "corner_rows": len(corner_rows),
        "hint": "wait_for_corner",
    }
    profiles = Counter(
        row.get("tuning_profile", "")
        for row in rows
        if row.get("tuning_profile", ""))
    if profiles:
        result["profiles"] = dict(sorted(profiles.items()))
    if not rows:
        result["hint"] = "empty_log"
        return result
    if not corner_rows:
        return result

    speeds = [as_float(row, "speed") for row in corner_rows]
    targets = [target_for_row(row) for row in corner_rows]
    before_limit = [as_float(row, "command_speed_before_limit") for row in corner_rows]
    corner_angles = [
        abs(as_float(row, "curvature_heading_change_deg", 0.0)) for row in corner_rows
    ]
    lateral_targets = [
        as_float(row, "lateral_accel_speed_target") for row in corner_rows
    ]
    lateral_limits = [as_float(row, "lateral_accel_limit") for row in corner_rows]
    lateral_accels = [
        as_float(row, "command_lateral_accel_estimate") for row in corner_rows
    ]
    rollouts = [as_float(row, "rollouts") for row in corner_rows]
    steering_targets_abs = [
        abs(as_float(row, "steering_target_deg")) for row in corner_rows
    ]
    steering_errors_abs = [
        abs(as_float(row, "steering_error_deg")) for row in corner_rows
    ]
    selected_best_gaps = [
        as_float(row, "selected_minus_best_sampled_cost") for row in corner_rows
    ]
    steering_response_ratios = [
        steering_response_ratio(row, args.steering_threshold_deg)
        for row in corner_rows
    ]
    use_corner_ratio = bool_ratio(corner_rows, "use_corner_speed_as_speed_target")
    use_lateral_ratio = bool_ratio(corner_rows, "use_lateral_accel_speed_limit")
    limiter_enabled_ratio = bool_ratio(corner_rows, "limit_command_speed_to_corner")
    limited_ratio = sum(
        1 for row in corner_rows
        if as_bool(row.get("command_speed_limited", False))) / len(corner_rows)
    floor_applied_ratio = sum(
        1 for row in corner_rows
        if as_bool(row.get("command_speed_floor_applied", False))) / len(corner_rows)
    tracking_speed_floors = [
        as_float(row, "tracking_speed_floor") for row in corner_rows
    ]

    over_target = [
        speed - target for speed, target in zip(speeds, targets)
        if math.isfinite(speed) and math.isfinite(target)
    ]
    before_over_target = [
        speed - target for speed, target in zip(before_limit, targets)
        if math.isfinite(speed) and math.isfinite(target)
    ]
    lateral_active = [
        row for row in corner_rows
        if math.isfinite(as_float(row, "lateral_accel_speed_target")) and
        math.isfinite(target_for_row(row)) and
        as_float(row, "lateral_accel_speed_target") <= target_for_row(row) + 0.03
    ]
    lateral_active_ratio = len(lateral_active) / len(corner_rows)

    avg_over = mean(over_target, 0.0)
    p95_over = percentile(over_target, 0.95, avg_over)
    max_over = max(finite(over_target), default=avg_over)
    avg_before_over = mean(before_over_target, avg_over)
    avg_lateral_limit = mean(lateral_limits)
    early_metrics = early_steering_metrics(rows, args)

    result.update({
        **early_metrics,
        "target_speed_last": last_float(corner_rows, "target_speed"),
        "max_speed_last": last_float(corner_rows, "max_speed"),
        "max_accel_last": last_float(corner_rows, "max_accel"),
        "max_steering_rate_last": last_float(corner_rows, "max_steering_rate"),
        "tracking_min_command_speed_last": last_float(
            corner_rows, "tracking_min_command_speed"),
        "iteration_count_last": last_float(corner_rows, "iteration_count"),
        "temperature_last": last_float(corner_rows, "temperature"),
        "heading_lookahead_weight_last": last_float(corner_rows, "heading_lookahead_weight"),
        "steering_rate_weight_last": last_float(corner_rows, "steering_rate_weight"),
        "steering_feedforward_command_gain_last": last_float(
            corner_rows, "steering_feedforward_command_gain"),
        "max_corner_deg": max(corner_angles),
        "speed_avg": mean(speeds, 0.0),
        "target_min": min(finite(targets), default=0.0),
        "target_avg": mean(targets, 0.0),
        "lateral_target_avg": mean(lateral_targets, 0.0),
        "lateral_limit_avg": avg_lateral_limit,
        "command_lateral_accel_avg": mean(lateral_accels, 0.0),
        "command_lateral_accel_p95": percentile(lateral_accels, 0.95, 0.0),
        "speed_over_target_avg": avg_over,
        "speed_over_target_p95": p95_over,
        "speed_over_target_max": max_over,
        "before_limit_over_target_avg": avg_before_over,
        "limited_ratio": limited_ratio,
        "floor_applied_ratio": floor_applied_ratio,
        "tracking_speed_floor_avg": mean(tracking_speed_floors, 0.0),
        "use_corner_speed_as_speed_target_ratio": use_corner_ratio,
        "use_lateral_accel_speed_limit_ratio": use_lateral_ratio,
        "limit_command_speed_to_corner_ratio": limiter_enabled_ratio,
        "lateral_active_ratio": lateral_active_ratio,
        "steering_target_abs_p95": percentile(steering_targets_abs, 0.95, 0.0),
        "steering_error_abs_avg": mean(steering_errors_abs, 0.0),
        "steering_error_abs_p95": percentile(steering_errors_abs, 0.95, 0.0),
        "steering_response_ratio_avg": mean(steering_response_ratios, 1.0),
        "selected_best_cost_gap_avg": mean(selected_best_gaps, 0.0),
        "selected_best_cost_gap_p95": percentile(selected_best_gaps, 0.95, 0.0),
        "rollouts_min": min(finite(rollouts), default=0.0),
        "rollouts_avg": mean(rollouts, 0.0),
    })

    feature_ratios = [
        ratio for ratio in (use_corner_ratio, use_lateral_ratio, limiter_enabled_ratio)
        if math.isfinite(ratio)
    ]
    if feature_ratios and min(feature_ratios) < 0.5:
        result["hint"] = "corner_features_disabled_restart_or_apply_early"
    elif (
        result["rollouts_min"] > 0.0 and
        result["selected_best_cost_gap_p95"] > args.optimizer_gap_threshold
    ):
        result["hint"] = "optimizer_gap_increase_iterations_or_lower_temperature"
    elif (
        result["speed_avg"] < 0.10 and
        result["target_min"] > 0.15 and
        floor_applied_ratio < 0.2
    ):
        result["hint"] = "tracking_floor_disabled_or_restart"
    elif (
        result["speed_avg"] > 0.20 and
        result["steering_target_abs_p95"] >= args.steering_threshold_deg and
        (
            (
                result["steering_error_abs_p95"] >= args.steering_threshold_deg and
                result["steering_response_ratio_avg"] < 0.70
            ) or
            (
                result["early_corner_rows"] > 0 and
                result["early_steering_lag_ratio"] >= 0.45
            )
        )
    ):
        result["hint"] = "steering_lag_try_aggressive_or_raise_steering_rate"
    elif limited_ratio < 0.2 and avg_before_over > 0.20:
        result["hint"] = "try_aggressive_or_lower_corner_speed_margin"
    elif limited_ratio > 0.6 and p95_over > 0.18:
        result["hint"] = "limit_active_raise_max_accel_or_lower_corner_speed_margin"
    elif p95_over > 0.16 and lateral_active_ratio < 0.4:
        result["hint"] = "lower_lateral_accel_limit_or_try_slow_corner"
    elif avg_over < -0.08:
        result["hint"] = "too_slow_raise_limits_or_use_stable"
    else:
        result["hint"] = "looks_reasonable_verify_vehicle_response"

    return result


def print_result(result, args):
    print(f"rows={result['rows']} corner_rows={result['corner_rows']}")
    if result.get("profiles"):
        profiles = ", ".join(
            f"{name}:{count}" for name, count in result["profiles"].items())
        print(f"profiles={profiles}")
    if result["corner_rows"] <= 0:
        print(f"hint={result['hint']}")
        return

    for key in (
        "max_corner_deg",
        "corner_event_count",
        "early_corner_rows",
        "speed_avg",
        "target_min",
        "target_avg",
        "lateral_target_avg",
        "lateral_limit_avg",
        "command_lateral_accel_avg",
        "command_lateral_accel_p95",
        "speed_over_target_avg",
        "speed_over_target_p95",
        "speed_over_target_max",
        "before_limit_over_target_avg",
        "limited_ratio",
        "floor_applied_ratio",
        "tracking_speed_floor_avg",
        "use_corner_speed_as_speed_target_ratio",
        "use_lateral_accel_speed_limit_ratio",
        "limit_command_speed_to_corner_ratio",
        "lateral_active_ratio",
        "steering_target_abs_p95",
        "steering_error_abs_avg",
        "steering_error_abs_p95",
        "steering_response_ratio_avg",
        "early_steering_response_ratio_avg",
        "early_steering_error_abs_p95",
        "early_steering_lag_ratio",
        "steering_response_delay_avg",
        "steering_response_delay_p95",
        "target_speed_last",
        "max_speed_last",
        "max_accel_last",
        "max_steering_rate_last",
        "tracking_min_command_speed_last",
        "iteration_count_last",
        "temperature_last",
        "heading_lookahead_weight_last",
        "steering_rate_weight_last",
        "steering_feedforward_command_gain_last",
        "selected_best_cost_gap_avg",
        "selected_best_cost_gap_p95",
        "rollouts_min",
        "rollouts_avg",
    ):
        value = result.get(key, math.nan)
        if key.endswith("_ratio"):
            print(f"{key}={value * 100.0:.1f}%")
        else:
            print(f"{key}={value:.3f}")

    hint = result["hint"]
    print(f"hint={hint}")
    if args.commands:
        command = command_for_hint(hint, args.node, result.get("lateral_limit_avg", math.nan))
        if command:
            print(f"command={command}")


def main():
    parser = argparse.ArgumentParser(
        description="Analyze an MPPI debug CSV log and suggest tuning changes.")
    parser.add_argument("csv_path", help="CSV file from mppi_log_debug.py")
    parser.add_argument(
        "--corner-threshold-deg",
        type=float,
        default=12.0,
        help="Minimum curvature heading change to count as a corner. Default: 12.0")
    parser.add_argument(
        "--node",
        default="/mppi_planner_node",
        help="Target MPPI node used in recommended commands. Default: /mppi_planner_node")
    parser.add_argument(
        "--commands",
        action="store_true",
        help="Print a recommended ros2 command for the analysis hint.")
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
        "--early-window-sec",
        type=float,
        default=1.0,
        help="Seconds after corner detection used for early steering response metrics. Default: 1.0")
    parser.add_argument(
        "--corner-event-gap-sec",
        type=float,
        default=0.5,
        help="Gap that starts a new corner event when analyzing steering delay. Default: 0.5")
    parser.add_argument(
        "--late-response-threshold",
        type=float,
        default=0.75,
        help="Steering response ratio below this is counted as late. Default: 0.75")
    args = parser.parse_args()

    path = Path(args.csv_path).expanduser()
    if not path.exists():
        print(f"CSV file not found: {path}", file=sys.stderr)
        return 1

    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))

    result = analyze(rows, args)
    print_result(result, args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
