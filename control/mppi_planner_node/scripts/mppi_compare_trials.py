#!/usr/bin/env python3
"""Compare multiple MPPI drive-trial CSV logs."""

import argparse
import csv
import math
import sys
from pathlib import Path
from types import SimpleNamespace

from mppi_analyze_log import analyze


BAD_HINT_PENALTY = {
    "corner_features_disabled_restart_or_apply_early": 8.0,
    "wait_for_corner": 6.0,
    "empty_log": 8.0,
    "optimizer_gap_increase_iterations_or_lower_temperature": 2.0,
    "tracking_floor_disabled_or_restart": 4.0,
    "steering_lag_try_aggressive_or_raise_steering_rate": 3.0,
    "try_aggressive_or_lower_corner_speed_margin": 2.0,
    "limit_active_raise_max_accel_or_lower_corner_speed_margin": 2.5,
    "lower_lateral_accel_limit_or_try_slow_corner": 2.0,
    "try_slow_corner_for_tighter_turns": 1.5,
    "too_slow_raise_limits_or_use_stable": 2.0,
}


def as_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def profile_label(result):
    profiles = result.get("profiles", {})
    if not profiles:
        return "unknown"
    if len(profiles) == 1:
        return next(iter(profiles.keys()))
    return "+".join(f"{name}:{count}" for name, count in profiles.items())


def score_result(result, objective):
    if result.get("corner_rows", 0) <= 0:
        return BAD_HINT_PENALTY.get(result.get("hint"), 6.0)

    speed_over = max(0.0, as_float(result.get("speed_over_target_p95"), 0.0))
    speed_under = max(0.0, -as_float(result.get("speed_over_target_avg"), 0.0) - 0.12)
    steering_error = as_float(result.get("steering_error_abs_p95"), 0.0)
    steering_response = as_float(result.get("steering_response_ratio_avg"), 1.0)
    steering_response_error = abs(steering_response - 1.0)
    early_lag = max(0.0, as_float(result.get("early_steering_lag_ratio"), 0.0))
    response_delay = max(0.0, as_float(result.get("steering_response_delay_p95"), 0.0))
    selected_best_gap = max(0.0, as_float(result.get("selected_best_cost_gap_p95"), 0.0))
    feature_gap = 1.0 - min(
        as_float(result.get("use_corner_speed_as_speed_target_ratio"), 1.0),
        as_float(result.get("use_lateral_accel_speed_limit_ratio"), 1.0),
        as_float(result.get("limit_command_speed_to_corner_ratio"), 1.0))
    rollout_gap = max(0.0, 12.0 - as_float(result.get("rollouts_min"), 12.0)) / 12.0
    hint_penalty = BAD_HINT_PENALTY.get(result.get("hint"), 0.0)

    if objective == "balanced":
        return (
            4.0 * speed_over +
            2.0 * speed_under +
            steering_response_error +
            early_lag +
            min(response_delay, 2.0) / 2.0 +
            steering_error / 25.0 +
            min(selected_best_gap, 5.0) / 5.0 +
            4.0 * feature_gap +
            rollout_gap +
            hint_penalty)

    return (
        5.0 * speed_over +
        1.0 * speed_under +
        0.7 * steering_response_error +
        1.5 * early_lag +
        0.6 * min(response_delay, 2.0) / 2.0 +
        steering_error / 40.0 +
        0.5 * min(selected_best_gap, 5.0) / 5.0 +
        4.0 * feature_gap +
        rollout_gap +
        hint_penalty)


def read_result(path, args):
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
    result["path"] = str(path)
    result["profile_label"] = profile_label(result)
    result["score"] = score_result(result, args.objective)
    return result


def fmt(value, width=6, precision=2):
    if not math.isfinite(as_float(value)):
        return f"{'nan':>{width}}"
    return f"{float(value):{width}.{precision}f}"


def print_table(results):
    header = (
        "rank score profile      corners speed_o95 opt_gap steer_e95 steer_resp early_lag "
        "ff_gain str_rate limited lat_act roll_min hint file")
    print(header)
    print("-" * len(header))
    for rank, result in enumerate(results, start=1):
        print(
            f"{rank:>4} "
            f"{fmt(result.get('score'), 5, 2)} "
            f"{result.get('profile_label', 'unknown'):<12.12} "
            f"{result.get('corner_rows', 0):>7} "
            f"{fmt(result.get('speed_over_target_p95'), 9, 2)} "
            f"{fmt(result.get('selected_best_cost_gap_p95'), 7, 2)} "
            f"{fmt(result.get('steering_error_abs_p95'), 9, 1)} "
            f"{fmt(as_float(result.get('steering_response_ratio_avg'), math.nan) * 100.0, 10, 0)}% "
            f"{fmt(as_float(result.get('early_steering_lag_ratio'), math.nan) * 100.0, 9, 0)}% "
            f"{fmt(result.get('steering_feedforward_command_gain_last'), 7, 2)} "
            f"{fmt(result.get('max_steering_rate_last'), 8, 2)} "
            f"{fmt(as_float(result.get('limited_ratio'), math.nan) * 100.0, 6, 0)}% "
            f"{fmt(as_float(result.get('lateral_active_ratio'), math.nan) * 100.0, 7, 0)}% "
            f"{fmt(result.get('rollouts_min'), 8, 0)} "
            f"{result.get('hint', ''):<45.45} "
            f"{Path(result['path']).name}")


def main():
    parser = argparse.ArgumentParser(
        description="Compare MPPI drive-trial CSV logs and rank them for tuning.")
    parser.add_argument("csv_paths", nargs="+", help="CSV files from mppi_drive_trial.py")
    parser.add_argument("--corner-threshold-deg", type=float, default=12.0)
    parser.add_argument("--steering-threshold-deg", type=float, default=8.0)
    parser.add_argument("--optimizer-gap-threshold", type=float, default=1.0)
    parser.add_argument("--early-window-sec", type=float, default=1.0)
    parser.add_argument("--corner-event-gap-sec", type=float, default=0.5)
    parser.add_argument("--late-response-threshold", type=float, default=0.75)
    parser.add_argument(
        "--objective",
        choices=("late_cornering", "balanced"),
        default="late_cornering",
        help="Ranking objective. Default: late_cornering.")
    args = parser.parse_args()

    results = []
    for csv_path in args.csv_paths:
        path = Path(csv_path).expanduser()
        if not path.exists():
            print(f"CSV not found: {path}", file=sys.stderr)
            return 1
        results.append(read_result(path, args))

    results.sort(key=lambda result: result["score"])
    print_table(results)
    if results:
        best = results[0]
        print(
            f"\nBest trial: {best['profile_label']} "
            f"score={best['score']:.2f} file={best['path']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
