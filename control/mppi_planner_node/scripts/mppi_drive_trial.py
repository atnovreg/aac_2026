#!/usr/bin/env python3
"""Run a bounded MPPI drive-tuning trial and analyze the debug log."""

import argparse
import subprocess
import sys
import time
from pathlib import Path

from mppi_tune import PROFILES


def default_output(profile):
    stamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = profile if profile != "keep" else "current"
    return f"/tmp/mppi_drive_trial_{suffix}_{stamp}.csv"


def run_step(name, command):
    print(f"== {name} ==", flush=True)
    print(" ".join(command), flush=True)
    completed = subprocess.run(command, check=False)
    if completed.returncode == 0:
        print(f"{name}: OK", flush=True)
    else:
        print(f"{name}: FAIL rc={completed.returncode}", file=sys.stderr, flush=True)
    return completed.returncode


def run_logger(command):
    print("== log_debug ==", flush=True)
    print(" ".join(command), flush=True)
    try:
        completed = subprocess.run(command, check=False)
        return completed.returncode
    except KeyboardInterrupt:
        print("log_debug: interrupted, analyzing captured rows.", flush=True)
        return 130


def command_gain_for_profile(profile):
    if profile == "keep":
        return 0.0
    try:
        return float(PROFILES.get(profile, {}).get("steering_feedforward_command_gain", 0.0))
    except (TypeError, ValueError):
        return 0.0


def runtime_check_command(args, profile=""):
    command = [
        "ros2", "run", "mppi_planner_node", "mppi_runtime_check.py",
        "--node", args.node,
    ]
    expected_profile = profile or args.expect_live_profile
    if expected_profile:
        command.extend(["--expect-profile", expected_profile])
    min_gain = args.min_live_command_gain
    if profile and profile != "keep":
        min_gain = max(min_gain, command_gain_for_profile(profile) - 1.0e-6)
    command.extend([
        "--min-steering-feedforward-command-gain",
        f"{min_gain:.2f}",
    ])
    if args.allow_disabled_corner_features or profile == "baseline" or (
        not profile and args.profile == "baseline"
    ):
        command.append("--allow-disabled-corner-features")
    return command


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Check MPPI freshness, optionally apply a profile, record debug_status "
            "during a drive trial, then analyze the CSV."))
    parser.add_argument(
        "--profile",
        choices=["keep"] + sorted(PROFILES.keys()),
        default="keep",
        help="Tuning profile to apply before logging. Default: keep current params.")
    parser.add_argument(
        "--node",
        default="/mppi_planner_node",
        help="Target MPPI node. Default: /mppi_planner_node")
    parser.add_argument(
        "--debug-topic",
        default="/mppi/debug_status",
        help="Debug status topic. Default: /mppi/debug_status")
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Logging duration in seconds. Default: 60.")
    parser.add_argument(
        "-o",
        "--output",
        default="",
        help="CSV output path. Default: /tmp/mppi_drive_trial_<profile>_<stamp>.csv")
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="Timeout for profile application. Default: 8.")
    parser.add_argument(
        "--corner-threshold-deg",
        type=float,
        default=12.0,
        help="Minimum heading change counted as a corner. Default: 12.")
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
        help="Gap that starts a new corner event in CSV analysis. Default: 0.5")
    parser.add_argument(
        "--late-response-threshold",
        type=float,
        default=0.75,
        help="Steering response ratio below this is counted as late. Default: 0.75")
    parser.add_argument(
        "--skip-runtime-check",
        action="store_true",
        help="Skip checking the currently running MPPI executable and parameters.")
    parser.add_argument(
        "--expect-live-profile",
        default="",
        help="Require this tuning_profile during the initial runtime check.")
    parser.add_argument(
        "--min-live-command-gain",
        type=float,
        default=0.0,
        help="Minimum live steering_feedforward_command_gain for runtime checks.")
    parser.add_argument(
        "--allow-disabled-corner-features",
        action="store_true",
        help="Allow baseline-like live nodes with corner speed features disabled.")
    parser.add_argument(
        "--skip-post-profile-check",
        action="store_true",
        help="Do not re-run runtime_check after applying --profile.")
    parser.add_argument(
        "--check-visualization",
        action="store_true",
        help="Verify live rollout MarkerArray contents before logging.")
    parser.add_argument(
        "--marker-topic",
        default="/mppi/trajectory_markers",
        help="Marker topic used with --check-visualization. Default: /mppi/trajectory_markers")
    parser.add_argument(
        "--visualization-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for rollout markers with --check-visualization. Default: 5.")
    parser.add_argument(
        "--skip-analyze",
        action="store_true",
        help="Only record the CSV; do not run mppi_analyze_log.py or mppi_suggest_tune.py afterward.")
    parser.add_argument(
        "--skip-suggest",
        action="store_true",
        help="Run mppi_analyze_log.py but skip mppi_suggest_tune.py.")
    parser.add_argument(
        "--assume-late",
        action="store_true",
        help="Tell mppi_suggest_tune.py to bias fallback advice toward earlier corner entry.")
    parser.add_argument(
        "--no-commands",
        action="store_true",
        help="Do not print recommended tuning commands during analysis.")
    args = parser.parse_args()

    output = Path(args.output or default_output(args.profile)).expanduser()

    if not args.skip_runtime_check:
        rc = run_step("runtime_check", runtime_check_command(args))
        if rc != 0:
            print("Runtime check failed. Restart the MPPI launch pane before a drive trial.", file=sys.stderr)
            return rc

    if args.profile != "keep":
        rc = run_step(
            "apply_profile",
            [
                "ros2", "run", "mppi_planner_node", "mppi_tune.py", args.profile,
                "--node", args.node,
                "--timeout", f"{args.timeout:.1f}",
            ])
        if rc != 0:
            return rc

        if not args.skip_runtime_check and not args.skip_post_profile_check:
            rc = run_step(
                "post_profile_runtime_check",
                runtime_check_command(args, args.profile))
            if rc != 0:
                print("Profile was applied but verification failed. Do not drive yet.", file=sys.stderr)
                return rc

    if args.check_visualization:
        rc = run_step(
            "visualization_check",
            [
                "ros2", "run", "mppi_planner_node", "mppi_visualization_check.py",
                "--marker-topic", args.marker_topic,
                "--timeout", f"{args.visualization_timeout:.1f}",
            ])
        if rc != 0:
            print(
                "Visualization check failed. Confirm local_path is publishing and RViz markers are live.",
                file=sys.stderr)
            return rc

    output.parent.mkdir(parents=True, exist_ok=True)
    logger_command = [
        "ros2", "run", "mppi_planner_node", "mppi_log_debug.py",
        "--debug-topic", args.debug_topic,
        "--duration", f"{args.duration:.1f}",
        "-o", str(output),
    ]
    rc = run_logger(logger_command)
    if rc not in (0, 130):
        return rc

    if args.skip_analyze:
        print(f"CSV saved: {output}")
        return 0

    analyze_command = [
        "ros2", "run", "mppi_planner_node", "mppi_analyze_log.py",
        str(output),
        "--node", args.node,
        "--corner-threshold-deg", f"{args.corner_threshold_deg:.1f}",
        "--steering-threshold-deg", f"{args.steering_threshold_deg:.1f}",
        "--optimizer-gap-threshold", f"{args.optimizer_gap_threshold:.2f}",
        "--early-window-sec", f"{args.early_window_sec:.2f}",
        "--corner-event-gap-sec", f"{args.corner_event_gap_sec:.2f}",
        "--late-response-threshold", f"{args.late_response_threshold:.2f}",
    ]
    if not args.no_commands:
        analyze_command.append("--commands")
    analyze_rc = run_step("analyze_log", analyze_command)

    suggest_rc = 0
    if not args.skip_suggest:
        suggest_command = [
            "ros2", "run", "mppi_planner_node", "mppi_suggest_tune.py",
            str(output),
            "--node", args.node,
            "--corner-threshold-deg", f"{args.corner_threshold_deg:.1f}",
            "--steering-threshold-deg", f"{args.steering_threshold_deg:.1f}",
            "--optimizer-gap-threshold", f"{args.optimizer_gap_threshold:.2f}",
            "--early-window-sec", f"{args.early_window_sec:.2f}",
            "--corner-event-gap-sec", f"{args.corner_event_gap_sec:.2f}",
            "--late-response-threshold", f"{args.late_response_threshold:.2f}",
            "--timeout", f"{args.timeout:.1f}",
        ]
        if args.assume_late:
            suggest_command.append("--assume-late")
        suggest_rc = run_step("suggest_tune", suggest_command)

    if analyze_rc != 0:
        return analyze_rc
    return suggest_rc


if __name__ == "__main__":
    sys.exit(main())
