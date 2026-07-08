#!/usr/bin/env python3
"""Run MPPI checks that should pass before driving."""

import argparse
import subprocess
import sys

from mppi_tune import PROFILES


def run_step(name, command, keep_going):
    print(f"== {name} ==", flush=True)
    completed = subprocess.run(command, check=False)
    if completed.returncode == 0:
        print(f"{name}: OK", flush=True)
        return True

    print(f"{name}: FAIL rc={completed.returncode}", file=sys.stderr, flush=True)
    if not keep_going:
        print("Stopping. Fix the failure above before driving.", file=sys.stderr)
    return False


def main():
    parser = argparse.ArgumentParser(
        description="Run runtime freshness and isolated corner probe checks before driving.")
    parser.add_argument(
        "--node",
        default="/mppi_planner_node",
        help="Running MPPI node checked by mppi_runtime_check.py.")
    parser.add_argument(
        "--skip-runtime-check",
        action="store_true",
        help="Skip checking the currently running MPPI process.")
    parser.add_argument(
        "--expect-live-profile",
        default="",
        help="Pass --expect-profile to mppi_runtime_check.py for the running node.")
    parser.add_argument(
        "--min-live-command-gain",
        type=float,
        default=0.0,
        help="Minimum live steering_feedforward_command_gain required by runtime check.")
    parser.add_argument(
        "--allow-disabled-corner-features",
        action="store_true",
        help="Allow baseline-like live nodes with corner features disabled.")
    parser.add_argument(
        "--skip-corner-probe",
        action="store_true",
        help="Skip the isolated synthetic corner probe.")
    parser.add_argument(
        "--check-visualization",
        action="store_true",
        help="Also check live /mppi/trajectory_markers namespaces from the running node.")
    parser.add_argument(
        "--marker-topic",
        default="/mppi/trajectory_markers",
        help="Marker topic used with --check-visualization. Default: /mppi/trajectory_markers.")
    parser.add_argument(
        "--profile",
        choices=["config"] + sorted(PROFILES.keys()),
        default="config",
        help="Apply this profile only to the isolated corner probe. Default: config.")
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="Run all checks even if an earlier check fails.")
    args = parser.parse_args()

    checks = []
    if not args.skip_runtime_check:
        runtime_command = [
            "ros2", "run", "mppi_planner_node", "mppi_runtime_check.py",
            "--node", args.node,
            "--min-steering-feedforward-command-gain",
            f"{args.min_live_command_gain:.2f}",
        ]
        if args.expect_live_profile:
            runtime_command.extend(["--expect-profile", args.expect_live_profile])
        if args.allow_disabled_corner_features:
            runtime_command.append("--allow-disabled-corner-features")
        checks.append((
            "runtime_check",
            runtime_command,
        ))
    if not args.skip_corner_probe:
        corner_probe_command = ["ros2", "run", "mppi_planner_node", "mppi_corner_probe.py"]
        if args.profile != "config":
            corner_probe_command.extend(["--profile", args.profile])
        checks.append((
            "corner_probe",
            corner_probe_command,
        ))
    if args.check_visualization:
        checks.append((
            "visualization_check",
            [
                "ros2", "run", "mppi_planner_node", "mppi_visualization_check.py",
                "--marker-topic", args.marker_topic,
            ],
        ))

    if not checks:
        print("No checks selected.")
        return 0

    failures = 0
    for name, command in checks:
        ok = run_step(name, command, args.keep_going)
        if not ok:
            failures += 1
            if not args.keep_going:
                break

    if failures:
        print(f"MPPI pre-drive check: FAIL ({failures} failed check(s))", file=sys.stderr)
        return 1

    print("MPPI pre-drive check: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
