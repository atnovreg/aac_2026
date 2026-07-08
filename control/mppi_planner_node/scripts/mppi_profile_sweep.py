#!/usr/bin/env python3
"""Compare MPPI tuning profiles on isolated synthetic corners."""

import argparse
import json
import math
import subprocess
import sys

from mppi_tune import PROFILES


DEFAULT_PROFILES = ("early", "aggressive", "slow_corner", "stable", "baseline")
DEFAULT_CURVATURES = (0.10, 0.18, 0.28)


def parse_profiles(text):
    if text == "default":
        return list(DEFAULT_PROFILES)
    if text == "all":
        return sorted(PROFILES.keys())

    valid = set(PROFILES.keys()) | {"config"}
    profiles = [item.strip() for item in text.split(",") if item.strip()]
    invalid = [profile for profile in profiles if profile not in valid]
    if invalid:
        raise ValueError(
            f"unknown profile(s): {', '.join(invalid)}; valid={', '.join(sorted(valid))}")
    return profiles


def parse_floats(text, name):
    values = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            values.append(float(item))
        except ValueError as exc:
            raise ValueError(f"{name} contains non-float value: {item}") from exc
    if not values:
        raise ValueError(f"{name} must contain at least one value")
    return values


def extract_json_object(text):
    start = text.find("{")
    if start < 0:
        return None

    depth = 0
    in_string = False
    escape = False
    for index in range(start, len(text)):
        char = text[index]
        if in_string:
            if escape:
                escape = False
            elif char == "\\":
                escape = True
            elif char == '"':
                in_string = False
            continue

        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return text[start:index + 1]
    return None


def as_float(value, default=math.nan):
    try:
        if value is None:
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


def run_probe(args, profile, curvature):
    command = [
        "ros2", "run", "mppi_planner_node", "mppi_corner_probe.py",
        "--profile", profile,
        "--curvature", f"{curvature:.5f}",
        "--points", str(args.points),
        "--spacing", f"{args.spacing:.5f}",
        "--duration", f"{args.duration:.1f}",
        "--settle-time", f"{args.settle_time:.2f}",
        "--profile-timeout", f"{args.profile_timeout:.1f}",
        "--max-visualized-rollouts", str(args.max_visualized_rollouts),
    ]

    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=args.timeout)
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode(errors="replace")
        output += f"\nTimed out after {args.timeout:.1f}s"
        return 124, None, output

    json_text = extract_json_object(completed.stdout)
    summary = None
    if json_text:
        try:
            summary = json.loads(json_text)
        except json.JSONDecodeError:
            summary = None
    return completed.returncode, summary, completed.stdout


def summarize(profile, curvature, returncode, summary, output):
    row = {
        "profile": profile,
        "curvature": curvature,
        "returncode": returncode,
        "ok": returncode == 0 and summary is not None,
        "speed": math.nan,
        "steering_deg": math.nan,
        "corner_target": math.nan,
        "effective_target": math.nan,
        "lateral_target": math.nan,
        "lateral_accel": math.nan,
        "lookahead_m": math.nan,
        "steering_target_deg": math.nan,
        "steering_error_deg": math.nan,
        "steering_response": math.nan,
        "limited": False,
        "rollouts": 0,
        "status": "ok" if returncode == 0 and summary is not None else "fail",
    }
    if summary is None:
        lines = [line.strip() for line in output.splitlines() if line.strip()]
        if lines:
            row["status"] = lines[-1][:64]
        return row

    debug = summary.get("debug", {})
    steering = as_float(summary.get("steering"))
    steering_target = as_float(debug.get("steering_target_deg"))
    row.update({
        "speed": as_float(summary.get("speed")),
        "steering_deg": steering * 180.0 / math.pi if math.isfinite(steering) else math.nan,
        "corner_target": as_float(debug.get("corner_speed_target")),
        "effective_target": as_float(debug.get("effective_speed_target")),
        "lateral_target": as_float(debug.get("lateral_accel_speed_target")),
        "lateral_accel": as_float(debug.get("command_lateral_accel_estimate")),
        "lookahead_m": as_float(debug.get("curvature_lookahead_actual_distance")),
        "steering_target_deg": steering_target,
        "steering_error_deg": as_float(debug.get("steering_error_deg")),
        "limited": as_bool(debug.get("command_speed_limited", False)),
        "rollouts": int(as_float(debug.get("rollouts"), 0.0)),
    })
    if (
        math.isfinite(row["steering_deg"]) and
        math.isfinite(steering_target) and
        abs(steering_target) > 1.0
    ):
        row["steering_response"] = abs(row["steering_deg"]) / abs(steering_target)
    return row


def format_float(value, width, precision=2):
    if not math.isfinite(value):
        return f"{'nan':>{width}}"
    return f"{value:{width}.{precision}f}"


def finite(values):
    return [value for value in values if math.isfinite(value)]


def mean(values, default=math.nan):
    clean = finite(values)
    if not clean:
        return default
    return sum(clean) / len(clean)


def profile_recommendations(rows, objective):
    by_profile = {}
    for row in rows:
        if not row["ok"]:
            continue
        by_profile.setdefault(row["profile"], []).append(row)

    recommendations = []
    for profile, profile_rows in by_profile.items():
        speed_over_target = [
            row["speed"] - row["effective_target"] for row in profile_rows
        ]
        steering_response_error = [
            abs(row["steering_response"] - 1.0) for row in profile_rows
        ]
        steering_error_abs = [
            abs(row["steering_error_deg"]) for row in profile_rows
        ]
        targets = [row["effective_target"] for row in profile_rows]
        speeds = [row["speed"] for row in profile_rows]
        limited_ratio = sum(1 for row in profile_rows if row["limited"]) / len(profile_rows)
        stall_ratio = sum(
            1 for row in profile_rows
            if math.isfinite(row["speed"]) and row["speed"] < 0.10) / len(profile_rows)

        speed_over_avg = mean(speed_over_target, 0.0)
        speed_under_avg = mean(
            [
                max(0.0, row["effective_target"] - row["speed"])
                for row in profile_rows
            ],
            0.0)
        steering_response_error_avg = mean(steering_response_error, 1.0)
        steering_error_abs_avg = mean(steering_error_abs, 30.0)
        target_avg = mean(targets, 1.0)
        speed_avg = mean(speeds, 0.0)

        if objective == "balanced":
            score = (
                3.0 * max(0.0, speed_over_avg) +
                2.0 * max(0.0, speed_under_avg - 0.15) +
                2.5 * stall_ratio +
                0.8 * target_avg +
                steering_response_error_avg +
                steering_error_abs_avg / 30.0)
        else:
            score = (
                2.5 * target_avg +
                1.5 * max(0.0, speed_over_avg) +
                2.0 * max(0.0, speed_under_avg - 0.20) +
                2.5 * stall_ratio +
                0.5 * steering_response_error_avg +
                steering_error_abs_avg / 80.0)
        if profile in ("baseline", "config"):
            score += 2.0

        recommendations.append({
            "profile": profile,
            "score": score,
            "samples": len(profile_rows),
            "speed_avg": speed_avg,
            "speed_over_target_avg": speed_over_avg,
            "speed_under_target_avg": speed_under_avg,
            "target_avg": target_avg,
            "target_min": min(finite(targets), default=math.nan),
            "steering_response_error_avg": steering_response_error_avg,
            "steering_error_abs_avg": steering_error_abs_avg,
            "limited_ratio": limited_ratio,
            "stall_ratio": stall_ratio,
        })

    return sorted(recommendations, key=lambda item: item["score"])


def print_table(rows):
    header = (
        "profile      curv speed eff_t corner_t lat_t lat_a steer steer_t "
        "err resp lim roll status")
    print(header)
    print("-" * len(header))
    for row in rows:
        response = row["steering_response"] * 100.0
        print(
            f"{row['profile']:<11} "
            f"{row['curvature']:>4.2f} "
            f"{format_float(row['speed'], 5)} "
            f"{format_float(row['effective_target'], 5)} "
            f"{format_float(row['corner_target'], 8)} "
            f"{format_float(row['lateral_target'], 5)} "
            f"{format_float(row['lateral_accel'], 5, 2)} "
            f"{format_float(row['steering_deg'], 5, 1)} "
            f"{format_float(row['steering_target_deg'], 7, 1)} "
            f"{format_float(row['steering_error_deg'], 5, 1)} "
            f"{format_float(response, 4, 0)}% "
            f"{'Y' if row['limited'] else 'N':>3} "
            f"{row['rollouts']:>4} "
            f"{row['status']}")


def print_recommendation(recommendations, duration, objective):
    if not recommendations:
        print("\nRecommendation: no successful probes to score")
        return

    best = recommendations[0]
    print(f"\nRecommendation ({objective})")
    print(
        "  start_profile={profile} score={score:.2f} "
        "target_avg={target_avg:.2f}mps target_min={target_min:.2f}mps "
        "speed_avg={speed_avg:.2f}mps "
        "speed_over_target_avg={speed_over_target_avg:.2f}mps "
        "speed_under_target_avg={speed_under_target_avg:.2f}mps "
        "steer_response_error_avg={steering_response_error_avg:.2f} "
        "steer_error_abs_avg={steering_error_abs_avg:.1f}deg "
        "limited={limited:.0f}% stalled={stalled:.0f}%".format(
            limited=best["limited_ratio"] * 100.0,
            stalled=best["stall_ratio"] * 100.0,
            **best))
    print(
        "  command: ros2 run mppi_planner_node mppi_drive_trial.py "
        f"--profile {best['profile']} --duration {duration:.0f}")
    if len(recommendations) > 1:
        backup = recommendations[1]
        print(
            "  backup_profile={profile} score={score:.2f}".format(**backup))


def main():
    parser = argparse.ArgumentParser(
        description="Run isolated MPPI corner probes across profiles and curvatures.")
    parser.add_argument(
        "--profiles",
        default="default",
        help=(
            "Comma-separated profiles, 'default', or 'all'. "
            "Default set: early,aggressive,slow_corner,stable,baseline."))
    parser.add_argument(
        "--curvatures",
        default=",".join(str(value) for value in DEFAULT_CURVATURES),
        help="Comma-separated synthetic path curvatures. Default: 0.10,0.18,0.28")
    parser.add_argument("--points", type=int, default=32)
    parser.add_argument("--spacing", type=float, default=0.15)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--settle-time", type=float, default=0.5)
    parser.add_argument("--profile-timeout", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--retries",
        type=int,
        default=1,
        help="Retry each failed probe this many times. Default: 1.")
    parser.add_argument("--max-visualized-rollouts", type=int, default=12)
    parser.add_argument(
        "--trial-duration",
        type=float,
        default=60.0,
        help="Drive-trial duration printed in the recommended command. Default: 60.")
    parser.add_argument(
        "--objective",
        choices=("late_cornering", "balanced"),
        default="late_cornering",
        help="Scoring style for the recommendation. Default: late_cornering.")
    parser.add_argument(
        "--no-recommend",
        action="store_true",
        help="Do not print a profile recommendation below the table.")
    parser.add_argument("--json", action="store_true", help="Print JSON rows instead of a table.")
    args = parser.parse_args()

    try:
        profiles = parse_profiles(args.profiles)
        curvatures = parse_floats(args.curvatures, "--curvatures")
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 2

    rows = []
    for profile in profiles:
        for curvature in curvatures:
            print(
                f"probe profile={profile} curvature={curvature:.3f}",
                file=sys.stderr,
                flush=True)
            returncode = 1
            summary = None
            output = ""
            for attempt in range(max(0, args.retries) + 1):
                returncode, summary, output = run_probe(args, profile, curvature)
                if returncode == 0 and summary is not None:
                    break
                if attempt < max(0, args.retries):
                    print(
                        f"retry profile={profile} curvature={curvature:.3f} "
                        f"attempt={attempt + 2}",
                        file=sys.stderr,
                        flush=True)
            rows.append(summarize(profile, curvature, returncode, summary, output))

    if args.json:
        print(json.dumps(rows, indent=2, sort_keys=True))
    else:
        print_table(rows)
        if not args.no_recommend:
            print_recommendation(
                profile_recommendations(rows, args.objective),
                args.trial_duration,
                args.objective)

    failures = sum(1 for row in rows if not row["ok"])
    if failures:
        print(f"MPPI profile sweep: FAIL ({failures} probe(s) failed)", file=sys.stderr)
        return 1

    print("MPPI profile sweep: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
