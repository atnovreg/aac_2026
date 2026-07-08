#!/usr/bin/env python3
"""Check that the running MPPI node publishes the expected RViz markers."""

import argparse
import sys
import time
from collections import Counter

import rclpy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


REQUIRED_NAMESPACES = (
    "mppi_rollouts",
    "mppi_best_sampled",
    "mppi_optimal",
    "mppi_diagnostics",
)


class MarkerChecker:
    def __init__(self, node, topic):
        self.node = node
        self.received = False
        self.namespaces = Counter()
        self.marker_count = 0
        self.line_point_max = Counter()
        self.diagnostic_texts = []
        self.frame_id = ""
        self.subscription = node.create_subscription(
            MarkerArray,
            topic,
            self.on_markers,
            10)

    def on_markers(self, msg):
        self.received = True
        self.marker_count = len(msg.markers)
        active_markers = [
            marker for marker in msg.markers
            if marker.action != Marker.DELETEALL]
        self.namespaces = Counter(marker.ns for marker in active_markers)
        self.line_point_max = Counter()
        self.diagnostic_texts = []
        self.frame_id = ""
        for marker in active_markers:
            if not self.frame_id and marker.header.frame_id:
                self.frame_id = marker.header.frame_id
            if marker.type == Marker.LINE_STRIP:
                self.line_point_max[marker.ns] = max(
                    self.line_point_max[marker.ns],
                    len(marker.points))
            if marker.ns == "mppi_diagnostics" and marker.text:
                self.diagnostic_texts.append(marker.text)


def required_namespaces(no_require_best):
    required = list(REQUIRED_NAMESPACES)
    if no_require_best:
        required.remove("mppi_best_sampled")
    return required


def visualization_issues(checker, args):
    required = required_namespaces(args.no_require_best)
    issues = []
    if not checker.received:
        issues.append(f"no MarkerArray received on {args.marker_topic}")
        return issues

    for namespace in required:
        if checker.namespaces.get(namespace, 0) <= 0:
            issues.append(f"missing marker namespace: {namespace}")

    rollout_count = checker.namespaces.get("mppi_rollouts", 0)
    if "mppi_rollouts" in required and rollout_count < args.min_rollouts:
        issues.append(
            f"mppi_rollouts: expected at least {args.min_rollouts}, got {rollout_count}")

    for namespace in ("mppi_rollouts", "mppi_best_sampled", "mppi_optimal"):
        if namespace not in required:
            continue
        point_count = checker.line_point_max.get(namespace, 0)
        if point_count < args.min_line_points:
            issues.append(
                f"{namespace}: expected a LINE_STRIP with at least "
                f"{args.min_line_points} points, got {point_count}")

    if (
        "mppi_diagnostics" in required and
        not args.no_require_diagnostic_text and
        not any("corner" in text and "steer target" in text for text in checker.diagnostic_texts)
    ):
        issues.append("mppi_diagnostics: missing corner/steering text marker")

    return issues


def main():
    parser = argparse.ArgumentParser(
        description="Verify MPPI rollout MarkerArray namespaces from the running node.")
    parser.add_argument(
        "--marker-topic",
        default="/mppi/trajectory_markers",
        help="MarkerArray topic. Default: /mppi/trajectory_markers")
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for markers. Default: 5.")
    parser.add_argument(
        "--no-require-best",
        action="store_true",
        help="Do not require the mppi_best_sampled namespace.")
    parser.add_argument(
        "--min-rollouts",
        type=int,
        default=1,
        help="Minimum mppi_rollouts markers required. Default: 1.")
    parser.add_argument(
        "--min-line-points",
        type=int,
        default=2,
        help="Minimum points required in rollout/best/optimal LINE_STRIP markers. Default: 2.")
    parser.add_argument(
        "--no-require-diagnostic-text",
        action="store_true",
        help="Do not require the mppi_diagnostics text marker.")
    args = parser.parse_args()

    rclpy.init(args=None)
    node = rclpy.create_node("mppi_visualization_check")
    checker = MarkerChecker(node, args.marker_topic)
    deadline = time.monotonic() + args.timeout

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if checker.received and not visualization_issues(checker, args):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    issues = visualization_issues(checker, args)

    print(f"marker_topic={args.marker_topic}")
    print(f"frame_id={checker.frame_id or 'unknown'}")
    print(f"marker_count={checker.marker_count}")
    print(f"namespaces={dict(sorted(checker.namespaces.items()))}")
    print(f"line_point_max={dict(sorted(checker.line_point_max.items()))}")
    if checker.diagnostic_texts:
        print(f"diagnostic_text={checker.diagnostic_texts[0]}")

    if issues:
        print("MPPI visualization check: FAIL", file=sys.stderr)
        for issue in issues:
            print(f"- {issue}", file=sys.stderr)
        print(
            "If the node was rebuilt recently, restart the MPPI launch pane.",
            file=sys.stderr)
        return 1

    print("MPPI visualization check: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
