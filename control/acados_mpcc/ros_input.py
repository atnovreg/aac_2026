"""Convert map-frame base odometry into timestamped MPCC feedback."""
import math

import numpy as np
from scipy.spatial.transform import Rotation

from navigation.localization3d.estimator import mpcc_state
from .track import Track


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def check_time(stamp, now):
    # A replay clock tick may arrive just after the corresponding sensor sample.
    if stamp <= 0 or now <= 0 or not -50_000_000 <= now - stamp <= 500_000_000:
        raise ValueError("Missing, stale or future measurement time")


def xyz(vector):
    return np.array([vector.x, vector.y, vector.z])


class Feedback:
    def __init__(self, track, map_frame="indoor_map", base_frame="base_link"):
        self.track = track
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.state = None
        self.stamp = None
        self.clock = None

    def update(self, odom, now):
        if self.clock is not None and now < self.clock:
            self.state = self.stamp = None
        self.clock = now
        stamp = stamp_ns(odom.header)
        check_time(stamp, now)
        if self.stamp is not None and stamp <= self.stamp:
            raise ValueError("Duplicate or out-of-order odometry")
        if odom.header.frame_id != self.map_frame or odom.child_frame_id != self.base_frame:
            raise ValueError(f"Expected odometry from {self.map_frame} to {self.base_frame}")

        orientation = odom.pose.pose.orientation
        quat = np.r_[xyz(orientation), orientation.w]
        position = xyz(odom.pose.pose.position)
        linear = xyz(odom.twist.twist.linear)
        angular = xyz(odom.twist.twist.angular)
        if not np.isfinite(np.r_[position, quat, linear, angular]).all():
            raise ValueError("Non-finite odometry")
        # Same gross origin-reset limits used by the FAST-LIO map tracker.
        if np.linalg.norm(linear) > 25 or np.linalg.norm(angular) > 15:
            raise ValueError("Discontinuous base twist")
        if abs(np.linalg.norm(quat) - 1) > .01:
            raise ValueError("Invalid odometry quaternion")
        pose = np.eye(4)
        pose[:3, 3] = position
        pose[:3, :3] = rotation = Rotation.from_quat(quat).as_matrix()
        if np.linalg.norm(rotation[:2, 0]) < 1e-3:
            raise ValueError("Planar heading is undefined")
        previous = self.state
        fresh_history = previous is not None and stamp - self.stamp <= 500_000_000
        progress = self.track.project(position[:2], previous[6] if fresh_history else None)
        if previous is not None and not fresh_history:
            progress += self.track.length * round((previous[6] - progress) / self.track.length)
        state = mpcc_state(pose, rotation @ linear, angular, progress)
        if previous is not None:
            state[2] += math.tau * round((previous[2] - state[2]) / math.tau)
        if fresh_history:
            dt = (stamp - self.stamp) * 1e-9
            # Allow the map tracker's accepted registration correction per scan.
            if (np.linalg.norm(state[:2] - previous[:2]) > .8 + 25 * dt
                    or abs(state[2] - previous[2]) > .3 + 15 * dt):
                raise ValueError("Discontinuous map pose")
        if not np.isfinite(state).all():
            raise ValueError("Invalid projected MPCC state")
        self.stamp, self.state = stamp, state
        return state


def main():
    import rclpy
    from aac_msgs.msg import MpccStateStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = Node("mpcc_input")
    try:
        path = node.declare_parameter("track_csv", "").value
        if not path:
            raise ValueError("Set track_csv to the closed reference path in the odometry map")
        feedback = Feedback(Track.from_csv(path),
                            node.declare_parameter("map_frame", "indoor_map").value,
                            node.declare_parameter("base_frame", "base_link").value)
        publisher = node.create_publisher(MpccStateStamped, "/mpcc/state", 10)

        def receive(odom):
            try:
                state = feedback.update(odom, node.get_clock().now().nanoseconds)
            except ValueError as exc:
                node.get_logger().warning(str(exc), throttle_duration_sec=2.)
                return
            output = MpccStateStamped()
            output.header = odom.header
            output.state = state.tolist()
            publisher.publish(output)

        node.create_subscription(Odometry, "/localization/odom", receive, qos_profile_sensor_data)
        node.get_logger().info(f"Projecting /localization/odom onto {path} ({feedback.track.length:.3f} m)")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
