"""Console entry point for OpenCV localization."""

from opencv_node.ros_wrapper import OpenCVLocalizationNode

import rclpy


def main(args=None):
    """Run the OpenCV localization node."""
    rclpy.init(args=args)
    node = OpenCVLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
