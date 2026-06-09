"""Console entry point for MINIMA localization."""

from minima_node.ros_wrapper import MINIMALocalizationNode

import rclpy


def main(args=None):
    """Run the MINIMA localization node."""
    rclpy.init(args=args)
    node = MINIMALocalizationNode()
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
