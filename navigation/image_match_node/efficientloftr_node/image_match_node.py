"""Console entry point for EfficientLoFTR localization."""

from efficientloftr_node.ros_wrapper import EfficientLoFTRLocalizationNode

import rclpy


def main(args=None):
    """Run the EfficientLoFTR localization node."""
    rclpy.init(args=args)
    node = EfficientLoFTRLocalizationNode()
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
