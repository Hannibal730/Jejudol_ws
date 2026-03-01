#!/usr/bin/env python3

import os

os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}] [{time}] {message}')

import rclpy

from MaRRTPathPlanNode import MaRRTPathPlanNode


def main(args=None):
    rclpy.init(args=args)
    node = MaRRTPathPlanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
