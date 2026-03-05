import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

THROTTLE_BASE = 0.16
THROTTLE_AMPLITUDE = 0.10
THROTTLE_PERIOD_SEC = 12.0


class AutoThrottlePublisher(Node):

    def __init__(self):
        super().__init__('auto_throttle_publisher')
        self.publisher_ = self.create_publisher(Float32, '/auto_throttle', 10)
        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.publish_throttle)

    def publish_throttle(self):
        msg = Float32()
        t = time.time() - self.start_time
        omega = 2.0 * math.pi / THROTTLE_PERIOD_SEC
        cmd = THROTTLE_BASE + THROTTLE_AMPLITUDE * math.sin(t * omega)
        msg.data = max(0.0, min(1.0, cmd))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutoThrottlePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
