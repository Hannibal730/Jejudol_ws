import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import time

STEER_MAX_DEG = 22.0
STEER_PERIOD_SEC = 6.0

class SteeringPublisher(Node):

    def __init__(self):
        super().__init__('steering_publisher')

        self.publisher_ = self.create_publisher(Float32, '/auto_steer_angle', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.start_time = time.time()

    def timer_callback(self):
        t = time.time() - self.start_time
        omega = 2.0 * math.pi / STEER_PERIOD_SEC

        msg = Float32()
        msg.data = STEER_MAX_DEG * math.sin(omega * t)  # ±22도

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
