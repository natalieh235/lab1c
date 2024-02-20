import rclpy
import random
import math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class FakeScanPublisher(Node):

    def __init__(self):
        super().__init__('fake_scan_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('publish_topic', "fake_scan"),
                ('publish_rate', 20),
                ('angle_min', -2/3.0 * math.pi),
                ('angle_max', 2/3.0 * math.pi),
                ('angle_increment', 1.0/300*math.pi),
                ('range_min', 1.0),
                ('range_max', 10.0),
            ]
        )   

        self.publisher_ = self.create_publisher(LaserScan, self.get_parameter('publish_topic').value, 10)
        timer_period = 1/self.get_parameter('publish_rate').value  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min = self.get_parameter('angle_min').value
        msg.angle_max = self.get_parameter('angle_max').value
        msg.angle_increment = self.get_parameter('angle_increment').value
        msg.range_min = self.get_parameter('range_min').value
        msg.range_max = self.get_parameter('range_max').value

        ranges_length = (msg.angle_max-msg.angle_min)//msg.angle_increment + 1
        msg.ranges = [random.uniform(msg.range_min, msg.range_max) for _ in range(int(ranges_length))]
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.ranges)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    fake_scan_publisher = FakeScanPublisher()

    rclpy.spin(fake_scan_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_scan_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
