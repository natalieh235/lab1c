import rclpy
import random
import math
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


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
        self.test_publisher = self.create_publisher(Float32, "range_test", 10)
        timer_period = 1/self.get_parameter('publish_rate').value  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_scan = rclpy.time.Time()


    def timer_callback(self):
        msg = LaserScan()
        cur_time = rclpy.time.Time()
        msg.header.stamp = cur_time.to_msg()
        msg.header.frame_id = "base_link"
        msg.angle_min = self.get_parameter('angle_min').value
        msg.angle_max = self.get_parameter('angle_max').value
        msg.angle_increment = self.get_parameter('angle_increment').value
        msg.range_min = self.get_parameter('range_min').value
        msg.range_max = self.get_parameter('range_max').value
        msg.scan_time = (cur_time - self.last_scan).nanoseconds / 10**-9
        self.last_scan = cur_time
        ranges_length = (msg.angle_max-msg.angle_min)//msg.angle_increment + 1
        msg.ranges = [random.uniform(msg.range_min, msg.range_max) for _ in range(int(ranges_length))]
        self.publisher_.publish(msg)

        test_msg = Float32()
        test_msg.data = float(len(msg.ranges))
        self.test_publisher.publish(test_msg)



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
