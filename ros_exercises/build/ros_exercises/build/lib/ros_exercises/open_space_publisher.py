import rclpy
import random
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Float32
from open_space_message.msg import OpenSpace
from sensor_msgs.msg import LaserScan


class OpenSpacePublisher(Node):

    def __init__(self):
        super().__init__('open_space_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sub_topic', "fake_scan"),
                ('pub_topic', "open_space"),
            ]
        )
        self.subscriber = self.create_subscription(LaserScan, self.get_parameter('sub_topic').value, self.listener_callback, 10)
        self.open_space_publisher = self.create_publisher(OpenSpace, self.get_parameter('pub_topic').value, 10)


    def listener_callback(self, msg):
        # distance_msg = Float32()
        # distance_msg.data = max(msg.ranges)

        # self.distance_publisher.publish(distance_msg)

        index_max = np.argmax(msg.ranges)
        # angle_msg = Float32()
        # angle_msg.data = msg.angle_min + index_max * msg.angle_increment

        # self.angle_publisher.publish(angle_msg)
        msg = OpenSpace()
        msg.distance = max(msg.ranges)
        msg.angle = msg.angle_min + index_max * msg.angle_increment
        self.open_space_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    open_space_publisher = OpenSpacePublisher()

    rclpy.spin(open_space_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    open_space_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
