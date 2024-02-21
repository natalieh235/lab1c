import rclpy
import random
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Quaternion, TransformStamped

from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, TransformListener, TransformException
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_matrix, quaternion_from_matrix


def tf_to_se3(transform):
    q = transform.rotation
    q = [q.x, q.y, q.z, q.w]
    t = transform.translation
    mat = quaternion_matrix(q)
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    return mat


def se3_to_tf(mat, time, parent, child):
    obj = TransformStamped()
    obj.header.stamp = time.to_msg()
    obj.header.frame_id = parent
    obj.child_frame_id = child
    obj.transform.translation.x = mat[0, 3]
    obj.transform.translation.y = mat[1, 3]
    obj.transform.translation.z = mat[2, 3]
    q = quaternion_from_matrix(mat)
    obj.transform.rotation.x = q[0]
    obj.transform.rotation.y = q[1]
    obj.transform.rotation.z = q[2]
    obj.transform.rotation.w = q[3]
    return obj


class BaseLinkTfPublisher(Node):
    def __init__(self):
        super().__init__('base_link_pub')
        # self.subscription = self.create_subscription(Float32, 'base_link_gt', self.handle_pose, 10)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        # Call on_timer function every second
        self.timer = self.create_timer(0.05, self.on_timer)

    def on_timer(self):
        try:
            tf_world_to_left = self.tf_buffer.lookup_transform(
                "world",
                "left_cam",
                rclpy.time.Time())
        except TransformException as e:
            # self.get_logger().info('error getting robot position' + str(e))
            return

        # self.get_logger().info('got position left')
        base_to_left = np.array([
            [1, 0, 0, 0.05],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        world_to_left = tf_to_se3(tf_world_to_left.transform)
        world_to_base = world_to_left @ base_to_left

        tf_world_to_base = se3_to_tf(world_to_base, rclpy.time.Time(), parent='world', child='base_link_gt_2')
        self.tf_broadcaster.sendTransform(tf_world_to_base)


def main(args=None):
    rclpy.init(args=args)

    pub = BaseLinkTfPublisher()

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
