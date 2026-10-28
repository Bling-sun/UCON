#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OriginOdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        rate = self.declare_parameter('publish_rate', 50.0).value
        self.publisher = self.create_publisher(Odometry, '/Odometry', 10)
        self.timer = self.create_timer(1.0 / float(rate), self.publish_odom)

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        odom.twist.twist = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0),
        )
        diagonal = (0, 7, 14, 21, 28, 35)
        odom.pose.covariance = [0.01 if i in diagonal else 0.0 for i in range(36)]
        odom.twist.covariance = [0.01 if i in diagonal else 0.0 for i in range(36)]
        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OriginOdomPublisher()
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
