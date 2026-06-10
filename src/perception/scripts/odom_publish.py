#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
import tf2_ros
import geometry_msgs.msg

class OriginOdomPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher')

        self.frame_id = "/odom"
        self.child_frame_id = ""
        self.publish_rate = rospy.get_param("~publish_rate", 50)  # Hz

        self.odom_pub = rospy.Publisher('/Odometry', Odometry, queue_size=10)

        # self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.publish_loop()

    def publish_loop(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            odom_msg = self.create_odom_message(current_time)
            self.odom_pub.publish(odom_msg)
            # if self.enable_tf:
            #     self.publish_tf(current_time)
            rate.sleep()

    def create_odom_message(self, stamp):
        """Create odometry message at origin"""
        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        # Pose covariance (6x6 matrix)
        # Format: [x, y, z, rotation about X, rotation about Y, rotation about Z]
        odom.pose.covariance = [
            0.01, 0, 0, 0, 0, 0,    # X variance
            0, 0.01, 0, 0, 0, 0,    # Y variance
            0, 0, 0.01, 0, 0, 0,    # Z variance
            0, 0, 0, 0.01, 0, 0,    # rotation X variance
            0, 0, 0, 0, 0.01, 0,    # rotation Y variance
            0, 0, 0, 0, 0, 0.01     # rotation Z variance
        ]

        # Zero velocity
        odom.twist.twist = Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0)
        )

        # Twist covariance
        odom.twist.covariance = [
            0.01, 0, 0, 0, 0, 0,    # linear X variance
            0, 0.01, 0, 0, 0, 0,    # linear Y variance
            0, 0, 0.01, 0, 0, 0,    # linear Z variance
            0, 0, 0, 0.01, 0, 0,    # angular X variance
            0, 0, 0, 0, 0.01, 0,    # angular Y variance
            0, 0, 0, 0, 0, 0.01     # angular Z variance
        ]

        return odom

    def publish_tf(self, stamp):
        """Publish TF transform from odom to base_link"""
        transform = geometry_msgs.msg.TransformStamped()

        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.child_frame_id

        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        publisher = OriginOdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Origin odom publisher shutdown")
