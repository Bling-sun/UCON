import math
import struct

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from traj_utils.msg import PolyTraj
from visualization_msgs.msg import Marker


class KinematicSimulator(Node):
    def __init__(self):
        super().__init__('ucon_kinematic_sim')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('speed', 0.35)
        self.declare_parameter('height', 1.5)
        self.declare_parameter('publish_environment_cloud', True)
        self.declare_parameter('follow_planning_trajectory', False)
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/cloud_registered_body', 10)
        self.marker_pub = self.create_publisher(Marker, '/ucon_sim/vehicle', 10)
        self.trajectory_sub = self.create_subscription(
            PolyTraj, '/planning/trajectory', self.trajectory_callback, 10)
        self.trajectory = None
        self.started = self.get_clock().now()
        rate = float(self.get_parameter('rate').value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info('ROS 2 kinematic simulation started')

    def trajectory_callback(self, msg):
        coefficient_count = len(msg.duration) * (int(msg.order) + 1)
        if msg.order != 5 or any(len(values) != coefficient_count for values in
                                 (msg.coef_x, msg.coef_y, msg.coef_z)):
            self.get_logger().error('Rejected malformed polynomial trajectory')
            return
        self.trajectory = msg
        self.get_logger().info(
            f'Following trajectory {msg.traj_id} with {len(msg.duration)} pieces')

    @staticmethod
    def evaluate_axis(coefficients, offset, t):
        # UCON stores [t^5, t^4, ..., t, 1] in each six-value block.
        c = coefficients[offset:offset + 6]
        position = sum(c[index] * t ** (5 - index) for index in range(6))
        velocity = sum((5 - index) * c[index] * t ** (4 - index) for index in range(5))
        return position, velocity

    def trajectory_state(self, now):
        msg = self.trajectory
        start_ns = msg.start_time.sec * 1_000_000_000 + msg.start_time.nanosec
        elapsed = max(0.0, (now.nanoseconds - start_ns) * 1e-9)
        piece = len(msg.duration) - 1
        local_t = elapsed
        for index, duration in enumerate(msg.duration):
            if local_t <= duration:
                piece = index
                break
            local_t -= duration
        local_t = min(local_t, float(msg.duration[piece]))
        offset = piece * 6
        x, vx = self.evaluate_axis(msg.coef_x, offset, local_t)
        y, vy = self.evaluate_axis(msg.coef_y, offset, local_t)
        z, vz = self.evaluate_axis(msg.coef_z, offset, local_t)
        return x, y, z, vx, vy, vz

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.started).nanoseconds * 1e-9
        follow_trajectory = bool(self.get_parameter('follow_planning_trajectory').value)
        if follow_trajectory and self.trajectory is not None:
            x, y, height, vx, vy, vz = self.trajectory_state(now)
            yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 1e-3 else 0.0
        elif follow_trajectory:
            x, y, height = 0.0, 0.0, float(self.get_parameter('height').value)
            vx = vy = vz = yaw = 0.0
        else:
            radius = float(self.get_parameter('radius').value)
            speed = float(self.get_parameter('speed').value)
            height = float(self.get_parameter('height').value)
            angle = speed * t
            x, y = radius * math.cos(angle), radius * math.sin(angle)
            vx, vy, vz = -radius * speed * math.sin(angle), radius * speed * math.cos(angle), 0.0
            yaw = angle + math.pi / 2.0

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = height
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        self.odom_pub.publish(odom)
        if bool(self.get_parameter('publish_environment_cloud').value):
            self.cloud_pub.publish(self.make_cloud(now))
        self.marker_pub.publish(self.make_marker(odom))

    @staticmethod
    def make_cloud(now):
        points = []
        for ix in range(-12, 13):
            points.append((ix * 0.25, 3.0, 0.2))
            points.append((ix * 0.25, -3.0, 0.2))
        data = b''.join(struct.pack('<fff', *point) for point in points)
        msg = PointCloud2()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'world'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = data
        return msg

    @staticmethod
    def make_marker(odom):
        marker = Marker()
        marker.header = odom.header
        marker.ns = 'ucon_sim'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = odom.pose.pose
        marker.scale.x, marker.scale.y, marker.scale.z = 0.45, 0.35, 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.7, 0.9, 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = KinematicSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
