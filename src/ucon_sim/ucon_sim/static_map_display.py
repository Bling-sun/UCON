"""Color perceived static surfaces without drawing coincident point clouds."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


class StaticMapDisplay(Node):
    def __init__(self):
        super().__init__('static_map_display')
        self.resolution = self.declare_parameter('resolution', 0.1).value
        self.points = None
        self.groups = None
        self.occupied = set()
        self.last_data = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PointCloud2, '/ucon_sim/static_display', qos)
        self.map_sub = self.create_subscription(
            PointCloud2, '/map_generator/global_cloud', self.map_callback, 1)
        self.occupancy_sub = self.create_subscription(
            PointCloud2, '/grid_map/occupancy_inflate', self.occupancy_callback, 1)

    def cells(self, points):
        return np.floor(points / self.resolution).astype(np.int64)

    def map_callback(self, message):
        if message.header.frame_id != 'world':
            return
        points = point_cloud2.read_points_numpy(
            message, field_names=('x', 'y', 'z'), skip_nans=True)
        if self.points is not None and np.array_equal(self.points, points):
            return
        self.points = points.copy()
        # Group the simulation surface once so one detected part highlights
        # the complete connected obstacle instead of a patch of its voxels.
        pairs = cKDTree(self.points).query_pairs(0.24, output_type='ndarray')
        graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                           shape=(len(self.points), len(self.points)))
        _, self.groups = connected_components(graph, directed=False)
        self.publish_display()

    def occupancy_callback(self, message):
        if message.header.frame_id != 'world':
            return
        points = point_cloud2.read_points_numpy(
            message, field_names=('x', 'y', 'z'), skip_nans=True)
        cells = set(map(tuple, self.cells(points)))
        if cells == self.occupied:
            return
        self.occupied = cells
        self.publish_display()

    def publish_display(self):
        if self.points is None:
            return
        sensed = np.fromiter((tuple(cell) in self.occupied for cell in self.cells(self.points)),
                             dtype=bool, count=len(self.points))
        sensed = np.isin(self.groups, np.unique(self.groups[sensed]))
        data = np.empty(len(self.points), dtype=[('x', '<f4'), ('y', '<f4'),
                                                ('z', '<f4'), ('rgb', '<u4')])
        for index, axis in enumerate(('x', 'y', 'z')):
            data[axis] = self.points[:, index]
        # Mutually exclusive colors on the same geometry; no overlay surface.
        data['rgb'] = np.where(sensed, (121 << 16) | (183 << 8) | 177,
                              (178 << 16) | (178 << 8) | 206)
        payload = data.tobytes()
        if payload == self.last_data:
            return
        message = PointCloud2()
        message.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='world')
        message.height = 1
        message.width = len(data)
        message.fields = [PointField(name=axis, offset=index * 4,
                                     datatype=PointField.FLOAT32, count=1)
                          for index, axis in enumerate(('x', 'y', 'z'))]
        message.fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
        message.point_step = 16
        message.row_step = message.width * message.point_step
        message.is_dense = True
        message.data = payload
        self.publisher.publish(message)
        self.last_data = payload


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
