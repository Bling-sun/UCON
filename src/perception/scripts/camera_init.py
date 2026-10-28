#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


# sys.path.insert(0, '/usr/lib/python3/dist-packages')
class D455Publisher(Node):
    def __init__(self):
        super().__init__('d455_camera_publisher')
        
        # Publishers
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 5)
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 5)
        
        # Initialize camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure depth and color streams
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start pipeline
        pipeline_profile = self.pipeline.start(self.config)
        
        # Align depth to color
        self.align = rs.align(rs.stream.color)
        
        # Get camera intrinsics
        # self.depth_intrinsics = pipeline_profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        # self.color_intrinsics = pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create camera info messages
        # self.depth_info_msg = self.create_camera_info(self.depth_intrinsics)
        # self.color_info_msg = self.create_camera_info(self.color_intrinsics)
        


    def create_camera_info(self, intrinsics):

        info_msg = CameraInfo()
        info_msg.height = intrinsics.height
        info_msg.width = intrinsics.width
        info_msg.distortion_model = "plumb_bob"
        info_msg.D = list(intrinsics.coeffs)
        info_msg.K = [
            intrinsics.fx, 0, intrinsics.ppx,
            0, intrinsics.fy, intrinsics.ppy,
            0, 0, 1
        ]
        info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.P = [
            intrinsics.fx, 0, intrinsics.ppx, 0,
            0, intrinsics.fy, intrinsics.ppy, 0,
            0, 0, 1, 0
        ]
        return info_msg

    def publish_images(self):
        try:
            while rclpy.ok():

                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                current_time = self.get_clock().now().to_msg()

                depth_image = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
                depth_msg.header.stamp = current_time
                depth_msg.header.frame_id = "camera_init"
                
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
                color_msg.header.stamp = current_time
                color_msg.header.frame_id = "camera_init"


                self.depth_pub.publish(depth_msg)
                self.color_pub.publish(color_msg)
                
                
        except Exception as e:
            self.get_logger().error(f"Camera error: {str(e)}")
        finally:
            self.pipeline.stop()

if __name__ == '__main__':
    rclpy.init()
    publisher = None
    try:
        publisher = D455Publisher()
        publisher.publish_images()
    except KeyboardInterrupt:
        pass
    finally:
        if publisher is not None:
            publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
