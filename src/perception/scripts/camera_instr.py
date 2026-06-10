import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()

# Set resolution and frame rate (example: 640x480 @ 30Hz)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)

# Get intrinsics
depth_intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

print("depth_intrinsics: [fx, fy, cx, cy] =", 
      [depth_intr.fx, depth_intr.fy, depth_intr.ppx, depth_intr.ppy])
print("color_intrinsics: [fx, fy, cx, cy] =", 
      [color_intr.fx, color_intr.fy, color_intr.ppx, color_intr.ppy])

pipeline.stop()