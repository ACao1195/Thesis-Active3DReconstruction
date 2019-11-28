import pyrealsense2 as rs
align = rs.align(rs.stream.color)
aligned_frames = align.proccess(depth_and_color_frameset)
color_frame = aligned_frames.first(rs.stream.color)
aligned_depth_frame = aligned_frames.get_depth_frame()

