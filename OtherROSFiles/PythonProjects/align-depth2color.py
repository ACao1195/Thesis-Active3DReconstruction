## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
from matplotlib.dates import strpdate2num

import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# Import pyarrow for parquet functionality (parquet is a file structure compatible with Matlab)
import pyarrow.parquet as pq

# Create a pipeline
pipeline = rs.pipeline()

#Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

# Streaming loop
try:
    while True:
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)





        # Render images

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))
        cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)


        # Do only once
        #break

        # Press esc or 'q' to close the image window and write the image to a file
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()

            # Instead, send image data to file
            # Depth_image contains depth data, bg_removed contains color image with areas of overlap removed
            depth_image1D = np.expand_dims(depth_image, axis=2)
            snapshotArray = np.concatenate((bg_removed, depth_image1D), axis=-1)

            # Could use a parquet object to transfer to matlab, probably too complicated
            #pq.write_table(snapshotArray, 'example.parquet')

            # Numpy savetext - delimits poorly, does not retain array
            #np.savetxt('test', snapshotArray, fmt='%uin')

            # OpenCV imwrite most effective, can write as 4D PNG

            # cv2.imwrite('test.png', snapshotArray, CV_IMWRITE_PNG_COMPRESSION=0)
            cv2.imwrite('test.png', snapshotArray, [cv2.IMWRITE_PNG_COMPRESSION, 0])

            break
finally:
    pipeline.stop()
