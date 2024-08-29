#!/usr/bin/env python
import rospy
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

# Set to True if you want to see the images in a window
DEBUG_MODE = False
# Set to True if you want to decimate and align the images
DECIMATING_AND_ALIGNING = False
# Set to True if you want to apply the filters on the depth image
FILTERING = False
# if you want to change the exposure of the color image
desired_exposure = 85
# if you want to see the laser power
LASER_POWER = False

class simple_realsense_cam:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        # change presets
        depth_sensor = pipeline_profile.get_device().first_depth_sensor()
        # Set the visual preset option to 3 (High Accuracy), 4(high density) and 5 (medium density) to optimize for the environment
        depth_sensor.set_option(rs.option.visual_preset, 3)
        # Get the laser power value
        if LASER_POWER:
            laser_power = depth_sensor.get_option(rs.option.laser_power)
            print(f"Laser Power: {laser_power}")
        # fix the exposure value of the color image otherwsie the merge is a mess
        #color_sensor = pipeline_profile.get_device().query_sensors()[0]
        #color_sensor.set_option(rs.option.enable_auto_exposure, False)
        #color_sensor.set_option(rs.option.exposure, desired_exposure)
        
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.bridge = CvBridge()
        decimate_param = 1
        # Processing blocks
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.decimate = rs.decimation_filter()
        self.decimate.set_option(rs.option.filter_magnitude, 2 ** decimate_param) # decimmation filter
        self.threshold_filter = rs.threshold_filter()             # Threshold filter
        self.spatial_filter = rs.spatial_filter()                 # Spatial filter
        self.temporal_filter = rs.temporal_filter()               # Temporal filter
        # not used
        self.depth_to_disparity = rs.disparity_transform(True)    # Depth to disparity transform
        self.disparity_to_depth = rs.disparity_transform(False)   # Disparity to depth transform

    def get_frames(self):
        # Grab camera data
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        depth_frame_aligned = frames.get_depth_frame()
        color_frame_aligned = frames.get_color_frame()

        if DECIMATING_AND_ALIGNING:
            # Decimate the depth frame
            decimated_frame = self.decimate.process(depth_frame_aligned).as_frameset()
            # Align the depth frame to color frame
            aligned_frames = self.align.process(decimated_frame)
            depth_frame_aligned = aligned_frames.get_depth_frame()
            color_frame_aligned = aligned_frames.get_color_frame()

        # here I apply the other filters
        if FILTERING:
            depth_frame_aligned = self.threshold_filter.process(depth_frame_aligned)
            depth_frame_aligned = self.spatial_filter.process(depth_frame_aligned)
            depth_frame_aligned = self.temporal_filter.process(depth_frame_aligned)

        if not depth_frame_aligned or not color_frame_aligned:
            return None, None
        depth_image = np.asanyarray(depth_frame_aligned.get_data())
        color_image = np.asanyarray(color_frame_aligned.get_data())
        return color_image, depth_image

    def stop(self):
        self.pipeline.stop()

class RealSenseNode:
    def __init__(self):
        rospy.init_node('simple_realsense_node', anonymous=True)
        self.cam = simple_realsense_cam()
        self.depth_pub = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=10)
        self.rgb_pub = rospy.Publisher("/camera/rgb/image_color", Image, queue_size=10)
        self.rate = rospy.Rate(40)  # 10 Hz

    def start(self):
        print("Start pubblishing images")
        try:
            if DEBUG_MODE:
                cv2.namedWindow("RGB Image", cv2.WINDOW_AUTOSIZE)  # Create a window for display
                pass
            while not rospy.is_shutdown():
                rgb_image, depth_image = self.cam.get_frames()
                if rgb_image is not None and depth_image is not None:
                    try:
                        self.rgb_pub.publish(self.cam.bridge.cv2_to_imgmsg(rgb_image, "bgr8"))
                        self.depth_pub.publish(self.cam.bridge.cv2_to_imgmsg(depth_image, "16UC1"))
                        if DEBUG_MODE:
                            cv2.imshow("RGB Image", rgb_image)
                            if cv2.waitKey(1) & 0xFF == ord('q'):  # Wait for the 'q' key to quit
                                break
                            pass
                    except CvBridgeError as e:
                        rospy.logerr(e)
                self.rate.sleep()
        finally:
            self.cam.stop()

if __name__ == '__main__':
    node = RealSenseNode()
    node.start()
