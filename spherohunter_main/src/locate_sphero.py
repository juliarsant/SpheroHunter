#!/usr/bin/env python3
import rospy

# ROS Libraries
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from cv_bridge import CvBridge, CvBridgeError
from spherohunter_msgs.msg import Tracker
from typing import Union
import time
import cv2
import numpy as np
import pyrealsense2
import tf2_ros
import tf2_geometry_msgs #his helps in the tf2 transform error and exception
import argparse

'''
    Locate Sphero will publish the x, y, z coordinates and depth of sphero
    to /sphero/tracker. It uses calculator.py to find the pixel values
    in the camera frame and then it will transform those pixel values
    into the x,y,z locations within the map frame.
'''

#Arguments for whether to save camera feed from the Locobot
parser = argparse.ArgumentParser(
    prog="locate_sphero"
)

parser.add_argument(
    "--video",
    action="store_true",
    help="Whether to save a video of the Locobot's camera, with detected contours, to the file search.avi",
)

args = parser.parse_args()


from calculator import ObjectOrientationCalculator

class LocobotSpheroLocator:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)  # Default position
        self.sphero_located = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS subscribers
        rospy.Subscriber("/locobot/camera/color/image_raw", Image, self.color_callback)
        rospy.Subscriber("/locobot/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/locobot/pose", PoseStamped, self.pose_callback)

    def color_callback(self, data):
        #Gets feed from color camera callback and converts message to opencv image
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        #Gets feed from depth camera callback and converts message to opencv image
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    def pose_callback(self, data):
        #Gets orientation of the locobot
        self.robot_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)
    
    
    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth): 
        #Converts the pixel values to the x, y, z values in the camera frame 
        #https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a
        _intrinsics = pyrealsense2.intrinsics()

        #Hard coded values for realsense camera
        _intrinsics.width = 640
        _intrinsics.height = 480
        _intrinsics.ppx = 320.894287109375
        _intrinsics.ppy = 245.15847778320312
        _intrinsics.fx = 604.5020141601562
        _intrinsics.fy = 604.4981079101562
        _intrinsics.model  = pyrealsense2.distortion.none
        _intrinsics.coeffs = [i for i in [0.0, 0.0, 0.0, 0.0, 0.0]]
        result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth/1000)  
        return result[2], -result[0], -result[1]


    def convert_point_to_real_position(self, point):
        #Convert point to x, y, and depth from point for use in frame conversion
        x, y, depth, _= point
        pixel_x = x
        print(pixel_x)
        
        #Get coordinates from pixel values
        x, y, depth = self.convert_depth_to_phys_coord_using_realsense(x, y, depth)

        #Transform where sphero is in camera frame to where sphero is in map frame
        transform = self.tf_buffer.lookup_transform("map", "locobot/camera_aligned_depth_to_color_frame", rospy.Time())

        # Transform the sphero coordinates to map
        point_in_camera = PointStamped()
        point_in_camera.point = Point(x=x, y=y, z=depth)

        point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)

        return point_in_base.point, pixel_x

if __name__ == "__main__":
    rospy.init_node('sphero_location', anonymous=True)
    rospy.loginfo("Sphero locator has been started")

    #Publish messages to this topic
    pub = rospy.Publisher("/sphero/tracker", Tracker, queue_size=10)
    locator = LocobotSpheroLocator()
    rate = rospy.Rate(10)
    start_time = time.time()
    display = False

    #Initialize calculator which extracts sphero from camera feed
    calculator = ObjectOrientationCalculator(args.video)

    while not rospy.is_shutdown():
        #Can error out if camera not fully started
        if time.time() - start_time > 5:
            display = True
        
        if display:

            #Extract sphero information from camera feed
            result = calculator.process(
                color_image=locator.color_image,
                depth_image=locator.depth_image
            )
            print("result:", result)
            if result is not None:
                #Convert camera pixel points to map coordinates
                x, y, depth, found = result
                msg = Tracker()
                if found:
                    point, pixel_x = locator.convert_point_to_real_position(result)
                    
                    
                    #Create message to publish to topic
                    msg.x = point.x
                    msg.y = point.y
                    msg.depth = depth
                    msg.pix_x = pixel_x
                else:
                    msg.x = 0.0
                    msg.y= 0.0
                    msg.depth = 0.0
                    msg.pix_x = 0
                
                #Flag to indicate whether sphero found or not
                msg.found= found
                
                # Publish message
                pub.publish(msg)
                rate.sleep()

        
