#!/usr/bin/env python3
import rospy

# ROS Libraries
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from SpheroHunter.msg import Tracker
import time
import cv2
import numpy as np

from sphero_locator import ObjectOrientationCalculator

class LocobotSpheroLocator:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)  # Default position
        self.sphero_located = False

        # ROS subscribers
        rospy.Subscriber("/locobot/camera/color/image_raw", Image, self.color_callback)
        rospy.Subscriber("/locobot/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/locobot/pose", PoseStamped, self.pose_callback)

    def color_callback(self, data):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    def pose_callback(self, data):
        self.robot_position = (data.pose.position.x, data.pose.position.y, data.pose.position.z)


if __name__ == "__main__":
    rospy.init_node('sphero_location', anonymous=True)
    rospy.loginfo("Sphero locator has been started")
    pub = rospy.Publisher("/sphero/tracker", Tracker, queue_size=10)
    locator = LocobotSpheroLocator()
    rate = rospy.Rate(10)
    start_time = time.time()
    display = False
    calculator = ObjectOrientationCalculator()
    calculator.mask_lower_bound = (5, 50, 50)
    calculator.mask_upper_bound = (15, 255, 255)
    calculator.use_hsv = True
    while not rospy.is_shutdown():
        if time.time() - start_time > 5:
            display = True
        
        if display:
            # if locator.color_image is not None:
            #     cv2.imwrite("img_sphero_offset.jpg", locator.color_image)
            # if locator.depth_image is not None:
            #     np.savetxt("img_sphero_offset_depth.txt", locator.depth_image)
            
            # exit()
            result = calculator.process(
                color_image=locator.color_image,
                depth_image=locator.depth_image
            )
            if result is not None:
                pitch, yaw = result
                msg = Tracker()
                pub.publish(msg)
                rate.sleep()
    
        # Get whether Sphero in image
            #Process sphero from saved image
            #Extract depth and location data from point cloud
        #Publish whether sphero in image, location of sphero in image

        
