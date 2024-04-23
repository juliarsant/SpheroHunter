#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

# ROS Libraries
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from SpheroHunter.msg import Tracker
import time

class ObjectOrientationCalculator:
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

    def get_object_position(self, mask):
        print("Mask (# nunzero pixels):")
        # np.set_printoptions(threshold=np.inf, linewidth=np.inf)
        print(np.count_nonzero(mask))
        # Assuming mask is a binary image of the segmented object
        moments = cv2.moments(mask)
        print("moments:")
        print(moments)
        if moments['m00'] == 0.0:
            print("NO MOMENTS FOUND")
            return None
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Get depth at the centroid
        z = self.depth_image[cy, cx]
        print("depth of centroid:", z)
        return (cx, cy, z)

    def calculate_relative_orientation(self, object_position):
        direction_vector = np.subtract(object_position, self.robot_position)
        pitch = np.arctan2(direction_vector[2], direction_vector[1])
        yaw = np.arctan2(direction_vector[0], direction_vector[1])
        return pitch, yaw

    def process(self):
        if self.color_image is None or self.depth_image is None:
            return 0, 0
        # print(self.color_image)

        print("depth image shape:")
        print(self.depth_image.shape)

        # cv2.imwrite("img.jpg", self.color_image)
        
        # np.savetxt("depth.txt", self.depth_image)

        # exit()
  
        # # Segment the object based on its color
        # lower_bound = np.array([255, 77, 0])  # Adjust these values
        # upper_bound = np.array([255, 196, 0])  # Adjust these values
        lower_bound = np.array([0, 0, 0])  # Adjust these values
        upper_bound = np.array([255, 255, 255])  # Adjust these values
        mask = cv2.inRange(self.color_image, lower_bound, upper_bound)
        # np.set_printoptions(threshold=np.inf, linewidth=np.inf)
        # print(mask)
        print("Mask shape:")
        print(mask.shape)
        print("Colored image shape:")
        print(self.color_image.shape)
        if np.count_nonzero(mask) == 0:
            print("Empty mask; Sphero not found")
            return None

        cv2.imshow('colored', self.color_image)
        cv2.imshow('mask', cv2.bitwise_and(self.color_image,self.color_image,mask = mask))

        return None
        object_position = self.get_object_position(mask)
        if object_position is None:
            return 0, 0
        pitch, yaw = self.calculate_relative_orientation(object_position)
        # time.sleep(5)

        return pitch, yaw

if __name__ == "__main__":
    rospy.init_node('sphero_location', anonymous=True)
    rospy.loginfo("Sphero locator has been started")
    pub = rospy.Publisher("/sphero/tracker", Tracker, queue_size=10)
    calculator = ObjectOrientationCalculator()
    rate = rospy.Rate(10)
    start_time = time.time()
    display = False
    while not rospy.is_shutdown():
        if time.time() - start_time > 5:
            display = True
        
        if display:
            result = calculator.process()
            if result is not None:
                pitch, yaw = result
                msg = Tracker()
                pub.publish(msg)
                rate.sleep()
    
        # Get whether Sphero in image
            #Process sphero from saved image
            #Extract depth and location data from point cloud
        #Publish whether sphero in image, location of sphero in image

        
