#!/usr/bin/env python3
import cv2
import numpy as np
from typing import Union, Tuple
import pyrealsense2
from visualization_msgs.msg import Marker
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
import tf2_geometry_msgs #his helps in the tf2 transform error and exception
import rospy

class ObjectOrientationCalculator:
    def __init__(self):
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)
        self.mask_lower_bound: Tuple[int, int, int] = (0, 65, 166)
        self.mask_upper_bound: Tuple[int, int, int] = (40, 120, 219)
        self.use_hsv: bool = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth):  
        _intrinsics = pyrealsense2.intrinsics()
        # _intrinsics.width = cameraInfo.width
        # _intrinsics.height = cameraInfo.height
        # _intrinsics.ppx = cameraInfo.K[2]
        # _intrinsics.ppy = cameraInfo.K[5]
        # _intrinsics.fx = cameraInfo.K[0]
        # _intrinsics.fy = cameraInfo.K[4]

        _intrinsics.width = 640
        _intrinsics.height = 480
        _intrinsics.ppx = 320.894287109375
        _intrinsics.ppy = 245.15847778320312
        _intrinsics.fx = 604.5020141601562
        _intrinsics.fy = 604.4981079101562


        #_intrinsics.model = cameraInfo.distortion_model
        _intrinsics.model  = pyrealsense2.distortion.none
        _intrinsics.coeffs = [i for i in [0.0, 0.0, 0.0, 0.0, 0.0]]
        result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth/1000)  
        #result[0]: right, result[1]: down, result[2]: forward
        return result[2], -result[0], -result[1]

    def get_object_position(self, mask: np.ndarray, depth_image: np.ndarray):
        print("Mask (# nunzero pixels):")
        # np.set_printoptions(threshold=np.inf, linewidth=np.inf)
        print(np.count_nonzero(mask))
        # Assuming mask is a binary image of the segmented object
        # contours = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        # # print("Contours:")
        # # print(contours)
        # largestContour = max(contours, key=cv2.contourArea)
        # print("Largest contour:")
        # print(largestContour)
        moments = cv2.moments(mask)
        # print("moments:")
        # print(moments)
        if moments['m00'] == 0.0:
            # print("NO MOMENTS FOUND")
            return None
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Get depth at the centroid
        z = depth_image[cy, cx]
        # print("depth of centroid:", z)
        # print(f"x: {cx}    y: {cy}    z: {z}")
        newX, newY, newZ = self.convert_depth_to_phys_coord_using_realsense(cx, cy, z)
        # print(f"newx: {newX}    newy: {newY}    newz: {newZ}")
        return (newX, newY, newZ)

    def calculate_relative_orientation(self, object_position):
        direction_vector = np.subtract(object_position, self.robot_position)
        pitch = np.arctan2(direction_vector[2], direction_vector[1])
        yaw = np.arctan2(direction_vector[0], direction_vector[1])
        # print(f"Pitch: {pitch}")
        # print(f"Yaw: {yaw}")
        return pitch, yaw

    def process(self, color_image: Union[np.ndarray, None], depth_image: Union[np.ndarray, None]):
        if color_image is None or depth_image is None:
            return 0, 0
        
        if self.use_hsv:
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # cv2.imwrite("img.jpg", self.color_image)
        
        # np.savetxt("depth.txt", self.depth_image)

        # exit()
  
        # # Segment the object based on its color
        # lower_bound = np.array([255, 77, 0])  # Adjust these values
        # upper_bound = np.array([255, 196, 0])  # Adjust these values
        lower_bound = np.array(self.mask_lower_bound)  # Adjust these values
        upper_bound = np.array(self.mask_upper_bound)  # Adjust these values
        mask = cv2.inRange(color_image, lower_bound, upper_bound)
        # np.set_printoptions(threshold=np.inf, linewidth=np.inf)
        # masked_image = cv2.bitwise_and(color_image, color_image, mask = mask)
        # combined = np.hstack([color_image, masked_image])
        # if self.use_hsv:
        #     combined = cv2.cvtColor(combined, cv2.COLOR_HSV2BGR)
        # # cv2.imshow('colored', color_image)
        # # cv2.imshow('masked image', masked_image)
        # cv2.imshow('image', combined)
        if np.count_nonzero(mask) == 0:
            print("Empty mask; Sphero not found")
            return None

        object_position = self.get_object_position(mask, depth_image)
        if object_position is None:
            return 0, 0, 0
        # pitch, yaw = self.calculate_relative_orientation(object_position)
        # time.sleep(5)
        # rospy.loginfo(marker_msg)
        transform = self.tf_buffer.lookup_transform("map", "locobot/camera_aligned_depth_to_color_frame", rospy.Time())

        # Transform the marker coordinates to arm_base_link frame
        point_in_camera = PointStamped()
        # point_in_camera.header = marker_msg.header
        # point_in_camera.point = Point(x=marker_msg.pose.position.x, y=marker_msg.pose.position.y, z=marker_msg.pose.position.z)
        point_in_camera.point = Point(x=object_position[0], y=object_position[1], z=object_position[2])
        # point_in_camera.header.frame_id = marker_msg.header.frame_id


        point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)

        print("POINT TRANSFORMED", point_in_base.point)