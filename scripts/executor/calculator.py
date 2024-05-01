#!/usr/bin/env python3
import cv2
import numpy as np
from typing import Union, Tuple
from visualization_msgs.msg import Marker
import rospy

class ObjectOrientationCalculator:
    def __init__(self):
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)
        self.mask_lower_bound: Tuple[int, int, int] = (0, 65, 166)
        self.mask_upper_bound: Tuple[int, int, int] = (40, 120, 219)
        self.use_hsv: bool = False

    def get_object_position(self, mask: np.ndarray, depth_image: np.ndarray):
        print("Mask (# nunzero pixels):")
        # np.set_printoptions(threshold=np.inf, linewidth=np.inf)
        print(np.count_nonzero(mask))
        # Assuming mask is a binary image of the segmented object
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
        return (cx, cy, z)

    def refine_mask(self, mask: np.ndarray) -> np.ndarray:
        """
        Filters all contours in the mask to only include the largest contour.
        """
        contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        largestContour = max(contours, key=cv2.contourArea)
        mask = np.zeros_like(mask)
        cv2.drawContours(mask, [largestContour], 0, (255, 255, 255), cv2.FILLED)
        return mask

    def process(self, color_image: Union[np.ndarray, None], depth_image: Union[np.ndarray, None]):
        if color_image is None or depth_image is None:
            return 0, 0, 0
        
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
        mask = self.refine_mask(mask)
        masked_image = cv2.bitwise_and(color_image, color_image, mask = mask)
        # combined = np.hstack([color_image, masked_image])
        if self.use_hsv:
            masked_image = cv2.cvtColor(masked_image, cv2.COLOR_HSV2BGR)
            # combined = cv2.cvtColor(combined, cv2.COLOR_HSV2BGR)
        
        cv2.imwrite("masked.jpg", masked_image)
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
        return object_position