#!/usr/bin/env python3
import cv2
import numpy as np
from typing import Union

import time

class ObjectOrientationCalculator:
    def __init__(self):
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)
        self.mask_lower_bound = [0, 0, 0]
        self.mask_upper_bound = [255, 255, 255]

    def get_object_position(self, mask: np.ndarray, depth_image: np.ndarray):
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
        z = depth_image[cy, cx]
        print("depth of centroid:", z)
        return (cx, cy, z)

    def calculate_relative_orientation(self, object_position):
        direction_vector = np.subtract(object_position, self.robot_position)
        pitch = np.arctan2(direction_vector[2], direction_vector[1])
        yaw = np.arctan2(direction_vector[0], direction_vector[1])
        return pitch, yaw

    def process(self, color_image: Union[np.ndarray, None], depth_image: Union[np.ndarray, None]):
        if color_image is None or depth_image is None:
            return 0, 0

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
        cv2.imshow('colored', color_image)
        cv2.imshow('masked image', cv2.bitwise_and(color_image, color_image, mask = mask))
        if np.count_nonzero(mask) == 0:
            print("Empty mask; Sphero not found")
            return None

        object_position = self.get_object_position(mask, depth_image)
        if object_position is None:
            return 0, 0
        pitch, yaw = self.calculate_relative_orientation(object_position)
        # time.sleep(5)

        return pitch, yaw
