#!/usr/bin/env python3
import cv2
import numpy as np
from typing import Union, Tuple
from visualization_msgs.msg import Marker
import rospy


MIN_CONTOUR_AREA = 200

class ObjectOrientationCalculator:
    def __init__(self):
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)
        self.mask_lower_bound: Tuple[int, int, int] = (10,50,200)
        self.mask_upper_bound: Tuple[int, int, int] = (20,255,255)
        self.use_hsv: bool = True

    def get_object_position(self, mask: np.ndarray, depth_image: np.ndarray):
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

    def refine_mask(self, mask: np.ndarray, color_image: np.ndarray) -> np.ndarray:
        """
        Filters all contours in the mask to only include the largest contour.
        """
        contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]
        if len(contours) == 0:
            return np.zeros_like(mask)
        print("number of contours:", len(contours))
        print("Contour areas:")
        print("\n\t".join([
            f"{i}: {cv2.contourArea(c)}" for i, c in enumerate(contours)
        ]))
        # c = cv2.drawContours(color_image, contours, -1, (0,255,0), 3)
        # cv2.imwrite("contours.jpg", cv2.cvtColor(c, cv2.COLOR_BGR2HSV))
        largestContour = max(contours, key=cv2.contourArea)
        mask = np.zeros_like(mask)
        cv2.drawContours(mask, [largestContour], 0, (255, 255, 255), cv2.FILLED)
        m = cv2.bitwise_and(color_image, color_image, mask = mask)
        m = cv2.drawContours(cv2.cvtColor(m, cv2.COLOR_HSV2BGR), contours, -1, (0,255,0), 3)
        cv2.imwrite("mask_contours.jpg", m)
        cv2.imwrite("img.jpg", cv2.cvtColor(color_image, cv2.COLOR_HSV2BGR))
        return mask

    def process(self, color_image: Union[np.ndarray, None], depth_image: Union[np.ndarray, None]):
        if color_image is None or depth_image is None:
            return 0, 0, 0
        
        # cv2.imwrite("no_sphero.jpg", color_image)
        
        # np.savetxt("depth.txt", self.depth_image)

        # exit()
        if self.use_hsv:
            color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

  
        # # Segment the object based on its color
        # lower_bound = np.array([255, 77, 0])  # Adjust these values
        # upper_bound = np.array([255, 196, 0])  # Adjust these values
        lower_bound = np.array(self.mask_lower_bound)  # Adjust these values
        upper_bound = np.array(self.mask_upper_bound)  # Adjust these values
        mask = cv2.inRange(color_image, lower_bound, upper_bound)
        mask = self.refine_mask(mask, color_image)
        
        # cv2.imwrite("masked.jpg", masked_image)
        # # cv2.imshow('colored', color_image)
        # # cv2.imshow('masked image', masked_image)
        # cv2.imshow('image', combined)
        if np.count_nonzero(mask) == 0:
            print("Empty mask; Sphero not found")
            return 0, 0, 0, False
        
        masked_image = cv2.bitwise_and(color_image, color_image, mask = mask)
        min_color = np.min(np.nonzero(masked_image))
        max_color = np.max(masked_image)
        print("min color:", min_color)
        print("max color:", max_color)
        # combined = np.hstack([color_image, masked_image])
        if self.use_hsv:
            masked_image = cv2.cvtColor(masked_image, cv2.COLOR_HSV2BGR)
            # combined = cv2.cvtColor(combined, cv2.COLOR_HSV2BGR)

        object_position = self.get_object_position(mask, depth_image)
        if object_position is None:
            return 0, 0, 0, False
        # pitch, yaw = self.calculate_relative_orientation(object_position)
        # time.sleep(5)
        # rospy.loginfo(marker_msg)
        x, y, z = object_position
        return x, y, z, True
    

    #publish message
    #rostopic list on sphero locator