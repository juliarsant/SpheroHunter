#!/usr/bin/env python3
import cv2
import numpy as np
from typing import Union, Tuple
from visualization_msgs.msg import Marker
import rospy


MIN_CONTOUR_AREA = 200

'''
    Calculator will find the location of the Sphero object using a RGB range 
    from the pixels in the image and extract the pixels with a mask. It finds
    the centroid of the mask and returns the x, y and depth in the image.
'''

class ObjectOrientationCalculator:
    def __init__(self, record_video = False):
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)

        #RGB range for orange
        self.mask_lower_bound: Tuple[int, int, int] = (10,50,200)
        self.mask_upper_bound: Tuple[int, int, int] = (20,255,255)
        self.video_writer: Union[cv2.VideoWriter, None] = None

        #Record video of object detection
        if record_video:
            self.video_writer = cv2.VideoWriter("search.avi", cv2.VideoWriter_fourcc(*'XVID'), 30, (640, 480))

    #Returns x, y and depth of the object based on the moment data
    def get_object_position(self, mask: np.ndarray, depth_image: np.ndarray):
        # Assuming mask is a binary image of the segmented object
        moments = cv2.moments(mask)
        if moments['m00'] == 0.0:
            # print("NO MOMENTS FOUND")
            return None
        
        #Locate moments to extract x and y pos
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Get depth at the centroid
        z = depth_image[cy, cx]
        return (cx, cy, z)

    def refine_mask(self, mask: np.ndarray, color_image: np.ndarray) -> np.ndarray:
        """
        Filters all contours in the mask to only include the largest contour.
        """
        contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]

        #If no contours then return 0s
        if len(contours) == 0:
            if self.video_writer is not None:
                f = cv2.cvtColor(color_image, cv2.COLOR_HSV2BGR)
                self.video_writer.write(f)
            return np.zeros_like(mask)
        print("number of contours:", len(contours))
        print("Contour areas:")
        print("\n\t".join([
            f"{i}: {cv2.contourArea(c)}" for i, c in enumerate(contours)
        ]))

        #Grab largest contour to filter out small pixel values
        largestContour = max(contours, key=cv2.contourArea)
        mask = np.zeros_like(mask)

        #Record video of object detection
        if self.video_writer is not None:
            f = cv2.cvtColor(color_image, cv2.COLOR_HSV2BGR)
            f = cv2.drawContours(f, [largestContour], 0, (255, 0, 0), 3)
            self.video_writer.write(f)

        #Draw contours on saved image
        cv2.drawContours(mask, [largestContour], 0, (255, 255, 255), cv2.FILLED)
        m = cv2.bitwise_and(color_image, color_image, mask = mask)
        m = cv2.drawContours(cv2.cvtColor(m, cv2.COLOR_HSV2BGR), contours, -1, (0,255,0), 3)
        cv2.imwrite("mask_contours.jpg", m)
        cv2.imwrite("img.jpg", cv2.cvtColor(color_image, cv2.COLOR_HSV2BGR))
        return mask

    #Processes the mask data and extracts object position
    def process(self, color_image: Union[np.ndarray, None], depth_image: Union[np.ndarray, None]):
        if color_image is None or depth_image is None:
            return 0, 0, 0
        
        print("image shape:", color_image.shape)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

  
        # Segment the object based on its color
        lower_bound = np.array(self.mask_lower_bound)
        upper_bound = np.array(self.mask_upper_bound)

        #Returned mask extracting pixel values in range of mask with specified contours
        mask = cv2.inRange(color_image, lower_bound, upper_bound)
        mask = self.refine_mask(mask, color_image)
        
        #No sphero in frame
        if np.count_nonzero(mask) == 0:
            print("Empty mask; Sphero not found")
            return 0, 0, 0, False
        
        #Only extract relevant pixels from mask using bitwise_and operator
        masked_image = cv2.bitwise_and(color_image, color_image, mask = mask)
        min_color = np.min(np.nonzero(masked_image))
        max_color = np.max(masked_image)
        print("min color:", min_color)
        print("max color:", max_color)
        masked_image = cv2.cvtColor(masked_image, cv2.COLOR_HSV2BGR)

        #Extract object position from mask and depth image to return
        object_position = self.get_object_position(mask, depth_image)
        if object_position is None:
            return 0, 0, 0, False
        x, y, z = object_position
        return x, y, z, True