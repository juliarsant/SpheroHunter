#!/usr/bin/env python3
import numpy as np
import cv2
from sphero_locator import ObjectOrientationCalculator

if __name__ == "__main__":
    color_image = cv2.imread("img.jpg")
    depth_image = np.loadtxt("depth.txt")
    calculator = ObjectOrientationCalculator()
    result = calculator.process(color_image, depth_image)
    cv2.waitKey(0)
    if result is not None:
        pitch, yaw = result
        print("pitch:", pitch)
        print("yaw:", yaw)
    else:
        print("No Sphero found")