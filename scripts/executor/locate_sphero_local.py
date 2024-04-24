#!/usr/bin/env python3
import numpy as np
import cv2
from sphero_locator import ObjectOrientationCalculator
import argparse
from typing import Union, Tuple, List
import re
import math

parser = argparse.ArgumentParser(
    prog="locate_sphero_local.py"
)

parser.add_argument(
    "--mask_lower",
    "-l",
    type=str,
    nargs="?",
    help="BGR color string defining the lower bound of the mask range",
)
parser.add_argument(
    "--mask_upper",
    "-u",
    type=str,
    nargs="?",
    help="BGR color string defining the upper bound of the mask range",
)

args = parser.parse_args()

if __name__ == "__main__":
    color_image = cv2.imread("img.jpg")
    depth_image = np.loadtxt("depth.txt")
    calculator = ObjectOrientationCalculator()
    mask_lower_bound = args.mask_lower
    mask_upper_bound = args.mask_upper
    
    if mask_lower_bound is not None:
        lower_bound: List[int] = [int(num) for num in re.split(r'\s*,\s*', mask_lower_bound.strip())]
        if len([n for n in lower_bound if math.isnan(n) or n < 0 or n > 255]) > 0:
            raise Exception(f"{mask_lower_bound} is not a valid color string")
        else:
            print("setting mask lower bound to", lower_bound)
            calculator.mask_lower_bound = lower_bound
            
    if mask_upper_bound is not None:
        upper_bound: List[int] = [int(num) for num in re.split(r'\s*,\s*', mask_upper_bound.strip())]
        if len([n for n in lower_bound if math.isnan(n) or n < 0 or n > 255]) > 0:
            raise Exception(f"{mask_upper_bound} is not a valid color string")
        else:
            print("setting mask upper bound to", upper_bound)
            calculator.mask_upper_bound = upper_bound
        
    result = calculator.process(color_image, depth_image)
    cv2.waitKey(0)
    if result is not None:
        pitch, yaw = result
        print("pitch:", pitch)
        print("yaw:", yaw)
    else:
        print("No Sphero found")