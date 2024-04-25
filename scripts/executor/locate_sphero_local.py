#!/usr/bin/env python3
import numpy as np
import cv2
from sphero_locator import ObjectOrientationCalculator
from typing import Tuple, List
import re
import math
import argparse

def cs_str_to_hsv(hsv: str) -> Tuple[int, int, int]:
    h, s, v = [int(num) for num in re.split(r'\s*,\s*', hsv.strip())]
    if (math.isnan(h) or h < 0 or h > 179):
        raise Exception(f"{hsv} is not a valid HSV color string: {h} is not a valid hue (0-179)")
    if (math.isnan(s) or s < 0 or s > 255):
        raise Exception(f"{hsv} is not a valid HSV color string: {s} is not a valid saturation (0-255)")
    if (math.isnan(v) or v < 0 or v > 255):
        raise Exception(f"{hsv} is not a valid HSV color string: {v} is not a valid value (0-255)")
    return (h, s, v)

def cs_str_to_bgr(bgr: str) -> Tuple[int, int, int]:
    b, g, r = [int(num) for num in re.split(r'\s*,\s*', bgr.strip())]
    if len([n for n in (b, g, r) if math.isnan(n) or n < 0 or n > 255]) > 0:
        raise Exception(f"{bgr} is not a valid color string")
    return (b, g, r)

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
parser.add_argument(
    "--hsv_lower",
    type=str,
    nargs="?",
    help="Comma-separated HSV string for the mask range lower bound",
)
parser.add_argument(
    "--hsv_upper",
    type=str,
    nargs="?",
    help="Comma-separated HSV string for the mask range upper bound",
)

args = parser.parse_args()

if not (args.mask_lower or args.mask_upper):
    # Found these as good values (taken from https://stackoverflow.com/a/10951189/324399)
    args.hsv_lower = "5,50,50"
    args.hsv_upper = "15,255,255"

if (args.mask_lower or args.mask_upper) and (args.hsv_lower or args.hsv_upper):
    raise Exception("Cannot specify both BGR and HSV lower/upper mask colors")

color_image = cv2.imread("img.jpg")
depth_image = np.loadtxt("depth.txt")
calculator = ObjectOrientationCalculator()
if args.mask_lower or args.mask_upper:
    mask_lower_bound = args.mask_lower
    mask_upper_bound = args.mask_upper
    
    if mask_lower_bound is not None:
        color = cs_str_to_bgr(mask_lower_bound)
        print("setting BGR mask lower bound to", color)
        calculator.mask_lower_bound = color
            
    if mask_upper_bound is not None:
        color = cs_str_to_bgr(mask_upper_bound)
        print("setting BGR mask upper bound to", color)
        calculator.mask_upper_bound = color
        
elif args.hsv_lower or args.hsv_upper:
    calculator.use_hsv = True
    if args.hsv_lower is not None:
        color = cs_str_to_hsv(args.hsv_lower)
        print("setting HSV mask upper bound to", color)
        calculator.mask_lower_bound = color
        
    if args.hsv_upper is not None:
        color = cs_str_to_hsv(args.hsv_upper)
        print("setting HSV mask upper bound to", color)
        calculator.mask_upper_bound = color
    
    
result = calculator.process(color_image, depth_image)
cv2.waitKey(0)
if result is not None:
    pitch, yaw = result
    print("pitch:", pitch)
    print("yaw:", yaw)
else:
    print("No Sphero found")