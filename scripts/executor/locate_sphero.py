#!/usr/bin/env python3
import rospy

# ROS Libraries
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from cv_bridge import CvBridge, CvBridgeError
from SpheroHunter.msg import Tracker
import time
import cv2
import numpy as np
import pyrealsense2
import tf2_ros
import tf2_geometry_msgs #his helps in the tf2 transform error and exception


from calculator import ObjectOrientationCalculator

class LocobotSpheroLocator:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.robot_position = (0, 0, 0)  # Default position
        self.sphero_located = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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

    def convert_point_to_real_position(self, point):
        x, y, depth = point
        x, y, depth = self.convert_depth_to_phys_coord_using_realsense(x, y, depth)
        transform = self.tf_buffer.lookup_transform("map", "locobot/camera_aligned_depth_to_color_frame", rospy.Time())

        # Transform the marker coordinates to arm_base_link frame
        point_in_camera = PointStamped()
        # point_in_camera.header = marker_msg.header
        # point_in_camera.point = Point(x=marker_msg.pose.position.x, y=marker_msg.pose.position.y, z=marker_msg.pose.position.z)
        point_in_camera.point = Point(x=point[0], y=point[1], z=point[2])
        # point_in_camera.header.frame_id = marker_msg.header.frame_id


        point_in_base = tf2_geometry_msgs.do_transform_point(point_in_camera, transform)

        return point_in_base.point

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
            print("result:", result)
            if result is not None:
                x, y, depth = result
                # img = cv2.circle(locator.color_image, (x, y), 3, (255,0,0), 4)
                # cv2.imshow("Sphero location", img)
                # cv2.waitKey(0)
                # break
                point = locator.convert_point_to_real_position(result)
                print("point:", point)
                # pitch, yaw = result
                msg = Tracker()
                pub.publish(msg)
                rate.sleep()
    
        # Get whether Sphero in image
            #Process sphero from saved image
            #Extract depth and location data from point cloud
        #Publish whether sphero in image, location of sphero in image

        
