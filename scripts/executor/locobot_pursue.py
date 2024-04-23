#!/usr/bin/env python3

import rospy
import math
from locobot_learning.srv import Approach
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib.simple_action_client import SimpleActionClient
from std_srvs.srv import Empty
import time

# This will be populated from the ROS parameter server
GOAL_LOCATIONS = {}

def initialize_ros_node():
    if not rospy.get_node_uri():
        rospy.init_node('pursue_server', anonymous=True)
    global GOAL_LOCATIONS
    try:
        # Retrieve the GOAL_LOCATIONS from the ROS parameter server
        GOAL_LOCATIONS = rospy.get_param("real_nav_goals")
    except rospy.ROSException as e:
        rospy.logerr("Could not retrieve GOAL_LOCATIONS parameter: %s", e)
        exit(1)

def sanitize_quaternion_for_2d_navigation(orientation):
    q_x = orientation.xlocobot_learning
def send_goal(goal_name: str):
    # Call the service to clear costmaps
    rospy.wait_for_service('/locobot/move_base/clear_costmaps')
    try:
        clear_costmaps = rospy.ServiceProxy('/locobot/move_base/clear_costmaps', Empty)
        clear_costmaps()
        rospy.loginfo("Cleared the cost maps")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
    client = SimpleActionClient('/locobot/move_base', MoveBaseAction)
    client.wait_for_server()

    pose_dict = GOAL_LOCATIONS[goal_name]
    rospy.loginfo("Sending goal: %s", pose_dict)

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position = Point(**pose_dict['position'])
    goal.target_pose.pose.orientation = sanitize_quaternion_for_2d_navigation(Quaternion(**pose_dict['orientation']))
    
    client.send_goal(goal)



    
    # # Wait for the result
    client.wait_for_result(timeout=rospy.Duration(30))

    
    # #Run robot for 30 seconds
    state = client.get_state()
    start_time = time.time()
    while state !=3 and time.time() - start_time < 30:
        if time.time() - start_time > 10:
            print("Cancelling goal!")
            client.cancel_goal()
            break
    
    state = client.get_state()
    print(f"State after exit {state}")

    
    if state == 3:  # SUCCEEDED
        rospy.loginfo("Goal reached!")
        return True, "Goal reached!"
    else:
        rospy.logerr("Failed to reach goal!")
        return False, "Failed to reach goal!"

def handle_approach(req):
    rospy.loginfo("Executing approach action towards %s", req.target)

    success, info = send_goal(req.target)

    return success, info

if __name__ == "__main__":
    initialize_ros_node()
    rospy.Service('pursue', Approach, handle_approach)
    rospy.spin()