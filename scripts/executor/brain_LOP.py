#!/usr/bin/env python3
import rospy
from SpheroHunter.msg import Tracker
from locobot_learning.srv import Approach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_srvs.srv import Empty
from actionlib.simple_action_client import SimpleActionClient
from collections import deque 
import time
import numpy as np
import sys
from enum import Enum
from cmd_vel_publisher import publish_velocity

class LocobotState(Enum):
    SEARCHING = 1
    PURSUING = 2


class Brain:
    def __init__(self):
        self.GOAL_LOCATIONS = {

            "home" : {
                "position" : {"x": -0.057, "y": -0.028, "z": 0.030},
                "orientation" : {"x": 0.0, "y": 0.0, "z": 0.0, "w":1.0}  },
            "center1" : {
                "position" : {"x": -0.830, "y": -0.058, "z": 0.024},
                "orientation" : {"x": -0.000, "y": 0.002, "z": -0.693, "w":0.721}  },
            "center2" : {
                "position" : {"x": -0.717, "y": -1.062, "z": 0.032},
                "orientation" : {"x": 0.001, "y": -0.006, "z": 0.104, "w": 0.995}  },
            "sphero": {
                "position" : {"x": -1.748, "y": -1.234, "z": 0.022},
                "orientation": {"x": -0.002, "y": -0.003, "z": -0.025, "w": 1.000}
            }
        }
        #Subscribe to sphero location info
        rospy. Subscriber("/sphero/tracker", Tracker, self.tracker_callback)
        self.sphero_located = False
        self.prev_sphero_located = False
        self.moving_to_sphero = False
        self.lop_behavior = False
        self.depth = 1000
        self.past_sphero = self.GOAL_LOCATIONS["sphero"]
        self.state = LocobotState.SEARCHING
        self.follow_dist = 675
        self.pix_x = 0
        self.spin_time = 7
        self.total_count= 0
        self.sphero_count = 0
        self.runtime = 60
        self.start_time = time.time()
        self.twist_interval = 0.1

        #Locations tracks the predefined locations in map to explore
        self.locations = deque()
        for key in self.GOAL_LOCATIONS:
                self.locations.append(key)


    #Callback for perception to determine if sphero in camera view
    def tracker_callback(self, data):
        self.total_count += 1
        #Update sphero location and flag based on found
        if data.found:
            self.sphero_count += 1
            self.depth = data.depth
            self.prev_sphero_located = self.sphero_located
            self.sphero_located = True
            self.state = LocobotState.PURSUING
            self.GOAL_LOCATIONS["sphero"] = {
                "position" : {"x": data.x, "y": data.y, "z": 0.0},
                #This needs to be the current orientation of the robot
                "orientation" : {"x": 0.0, "y": 0.0, "z": 0.0, "w":1.0}
            }
            self.pix_x = data.pix_x
            # print("Sphero Was Located")
        else:
            self.prev_sphero_located = self.sphero_located
            self.sphero_located = False
            self.depth = 1000
            # print("Sphero Not Located")
            self.state = LocobotState.SEARCHING
            if self.prev_sphero_located == True:
                print("LOP Triggered")
                self.lop_behavior = True

    def step1(self):
        #If sphero not found, go to locations
        while self.sphero_located == False:
            if time.time() - self.start_time > self.runtime:
                return
            if self.lop_behavior == True:
                self.lop_execute()
            self.moving_to_sphero = False
            #Go to first location in queue, add location back to end of queue for later
            location = self.locations.popleft()
            self.locations.append(location)

            #Move to desired location using service
            self.move(location)
            #Spin around for time period or until sphero found
            start_time = time.time()
            while time.time() - start_time < self.spin_time and self.sphero_located == False:
                publish_velocity(0, 0.75, self.twist_interval)

        if time.time() - self.start_time > self.runtime:
            return
        #Sphero is found, move there
        self.past_sphero = self.GOAL_LOCATIONS["sphero"]
        self.moving_to_sphero = True
        if self.sphero_located == True and self.prev_sphero_located == False:
            self.move("sphero")
    
    def lop_execute(self):
        start_time = time.time()
        while time.time() - start_time < self.spin_time and self.sphero_located == False:
            if self.pix_x < 320:
                publish_velocity(0, 0.75, self.twist_interval)
            else:
                publish_velocity(0, -0.75, self.twist_interval)
        print("Moving to LOP of sphero")
        self.move("sphero")
        self.lop_behavior = False

    def clear_costmaps(self):
        rospy.wait_for_service('/locobot/move_base/clear_costmaps')
        try:
            clear_costmaps = rospy.ServiceProxy('/locobot/move_base/clear_costmaps', Empty)
            clear_costmaps()
            # rospy.loginfo("Cleared the cost maps")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


    def move(self, goal_name: str):
        #Position and orientation of goal
        position = self.GOAL_LOCATIONS[goal_name]["position"]
        orientation = self.GOAL_LOCATIONS[goal_name]["orientation"]
    
            
        print(f"moving to: {goal_name}")


        # Call the service to clear costmaps
        self.clear_costmaps()
            
        client = SimpleActionClient('/locobot/move_base', MoveBaseAction)
        client.wait_for_server()

        pose_dict = self.GOAL_LOCATIONS[goal_name]
        # rospy.loginfo("Sending goal: %s", pose_dict)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        #Unpack dictionary and send to point and orientation
        goal.target_pose.pose.position = Point(**pose_dict['position'])
        goal.target_pose.pose.orientation = Quaternion(**pose_dict['orientation'])
        
        client.send_goal(goal)

        #Run robot for 20 seconds
        state = client.get_state()
        start_time = time.time()

        #Run until succeeded, break if over time or sphero located when searching for it
        while state !=3:
            if time.time() - self.start_time > self.runtime:
                print("Time limit met")
                client.cancel_goal()
                break
            distance = 0

            if self.sphero_located:
                past_position = self.past_sphero["position"]
                x_past = past_position["x"]
                y_past = past_position["y"]
                x = position["x"]
                y = position["y"]
                        #result[0]: right, result[1]: down, result[2]: forward
                distance = np.sqrt((x-x_past)**2+(y-y_past)**2)
                print(f"Distance change: {distance}")
                print(f"Depth to sphero {self.depth}")

            state = client.get_state()
            if (time.time() - start_time > 20 
                or distance > 0.5 
                or (self.moving_to_sphero == False and self.sphero_located == True)
                or self.depth < self.follow_dist):
                print("Cancelling goal!")
                # print("Value log")
                # print(f"Sphero distance {distance}")
                # print(f"Moving to sphero {self.moving_to_sphero}")
                # print(f"Sphero located {self.sphero_located}")
                # print(f"Depth {self.depth}")
                client.cancel_goal()
                break

if __name__ == "__main__":
    rospy.init_node('brain', anonymous=True)
    rospy.loginfo("Starting brain...")
    brain = Brain()
    start_time = time.time()
    while not rospy.is_shutdown() and time.time() - start_time < brain.runtime:
        #Run tracking sphero sitting in map
        brain.step1()
    
    print(brain.sphero_count/brain.total_count)
    



"""
Step 1: Move to each quadrant until sphero is found
If sphero found, go to sphero until distance thershold

Step 2: Follow slow sphero

Step 3: Full hide and seek
"""

