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
        self.moving_to_sphero = False
        self.depth = 1000
        self.past_sphero = self.GOAL_LOCATIONS["sphero"]
        self.state = LocobotState.SEARCHING
        self.follow_dist = 675

        #Locations tracks the predefined locations in map to explore
        self.locations = deque()
        for key in self.GOAL_LOCATIONS:
                self.locations.append(key)


    #Callback for perception to determine if sphero in camera view
    def tracker_callback(self, data):
        #Update sphero location and flag based on found
        if data.found:
            self.depth = data.depth
            self.sphero_located = True
            self.state = LocobotState.PURSUING
            self.GOAL_LOCATIONS["sphero"] = {
                "position" : {"x": data.x, "y": data.y, "z": 0.0},
                #This needs to be the current orientation of the robot
                "orientation" : {"x": 0.0, "y": 0.0, "z": 0.0, "w":1.0}
            }
            print("Sphero Was Located")
        else:
            self.sphero_located = False
            self.depth = 1000
            # print("Sphero Not Located")
            self.state = LocobotState.SEARCHING

    def step1(self):
        #If sphero not found, go to locations
        while self.sphero_located == False:
            self.moving_to_sphero = False
            #Go to first location in queue, add location back to end of queue for later
            location = self.locations.popleft()
            self.locations.append(location)

            #Move to desired location using service
            self.move(location)
            #Spin around for time period or until sphero found
            start_time = time.time()
            print("Spinning")
            while time.time() - start_time < 10 and self.sphero_located == False:
                publish_velocity(0, 0.75, 0.5)

        #Sphero is found, move there
        self.past_sphero = self.GOAL_LOCATIONS["sphero"]
        self.moving_to_sphero = True
        self.move("sphero")

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
    
            
        print(f"Position to move: {position}")
        print(f"Orientation to move {orientation}")


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
            distance = 0

            if self.sphero_located:
                past_position = self.past_sphero["position"]
                x_past = past_position["x"]
                y_past = past_position["y"]
                x = position["x"]
                y = position["y"]
                distance = np.sqrt((x-x_past)**2+(y-y_past)**2)
                print(f"Distance change: {distance}")
                print(f"Depth to sphero {self.depth}")

            state = client.get_state()
            if (time.time() - start_time > 20 
                or distance > 0.5 
                or (self.moving_to_sphero == False and self.sphero_located == True)
                or self.depth < self.follow_dist):
                print("Cancelling goal!")
                print("Value log")
                print(f"Sphero distance {distance}")
                print(f"Moving to sphero {self.moving_to_sphero}")
                print(f"Sphero located {self.sphero_located}")
                print(f"Depth {self.depth}")
                client.cancel_goal()
                time.sleep(1)
                break

if __name__ == "__main__":
    rospy.init_node('brain', anonymous=True)
    rospy.loginfo("Starting brain...")
    brain = Brain()
    while not rospy.is_shutdown():
        #Run tracking sphero sitting in map
        brain.step1()


"""
Step 1: Move to each quadrant until sphero is found
If sphero found, go to sphero until distance thershold

Step 2: Follow slow sphero

Step 3: Full hide and seek
"""

