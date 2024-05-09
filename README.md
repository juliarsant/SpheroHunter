# SpheroHunter
This project uses ROS Noetic and OpenCV on a Locobot to track a Sphero around a pre-built map. The Locobot uses a simple color range finder to locate orange within the camera frame in calculator.py. A mask is then used to extract the orange pixels from the frame and generate a centroid of the object.

The centroid is then transformed to the map frame in locate_sphero.py which is published to the topic /sphero/tracker. Brain_LOP.py listens to /sphero/tracker and uses the ROS MoveBase package to move towards the Sphero until it is within 0.6 meters of it. At this point the Locobot will wait until the Sphero is out of frame or at a distance greater than 0.6 meters until it moves again. If the Sphero exits to the right of the frame the Locobot will turn right otherwise it will turn left to try to relocate the Sphero.

During searching for the Sphero, when the Sphero is not within the camera frame the Locobot will first go to the last observed position it saw the Sphero at and spin around before going to the predefined positions of home, center1, and center2. The spin functionality is accomplished by publishing to the topic /mobile_base/cmd_vel with a ROS Twist message to tell the Locobot specific angular and linear velocity commands. These locations can see the full map and maximize the likelihood that the Sphero is found.



## Run Locobot
Ensure that you have ROS Noetic installed on the Locobot.

Install Dependencies:
```pip install numpy cv2 pyrealsense2 ```

Run the launch file:
```python tracker.launch```

This will launch the brain_LOP.py, locate_sphero.py and all background ROS nodes needed for the project.


## Run Sphero
Run Sphero:
You can either manually control the Sphero through the SpheroEdu app or run the sphero_back_and_forth.js script within the SpheroEdu app. sphero_back_and_forth.js was used for experimentation purposes and simply drives the Sphero back and forth in a line indefinitely.
