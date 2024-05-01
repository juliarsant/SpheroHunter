import rospy
from SpheroHunter.msg import Tracker


class Brain:
    def __init__(self):
        rospy. Subscriber("/sphero/tracker", Tracker, self.tracker_callback)

    def tracker_callback(self, data):
        if data.found:
            print("Sphero Was Located")
        else:
            print("Sphero Not Located")

if __name__ == "__main__":
    rospy.init_node('sphero_location', anonymous=True)
    brain = Brain()
    rospy.spin()


"""
Step 1: Move to each quadrant until sphero is found
If sphero found, go to sphero until distance thershold

Step 2: Follow slow sphero

Step 3: Full hide and seek
"""

