#!/usr/bin/env python3

import rospy
import unittest
from std_msgs.msg import Float32

class TestCrossTrackError(unittest.TestCase):

    def setUp(self):
        # Initialize the ROS node
        rospy.init_node('test_cross_track_error', anonymous=True)
        self.cross_track_error = None

        # Subscriber to the cross_track_error topic
        rospy.Subscriber("/cross_track_error", Float32, self.callback)

        # Allow some time for the subscriber to connect
        rospy.sleep(1)

    def callback(self, msg):
        # Callback function to store the cross track error value
        self.cross_track_error = msg.data

    def test_cross_track_error(self):
        # Execute the polaris.py script
        rospy.loginfo("Starting polaris.py script")
        rospy.sleep(30)  # Wait for 30 seconds

        # Check if the cross_track_error has been updated
        self.assertIsNotNone(self.cross_track_error, "Did not receive any messages on /cross_track_error")

        # Assert that the cross track error is under 1.0
        rospy.loginfo("Evaluating cross track error")
        self.assertLess(self.cross_track_error, 1.0, "Cross track error is not under 1.0")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('your_package_name', 'test_cross_track_error', TestCrossTrackError)