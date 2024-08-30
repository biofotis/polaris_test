#!/usr/bin/env python3

import rospy
import unittest
from std_msgs.msg import Float32

class TestCrossTrackError(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_cross_track_error', anonymous=True)
        self.cross_track_errors = []

        rospy.Subscriber("/cross_track_error", Float32, self.callback)
        rospy.sleep(1)

    def callback(self, msg):
        self.cross_track_errors.append(msg.data)

    def test_cross_track_error(self):
        start_time = rospy.Time.now()
        # Added a hardcoded value of 30 seconds here. I could use an arg in launch file to customize this later
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 30:
            rospy.sleep(1)
        self.assertGreater(len(self.cross_track_errors), 0, "Did not receive any messages on /cross_track_error")
        for error in self.cross_track_errors:
            self.assertLess(error, 1.0, f"Cross track error {error} is >= 1.0, test failed")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('test_cross_error', 'test_cross_track_error', TestCrossTrackError)