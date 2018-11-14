#!/usr/bin/env python
import rospy
from roboteq_driver.srv import Feedback

def read_encoder_state(req):
    return FeedbackResponse(0.0)

def odom_forwarder:
    rospy.init_node('listener', anonymous=True)

    # Subscribe to feedback topics
    # rospy.Subscriber("/left/feedback", String, callback)
    # rospy.Subscriber("/right/feedback", String, callback)

    s = rospy.Service('read_encoder_state', Feedback, get_feedback)

    # Handle callbacks
    rospy.spin()

if __name__ == '__main__':
    odom_forwarder()