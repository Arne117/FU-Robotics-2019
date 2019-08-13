#/usr/bin/env python

# RAN SCAN SUBSCRIBER

import rospy
from autominy_msgs import Speed

def callback(raw_msg):
    echo raw_msg

# Initialize node
rospy.init_node("subscriber_node")

# Initialize subscriber
rospy.Subscriber("/sensors/speed", Speed, callback)

rospy.spin()
