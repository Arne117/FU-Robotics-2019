#!/usr/bin/env python

# RAN SCAN SUBSCRIBER

print("running subscriber")

import rospy
# from rplidar_decoder import PointCloudConverter

from autominy_msgs.msg import Speed
# import autominy_msgs
# from autominy_msgs import *

print(Speed)
print(autominy_msgs)

def callback(raw_msg):
    print(raw_msg)

# Initialize node
# rospy.init_node("name")

# Initialize subscriber

# rospy.Subscriber("/sensors/speed", Speed, callback)

# rospy.spin()



######################
# ROS Example
######################
# import rospy
# from std_msgs.msg import String
#
# def callback(data):
#     rospy.loginfo("I heard %s",data.data)
#
# def listener():
#     rospy.init_node('node_name')
#     rospy.Subscriber("chatter", String, callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
