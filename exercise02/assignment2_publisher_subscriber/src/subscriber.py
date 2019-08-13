#/usr/bin/env python

# RAN SCAN SUBSCRIBER

import rospy
#from rplidar_decoder import PointCloudConverter
from autominy_msgs import Speed

def callback(raw_msg):
    echo raw_msg
    # publisher.publish(point_cloud_msg)

# Initialize node
rospy.init_node("pointcloud_node")

# Initialize publisher and subscriber
# publisher = rospy.Publisher("/pointcloud", PointCloud)
# rospy.Subscriber("/scan", LaserScan, callback)

rospy.Subscriber("/sensors/speed", Speed, callback)

# /sensors/speed

rospy.spin()
