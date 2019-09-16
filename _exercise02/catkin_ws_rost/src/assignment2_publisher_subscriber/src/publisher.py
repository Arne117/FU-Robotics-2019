#!/usr/bin/env python

# RAN SCAN PUBLISHER

import rospy
from rplidar import ObstacleDetector
from sensor_msgs.msg import LaserScan

# Initialize node
rospy.init_node("rplidar_driver_mode")

# Start driver
driver = ScannerDriver.load()

# Initialize publisher
rate = rospy.Rate(10) # 10 Hz
publisher = rospy.Publisher("/scan", LaserScan)

while not rospy.is_shutdown():
    msg = driver.getData()
    publisher.publish(msg)
    #rospy.sleep(0.5) # sleep a half second
    rate.sleep()
