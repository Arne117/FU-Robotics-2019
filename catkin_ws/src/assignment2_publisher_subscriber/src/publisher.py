#/usr/bin/env python

# RAN SCAN PUBLISHER

import rospy
#from rplidar import ScannerDriver
from autominy_msgs import NormalizedSteeringCommand, SpeedCommand
#from sensor_msgs.msg import LaserScan

# Initialize node
rospy.init_node("publisher_node")

# Start driver
#driver ScannerDriver.load()

# Initialize publisher
steering_publisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand)
speed_publisher = rospy.Publisher("/actuators/speed", SpeedCommand)

while not rospy.is_shutdown():
    steering_msg = NormalizedSteeringCommand(1.0)
    speed_msg = SpeedCommand(0.3)
    steering_publisher.publish(steering_msg)
    speed_publisher.publish(speed_msg)
    rospy.sleep(0.01)  # sleep for 10 milliseconds  --> 100Hz
