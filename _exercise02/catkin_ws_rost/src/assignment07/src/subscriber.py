#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SteeringPWMCommand, SpeedPWMCommand

class BasicSubscriber:

    def __init__(self):
        rospy.init_node("steering_controller_pid")
        self.gps_sub = rospy.Subscriber("/communication/gps/3", Odometry, self.callback)

        rospy.spin()

    def callback(self, arg):
        print('callback', arg)

if __name__ == "__main__":
    BasicSubscriber()
