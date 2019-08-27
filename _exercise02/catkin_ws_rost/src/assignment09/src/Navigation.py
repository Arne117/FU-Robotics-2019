#!/usr/bin/env python

import rospy
import sys
import time
import math
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf.transformations

# from SteeringControllerPid import SteeringControllerPid
# from Spline import Spline
from map import Lane, Map, MapVisualization
from pid import PID

class Navigation:

    def __init__(self):
        self.desired_angle_pub = rospy.Publisher("/control/desired_angle", Float32, queue_size=10)
        self.localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.on_localization, queue_size=1)
        # self.steering_pub = rospy.Publisher("/control/desired_angle", String, queue_size=10)
        self.map = Map()
        self.steeringAngle = 0.0
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.3), self.on_steering)

        print('init Navigation')

    def on_steering(self, msg):
        self.desired_angle_pub.publish(self.steeringAngle)

    def on_localization(self, msg):
        point = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        look1 = self.map.lanes[0].lookahead_point(point , 0.3)

        quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        roll, pitch, carYaw = tf.transformations.euler_from_quaternion(quat)
        print("carYaw: ", carYaw)

        # angle = math.acos(np.dot(point, look1[0]) / (abs(point) * abs(look1[0])))
        desiredAngle = self.angle_between(point, look1[0])
        print("desiredAngle: ", desiredAngle)
        self.steeringAngle = desiredAngle - carYaw
        print("steeringAngleTotal:", self.steeringAngle)

    def unit_vector(self, vector):
        return vector / np.linalg.norm(vector)

    def angle_between(self, v1, v2):
        v1_u = self.unit_vector(v1)
        v2_u = self.unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def main(args):
    rospy.init_node('Navigation')

    nav = Navigation()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
