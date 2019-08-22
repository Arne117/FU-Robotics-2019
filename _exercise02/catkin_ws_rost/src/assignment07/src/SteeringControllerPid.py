#!/usr/bin/env python

import rospy
import sys
import time
import threading
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand

# PID Formel
# u(t) = Kp * e(t) + Ki * scipy.integrate.quad(e(t') * dt', 0, t) + Kd * (de(t) / dt)

class SteeringControllerPid:

    def __init__(self):
        # Subscriber
        self.gps_sub = rospy.Subscriber("/communication/gps/3", Odometry, self.setValues)

        # Publischer
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=1)
        self.steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=1)

        # Values
        self.targetAngle = 0.0
        self.currentTime = time.time()
        self.lastTime = self.currentTime
        self.lastError = 0.0
        self.Kp = 1
        self.Ki = 0.2
        self.Kd = 0.15
        self.P = 0
        self.I = 0
        self.D = 0
        self.maxI = 3
        self.yaw = 0
        self.speed = 0.5

        rospy.Timer(rospy.Duration(0.1), self.updateSteering)
        rospy.Timer(rospy.Duration(1), self.setSpeed)

    def setValues(self, arg):
        # print('-----------setValues--------------')
        # print(arg)
        quaternion = (
            arg.pose.pose.orientation.x,
            arg.pose.pose.orientation.y,
            arg.pose.pose.orientation.z,
            arg.pose.pose.orientation.w
        )
        (roll, pitch, self.yaw) = euler_from_quaternion(quaternion)
        # print('yaw: ', self.yaw)

    def updateSteering(self, event):
        print('-----------updateSteering--------------')
        error = self.targetAngle - self.yaw
        self.currentTime = time.time()
        deltaTime = self.currentTime - self.lastTime

        deltaError = error - self.lastError
        self.P = error
        self.I = self.I * 0.95 + error * deltaTime

        if self.maxI != 0:
            if (self.I < - self.maxI):
                self.I = - self.maxI
            elif (self.I > self.maxI):
                self.I = self.maxI

        self.D = 0.0
        if (deltaTime > 0):
            self.D = deltaError / deltaTime

        u = (self.Kp * self.P) + (self.Ki * self.I) + (self.Kd * self.D)
        print('P= ', self.P)
        print('I= ', self.I)
        print('D= ', self.D)
        print('u= ', u)

        self.lastError = error
        self.lastTime = time.time()

        self.setSteering(u)

    def setSteering(self, value):
        if value > 1:
            value = 1
        elif value < -1:
            value = -1
        steer_msg = NormalizedSteeringCommand()
        steer_msg.value = value
        self.steering_pub.publish(steer_msg)

    def setSpeed(self, event):
        speed_msg = SpeedCommand()
        speed_msg.value = self.speed
        self.speed_pub.publish(speed_msg)
        print("Set speed to:", self.speed)

def main(args):
  rospy.init_node('steering_controller_pid')
  SteeringControllerPid()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
