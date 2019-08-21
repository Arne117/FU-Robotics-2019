#!/usr/bin/env python

import rospy
# import tf
import sys
# import autominy_msgs.msg
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SteeringPWMCommand, SpeedPWMCommand, Speed

# import scipy.integrate

# from numpy import exp
# f= lambda x:exp(-x**2)
# i = scipy.integrate.quad(f, 0, 1)

# Implement a PID control for the steering motor using the orientation from the localization system.
# Your PID controller should accept a desired yaw angle as an input so that it can be used in laterassignments.
# For testing let the car steer to a given constant orientation.
# At the start the carshould be facing at least 135 deg away from the desired orientation.
# All tests should be performed atlow (constant) speed (<0.5 m/s) to avoid crashes.
# Your car should drive at 0 or pi yaw dependingon the direction

class steering_controller_pid:

  def __init__(self):
    # self.steering_pub = rospy.Publisher("/actuators/steering_pwm", autominy_msgs/SteeringPWMCommand)
    # self.speed_pub = rospy.Publisher("/actuators/speed_pwm", autominy_msgs/SpeedPWMCommand)

    self.gps_sub = rospy.Subscriber("/communication/gps/3", Odometry, self.callback)
    self.speed_subscriber = rospy.Subscriber("/sensors/speed", Speed, self.on_speed, queue_size=10)

    # self.callback('test')
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

# winkel uebergeben
# winkel von gps ist quaternion
# setpoint winkel den wir uebergeben (wo wir hin wollen)
# Proscess winkel vom gps
# e = Fehlerwert / Differenz
# Dieser wird in allen berechnungen verwendet.

# PID Formel
# u(t) = Kp * e(t) + Ki * scipy.integrate.quad(e(t') * dt', 0, t) + Kd * (de(t) / dt)

  def on_speed(self, msg):
    print(msg.value)

  def calculate_pid(self, arg):
    Kp = 1
    Ki = 1
    Kd = 1

  def callback(self, arg):
    print('callback', arg)
    # pv =
    quaternion = (
        arg.pose.pose.orientation.x,
        arg.pose.pose.orientation.y,
        arg.pose.pose.orientation.z,
        arg.pose.pose.orientation.w
    )

    eueler = tf.transformation.euler_from_quaternion(quaternion)

    print(euler[2])


def main(args):
  print('hi')
  rospy.init_node('steering_controller_pid', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
