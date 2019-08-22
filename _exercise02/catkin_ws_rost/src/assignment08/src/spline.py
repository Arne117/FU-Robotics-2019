#!/usr/bin/env python

import rospy
import sys
import time
import numpy as np
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from scipy import interpolate

class Spline:

    def __init__(self):
        # Subscriber
        self.gps_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_callback)

        # Publischer
        self.marker_pub = rospy.Publisher("/visualization_msgs/Marker", Marker, queue_size=1)
        self.point_clicked_pub = rospy.Publisher("/visualization_msgs/Marker/point_clicked", Marker, queue_size=1)
        self.point_closest_pub = rospy.Publisher("/visualization_msgs/Marker/point_closest", Marker, queue_size=1)

        self.marker = self.createMarker()
        self.point_clicked = self.createMarker()
        self.point_closest = self.createMarker()

        # Values
        self.innerlane = np.load('/home/arne/Documents/catkin_ws_rost/_exercise02/catkin_ws_rost/src/assignment08/src/lane1.npy')
        self.outerlane = np.load('/home/arne/Documents/catkin_ws_rost/_exercise02/catkin_ws_rost/src/assignment08/src/lane2.npy')
        self.supportVectors = []


        counter = 0
        while len(self.outerlane) > counter:
            if len(self.supportVectors) == 0:
                self.supportVectors.append(self.outerlane[counter])
            else:
                self.supportVectors.append(self.outerlane[counter])
            counter += int(len(self.outerlane)  / 20)

        self.supportVectors = np.array(self.supportVectors)

        arc = self.supportVectors[:,0]
        x = self.supportVectors[:,1]
        y = self.supportVectors[:,2]

        self.splineX = interpolate.CubicSpline(arc, x, axis=0, bc_type='not-a-knot', extrapolate=None)
        self.splineY = interpolate.CubicSpline(arc, y, axis=0, bc_type='not-a-knot', extrapolate=None)

        print(self.splineX(0.01), self.splineY(0.01))

        steps = np.arange(0.01, self.outerlane[-1][0], 0.01)
        splineX_samples = self.splineX(steps)
        splineY_samples = self.splineY(steps)

        marker = self.createMarker()

        i = 0
        while len(splineX_samples) > i:
            line_point = Point()
            line_point.x = splineX_samples[i]
            line_point.y = splineY_samples[i]
            line_point.z = 0.0
            marker.color.a = 1.0
            marker.points.append(line_point)
            i += 1

        self.marker = marker

    def createMarker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.color.a = 0.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.points = []
        return marker

    def clicked_point_callback(self, msg):
        marker = self.createMarker()
        marker.type = marker.SPHERE
        marker.color.g = 0.0
        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        self.point_clicked = marker

        # math.calculateDistance(msg.point.x, msg.point.y, binary.x, binary.y)



        marker_close = self.createMarker()
        marker_close.type = marker.SPHERE
        marker_close.color.r = 0.0
        marker_close.color.a = 1.0
        marker_close.scale.x = 0.05
        marker_close.scale.y = 0.05
        marker_close.scale.z = 0.05
        self.point_closest = marker_close

        # Startpoint
        arc = self.outerlane[int(len(self.outerlane) / 2)][0]
        newArc = arc
        step = 0.5
        dist = math.hypot(self.splineX(arc) - msg.point.x, self.splineY(arc) - msg.point.y)
        distanceChange = 100
        i = 0

        while distanceChange >= 0.0001:
            newArc = newArc + step
            newX = self.splineX(newArc)
            newY = self.splineY(newArc)

            self.point_closest.pose.position.x = newX
            self.point_closest.pose.position.y = newY

            newDist = math.hypot(newX - msg.point.x, newY - msg.point.y)
            stepChange = dist - newDist
            if stepChange < 0:
                step *= -1
                if i > 0:
                    step = step / 2
                print('swap')
            else:
                distanceChange = stepChange
            print(i, "step", step, " newArc: ", newArc, " newDist: ", newDist, " distanceChange: ", distanceChange)
            dist = newDist
            i += 1
            time.sleep(0.3)

            


def main(args):
  rospy.init_node('Spline')
  spline = Spline()

  while not rospy.is_shutdown():
      spline.marker_pub.publish(spline.marker)
      spline.point_clicked_pub.publish(spline.point_clicked)
      spline.point_closest_pub.publish(spline.point_closest)
      time.sleep(0.1)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
