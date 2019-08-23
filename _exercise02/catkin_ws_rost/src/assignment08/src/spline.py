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
        self.outerlane_marker_pub = rospy.Publisher("/visualization_msgs/outerlane", Marker, queue_size=1)
        self.innerlane_marker_pub = rospy.Publisher("/visualization_msgs/innerlane", Marker, queue_size=1)
        self.point_clicked_pub = rospy.Publisher("/visualization_msgs/Marker/point_clicked", Marker, queue_size=1)
        self.point_closest_outer_pub = rospy.Publisher("/visualization_msgs/Marker/point_closest_outer", Marker, queue_size=1)
        self.point_closest_inner_pub = rospy.Publisher("/visualization_msgs/Marker/point_closest_inner", Marker, queue_size=1)
        self.lookahead_outer_pub = rospy.Publisher("/visualization_msgs/Marker/lookahead_outer", Marker, queue_size=1)
        self.lookahead_inner_pub = rospy.Publisher("/visualization_msgs/Marker/lookahead_inner", Marker, queue_size=1)

        self.marker = self.createMarker()
        self.point_clicked = self.createMarker()
        self.point_closest_outer = self.createMarker()
        self.point_closest_inner = self.createMarker()

        self.outerLookaheadMarker = self.createMarker()
        self.innerLookaheadMarker = self.createMarker()

        # Values
        self.innerlane = np.load('/home/arne/Documents/catkin_ws_rost/_exercise02/catkin_ws_rost/src/assignment08/src/lane1.npy')
        self.outerlane = np.load('/home/arne/Documents/catkin_ws_rost/_exercise02/catkin_ws_rost/src/assignment08/src/lane2.npy')

        self.outerSplineX, self.outerSplineY = self.calculatePoints(self.outerlane)
        self.innerSplineX, self.innerSplineY = self.calculatePoints(self.innerlane)

        self.outerlaneMarker = self.drawMarker(self.outerlane, self.outerSplineX, self.outerSplineY)
        self.innerlaneMarker = self.drawMarker(self.innerlane, self.innerSplineX, self.innerSplineY)

        self.lookaheadWidth = 0.3

        print('init done')

    def drawMarker(self, lane, splineX, splineY):
        steps = np.arange(0.01, lane[-1][0], 0.01)
        splineX_samples = splineX(steps)
        splineY_samples = splineY(steps)

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

        return marker

    def calculatePoints(self, lane):
        supportVectors = []
        counter = 0
        while len(lane) > counter:
            if len(supportVectors) == 0:
                supportVectors.append(lane[counter])
            else:
                supportVectors.append(lane[counter])
            counter += int(len(lane)  / 20)

        supportVectors = np.array(supportVectors)

        arc = supportVectors[:,0]
        x = supportVectors[:,1]
        y = supportVectors[:,2]

        return interpolate.CubicSpline(arc, x, axis=0, bc_type='not-a-knot', extrapolate=None), interpolate.CubicSpline(arc, y, axis=0, bc_type='not-a-knot', extrapolate=None)

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
        self.point_closest_outer = marker_close

        marker_close = self.createMarker()
        marker_close.type = marker.SPHERE
        marker_close.color.r = 0.0
        marker_close.color.a = 1.0
        marker_close.scale.x = 0.05
        marker_close.scale.y = 0.05
        marker_close.scale.z = 0.05
        self.point_closest_inner = marker_close

        outerArc, outerX, outerY = self.binarySearch(self.outerlane, self.outerSplineX, self.outerSplineY, msg, self.point_closest_outer)
        innerArc, innerX, innerY = self.binarySearch(self.innerlane, self.innerSplineX, self.innerSplineY, msg, self.point_closest_inner)

        self.drawLookAhead(outerArc, self.outerLookaheadMarker, self.outerSplineX, self.outerSplineY, self.lookaheadWidth)
        self.drawLookAhead(innerArc, self.innerLookaheadMarker, self.innerSplineX, self.innerSplineY, self.lookaheadWidth)

    def drawLookAhead(self, arc, marker, splineX, splineY, lookaheadWidth):
        lookaheadArc = arc + lookaheadWidth
        lookaheadX = splineX(lookaheadArc)
        lookaheadY = splineY(lookaheadArc)

        marker.type = marker.SPHERE
        marker.pose.position.x = lookaheadX
        marker.pose.position.y = lookaheadY
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        return lookaheadArc, lookaheadX, lookaheadY

    def binarySearch(self, lane, splineX, splineY, msg, marker):
        # Startpoint
        arc = lane[int(len(lane) / 2)][0]
        newArc = arc
        step = 0.5
        dist = math.hypot(splineX(arc) - msg.point.x, splineY(arc) - msg.point.y)
        distanceChange = 100
        i = 0
        newX = 0
        newY = 0

        while distanceChange >= 0.0001:
            newArc = newArc + step
            newX = splineX(newArc)
            newY = splineY(newArc)

            marker.pose.position.x = newX
            marker.pose.position.y = newY

            newDist = math.hypot(newX - msg.point.x, newY - msg.point.y)
            stepChange = dist - newDist
            if stepChange < 0:
                step *= -1
                if i > 0:
                    step = step / 2
                # print('swap')
            else:
                distanceChange = stepChange
            # print(i, "step", step, " newArc: ", newArc, " newDist: ", newDist, " distanceChange: ", distanceChange)
            dist = newDist
            i += 1
            # time.sleep(0.3)

        return newArc, newX, newY


def main(args):
  rospy.init_node('Spline')
  spline = Spline()

  while not rospy.is_shutdown():
      spline.outerlane_marker_pub.publish(spline.outerlaneMarker)
      spline.innerlane_marker_pub.publish(spline.innerlaneMarker)
      spline.point_clicked_pub.publish(spline.point_clicked)
      spline.point_closest_outer_pub.publish(spline.point_closest_outer)
      spline.point_closest_inner_pub.publish(spline.point_closest_inner)
      spline.lookahead_outer_pub.publish(spline.outerLookaheadMarker)
      spline.lookahead_inner_pub.publish(spline.innerLookaheadMarker)

      time.sleep(0.1)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
