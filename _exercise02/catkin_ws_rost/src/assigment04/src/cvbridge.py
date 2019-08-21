#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('../package.xml')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy.linalg import norm

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("cvbridge",Image)

    self.bridge = CvBridge()
    # Aufgabe 3
    self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, self.callback)
    self.distanceThreshold = 15

  def getWhitePixels(self, img):
      counter = 0
      whitePixels = []
      for y, row in enumerate(img):
          for x, pixel in enumerate(row):
              if pixel[0] == 255 and pixel[1] == 255 and pixel[2] == 255:
                  # print(x, y)
                  whitePixels.append(np.array([x, y]))
                  # sumX += x
                  # sumY += y
                  counter += 1

      return whitePixels

  def ransec(self, whitePixels, img):
      print('------------------RANSEC-----------------')
      lines = []
      while len(whitePixels) >= 150:
          s = len(whitePixels)
          # whitePixels.pop(byIndex)
          # whitePixels.remove(byValue)
          samplesIndex = np.random.choice(s, 2)
          # print("-------")
          samples = [whitePixels[samplesIndex[0]], whitePixels[samplesIndex[1]]]
          # print("-------")

          inliers = []
          for i, pixel in enumerate(whitePixels):
              # dist = norm(np.cross(p2-p1, p1-p3))/norm(p2-p1)
              dist = norm(np.cross(samples[1]-samples[0], samples[0]-pixel)) / norm(samples[1]-samples[0])
              if dist <= self.distanceThreshold:
                  inliers.append(i)

          if len(inliers) >= s / 4:
              lines.append([samples[0], samples[1]])
              print('accepted')
              print('------------------s-----------------', s)
              # whitePixels = [ e for e in whitePixels if e not in inliers ]
              # whitePixels = np.delete(whitePixels, inliers, axis=1)
              length = len(inliers) - 1
              for i, j in enumerate(inliers):
                  # print(i, j)
                  del whitePixels[inliers[length-i]]
              print(len(whitePixels))
          else:
              print('failed')

          # whitePixels = [ e for e in whitePixels if e not in ('item', 5) ]
          # print(list(filter(lambda x: x < 0, whitePixels)))
      for line in lines:
          m =  line[1][1] - line[0][1] / line[1][0] - line[0][0]
          b = line[1][1] - m * line[1][0]
          # print('m = ', m)
          # print('b = ', b)
          print('y = ', m, 'x + ', b)
          cv2.line(img, (line[1][0], line[1][1]), (line[0][0], line[0][1]) ,(255,0,0), 2)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    # cv2.rectangle(cv_image, (0, 0), (640, 93), 10, -1, lineType=8, shift=0)
    # cv2.rectangle(cv_image, (0, 240), (640, 480), 10, -1, lineType=8, shift=0)

    croppedImg = cv_image[93:240, 0:640]

    # Aufgabe 3 Change image threshhold
    ret,thresh1 = cv2.threshold(croppedImg, 200, 255, cv2.THRESH_BINARY)

    # whitePixels = self.getWhitePixels(thresh1)
    #
    # self.ransec(whitePixels, thresh1)

    cv2.imshow("Image window", thresh1)
    cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
