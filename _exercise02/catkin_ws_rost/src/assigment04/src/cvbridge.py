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

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("cvbridge",Image)

    self.bridge = CvBridge()
    # Aufgabe 3
    self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image,self.callback)
    # Starts with top left and goes to bottom right.
    self.realWorldObjectPoints = np.array([
        [1.1, 0.2, 0],
        [1.1, -0.2, 0],
        [0.8, 0.2, 0],
        [0.8, -0.2, 0],
        [0.5, 0.2, 0],
        [0.5, -0.2, 0],
    ], dtype = np.float32)
    self.cameraMatrix = np.matrix('383.7944641113281 0 322.3056945800781; 0 383.7944641113281 241.67051696777344; 0 0 1')
    self.distCoeffs = np.array([ 0.0, 0.0, 0.0, 0.0, 0.0 ])

  def getCenter(self, img):
      counter = 0
      sumX = 0
      sumY = 0
      for y, row in enumerate(img):
          for x, pixel in enumerate(row):
              if pixel[0] == 255 and pixel[1] == 255 and pixel[2] == 255:
                  # print(x, y)
                  sumX += x
                  sumY += y
                  counter += 1

      return [sumY / counter, sumX / counter]

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      # Aufgabe 3
      cv2.rectangle(cv_image, (0, 0), (640, 108), 10, -1, lineType=8, shift=0)
      cv2.rectangle(cv_image, (0, 247), (640, 480), 10, -1, lineType=8, shift=0)
      cv2.rectangle(cv_image, (0, 108), (180, 247), 10, -1, lineType=8, shift=0)

      # Aufgabe 4 Bounding Rect positions
      #11
      cv2.rectangle(cv_image, (255, 115), (280, 130), 10, thickness=1, lineType=8, shift=0)
      #12
      cv2.rectangle(cv_image, (408, 108), (428, 118), 10, thickness=1, lineType=8, shift=0)
      #21
      cv2.rectangle(cv_image, (238, 155), (256, 168), 10, thickness=1, lineType=8, shift=0)
      #22
      cv2.rectangle(cv_image, (433, 145), (458, 155), 10, thickness=1, lineType=8, shift=0)
      #31
      cv2.rectangle(cv_image, (195, 230), (220, 245), 10, thickness=1, lineType=8, shift=0)
      #32
      cv2.rectangle(cv_image, (488, 218), (520, 231), 10, thickness=1, lineType=8, shift=0)

      # Aufgabe 4
      subImg11Coordiantes = [115, 255]
      subImg12Coordiantes = [108, 408]
      subImg21Coordiantes = [155, 238]
      subImg22Coordiantes = [145, 433]
      subImg31Coordiantes = [230, 195]
      subImg32Coordiantes = [218, 488]

      subImageCoordiantes = [ subImg11Coordiantes, subImg12Coordiantes, subImg21Coordiantes, subImg22Coordiantes, subImg31Coordiantes, subImg32Coordiantes ]

    # Aufgabe 3 Change image threshhold
    ret,thresh1 = cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY)

    # Aufgabe 4 sub images ausschneiden
    subImg11 = thresh1[115:130, 255:280]
    subImg12 = thresh1[108:118, 408:428]
    subImg21 = thresh1[155:168, 238:256]
    subImg22 = thresh1[145:155, 433:458]
    subImg31 = thresh1[230:245, 195:220]
    subImg32 = thresh1[218:231, 488:520]

    subImages = [ subImg11, subImg12, subImg21, subImg22, subImg31, subImg32 ]

    #Aufgabe 4
    absoluteCenters = []
    for i, img in enumerate(subImages):
        imgCenter = self.getCenter(img)
        absoluteCenter = [subImageCoordiantes[i][1] + imgCenter[1], subImageCoordiantes[i][0] + imgCenter[0]]
        absoluteCenters.append(absoluteCenter)
        cv2.rectangle(cv_image, (absoluteCenter[0], absoluteCenter[1]), (absoluteCenter[0] + 1, absoluteCenter[1] + 1), 10, thickness=1, lineType=8, shift=0)

    print("----absoluteCenters-----")
    print(absoluteCenters)


    # Aufgabe 5
    retval, rvec, tvec = cv2.solvePnP(self.realWorldObjectPoints, np.array(absoluteCenters, dtype = np.float32), self.cameraMatrix, self.distCoeffs)
    print("----rvec-----")
    print(rvec)
    print("----tvec-----")
    print(tvec)

    print("----Rodrigues-----")
    print(cv2.Rodrigues(rvec))

    cv2.imshow("Image window", cv_image)
    # cv2.imshow("cropped", dot11)
    # cv2.imshow("cropped", dot12)
    # cv2.imshow("cropped", dot21)
    # cv2.imshow("cropped", dot22)
    # cv2.imshow("cropped", dot31)
    # cv2.imshow("cropped", dot32)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

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
