#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    self.y_coordinates = rospy.Publisher("/ycor",Float64MultiArray, queue_size = 1)
    self.target_coordinates = rospy.Publisher("/target_cords", Float64MultiArray, queue_size=1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.memory_red = [0, 0]
    self.memory_green = [0, 0]
    self.memory_blue = [0, 0]
    self.memory_yellow = [0, 0]
    self.circless = np.uint16(np.zeros((1,1,3)))

  def detect_red(self, image):
    mask = cv2.inRange(image, (0, 0, 80), (20, 20, 255))
    M = cv2.moments(mask)
    return int(M['m10'] / M['m00'])

  def detect_green(self, image):
    mask = cv2.inRange(image, (0, 80, 0), (20, 255, 20))
    M = cv2.moments(mask)
    return int(M['m10'] / M['m00'])

  def detect_blue(self, image):
    mask = cv2.inRange(image, (80, 0, 0), (255, 20, 20))
    M = cv2.moments(mask)
    return int(M['m10'] / M['m00'])
  def detect_yellow(self, image):
    mask = cv2.inRange(image, (0,200,200),(0,255,255))
    M = cv2.moments(mask)
    return int(M['m10'] / M['m00'])
  def detect_target(self, img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
    mask = cv2.inRange(hsv, (10, 100, 20), (25, 255, 255))
    gray_masked = cv2.bitwise_and(gray, gray, mask=mask)
    blurred = cv2.medianBlur(gray_masked, 5)
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1, 1000,
                               param1=50, param2=15, minRadius=0, maxRadius=100)

    yel = self.detect_yellow(img)
    yel = np.float64(yel)
    if circles is None:
      circles = self.circless
    else:
      circles = np.uint16(circles)
      self.circless = circles
    cords = Float64MultiArray()
    for i in circles[0, :]:
      cords.data = np.array([(i[0]-(yel)),i[1]])
    self.target_coordinates.publish(cords)

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', self.cv_image2)
    #im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)
    try:
      red = self.detect_red(self.cv_image2)
      self.memory_red = red
    except ZeroDivisionError:
      red = self.memory_red
    try:
      green = self.detect_green(self.cv_image2)
      self.memory_green = green
    except ZeroDivisionError:
      green = self.memory_green
    try:
      blue = self.detect_blue(self.cv_image2)
      self.memory_blue = blue
    except ZeroDivisionError:
      blue = self.memory_blue
    try:
      yellow = self.detect_yellow(self.cv_image2)
      self.memory_yellow = yellow
    except ZeroDivisionError:
      yellow = self.memory_yellow
    y = Float64MultiArray()
    y.data = np.array([red,green,blue,yellow])
    self.y_coordinates.publish(y)
    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)
    self.detect_target(self.cv_image2)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


