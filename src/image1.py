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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

  def detect_red(self, image):
    mask = cv2.inRange(image, (0, 0, 80), (20, 20, 255))
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_green(self, image):
    mask = cv2.inRange(image, (0, 80, 0), (20, 255, 20))
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def detect_blue(self, image):
    mask = cv2.inRange(image, (80, 0, 0), (255, 20, 20))
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  def pixel2meter(self, image):
    c_1 = self.detect_red(image)
    c_2 = self.detect_green(image)
    distance = np.sqrt(np.sum((c_1 - c_2) ** 2))
    return 3 / distance

  def find_angles(self, image):
    a = self.pixel2meter(image)
    c_b = a * self.detect_blue(image)
    c_g = a * self.detect_green(image)
    end_effector = a * self.detect_red(image)
    a_2 = np.arctan2(c_b[0] - c_g[0], c_b[1] - c_g[1])
    a_4 = np.arctan2(c_g[0] - end_effector[0], c_g[1] - end_effector[1]) - a_2
    return np.array([a_2, a_4])
  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    im1=cv2.imshow('window1', self.cv_image1)
    print(self.find_angles(self.cv_image1))
    cv2.waitKey(1)
    # Publish the results
    try:
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

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


