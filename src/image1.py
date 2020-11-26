#!/usr/bin/env python3

import roslib
import sys
import time
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from std_msgs.msg import MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.start_time = time.time()
        self.gjoint2_pub = rospy.Publisher("joint2", Float64, queue_size=10)
        self.gjoint3_pub = rospy.Publisher("joint3", Float64, queue_size=10)
        self.gjoint4_pub = rospy.Publisher("joint4", Float64, queue_size=10)


        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        cv2.waitKey(10)
        self.y_cords = rospy.Subscriber("/ycor", Float64MultiArray, self.callback)

    def detect_red(self, image):
        mask = cv2.inRange(image, (0, 0, 80), (20, 20, 255))
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx,0.0, cy])

    def detect_green(self, image):
        mask = cv2.inRange(image, (0, 80, 0), (20, 255, 20))
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx,0.0, cy])

    def detect_blue(self, image):
        mask = cv2.inRange(image, (80, 0, 0), (255, 20, 20))
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx,0.0, cy])

    def detect_yellow(self, image):
        mask = cv2.inRange(image, (0, 200, 200), (0, 255, 255))
        M = cv2.moments(mask)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return np.array([cx,0.0, cy])

    def pixel2meter(self, image):
        c_1 = self.detect_yellow(image)
        c_2 = self.detect_blue(image)
        distance = np.sqrt(np.sum((c_1 - c_2) ** 2))
        return 2.5 / distance

    def find_angles(self, a,red,green,blue,yellow):
        red = red*a
        green = green*a
        blue = blue*a
        yellow = yellow*a
        joint2 = 0-(1.57+np.arctan2((green[2]-blue[2]), (green[0]-blue[0])))
        joint3 = (1.57+np.arctan2((green[2]-blue[2]), (green[1]-blue[1])))
        joint4 = np.arctan(((red[2]-green[2]) / ((np.sqrt(((red[0]-green[0])**2)+((red[1]-green[1])**2))))))
        return np.array([joint2, joint3, joint4])

    # Receive data from camera 1, process it, and publish

    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)
        im1 = cv2.imshow('window1', self.cv_image1)
        cv2.waitKey(1)
        # Publish the results
        joint2 = Float64()
        joint2.data = (np.pi / 2) * np.sin((np.pi / 15) * (time.time() - self.start_time))
        joint3 = Float64()
        joint3.data = (np.pi / 2) * np.sin((np.pi / 18) * (time.time() - self.start_time))
        joint4 = Float64()
        joint4.data = (np.pi / 2) * np.sin((np.pi / 20) * (time.time() - self.start_time))
        self.joint2_pub.publish(joint2)
        self.joint3_pub.publish(joint3)
        self.joint4_pub.publish(joint4)
        try:
            self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def callback(self, data):
        red = self.detect_red(self.cv_image1)
        green = self.detect_green(self.cv_image1)
        blue = self.detect_blue(self.cv_image1)
        yellow = self.detect_yellow(self.cv_image1)
        red[1] = data.data[0]
        green[1] = data.data[1]
        blue[1] = data.data[2]
        yellow[1] = data.data[3]
        a = self.pixel2meter(self.cv_image1)
        angles = self.find_angles(a,red,green,blue,yellow)
        gj_2 = Float64()
        gj_2.data = angles[0]
        gj_3 = Float64()
        gj_3.data = angles[1]
        gj_4 = Float64()
        gj_4.data = angles[2]
        self.gjoint2_pub.publish(gj_2)
        self.gjoint3_pub.publish(gj_3)
        self.gjoint4_pub.publish(gj_4)
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
    cv2.waitKey(10)
    main(sys.argv)
