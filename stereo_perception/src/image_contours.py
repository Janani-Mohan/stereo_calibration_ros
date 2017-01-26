#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('stereo_perception')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    rospy.init_node('image_converter', anonymous=True)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/multisense_sl/camera/left/image_raw",Image,self.callback)

  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # patternSize = cv2.Size(9,7)
    # detected_corners = cv2.findChessboardCorners(cv_image,patternSize , flags=CV_CALIB_CB_ADAPTIVE_THRESH)
    # cv2.drawChessboardCorners(cv_image, patternSize, detected_corners, 1)
    gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray,2,3,0.04)

    #result is dilated for marking the corners, not important
    dst = cv2.dilate(dst,None)

    # Threshold for an optimal value, it may vary depending on the image.
    cv_image[dst>0.01*dst.max()]=[0,0,255]
    cv2.imshow("Image window", cv_image)
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
