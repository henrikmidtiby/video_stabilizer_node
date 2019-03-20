#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('video_stabilizer')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("analyzed_image", Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)


  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.analyze_image(cv_image)
    self.showImage(cv_image)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


  def analyze_image(self, image):
    (rows, cols, channels) = image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(image, (50, 50), 10, 255, 3)
    return image


  def showImage(self, image):
    cv2.imshow("Image window", image)
    cv2.waitKey(3)


def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    print("Launching the stabilizer")
    main(sys.argv)
