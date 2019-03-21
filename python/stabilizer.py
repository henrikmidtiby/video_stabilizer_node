#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('video_stabilizer_node')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class VideoStabilizer():
    def __init__(self):
        self.has_seen_first_frame = False

        # Initiate STAR detector
        self.orb = cv2.ORB_create()

        # Initiate FAST object with default values
        self.fast = cv2.FastFeatureDetector_create(nonmaxSuppression=True,
                threshold=100)
        
        # create BFMatcher object
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    

    def determine_homography_to_reference_frame(self, frame):
        pass
    
    def stabilize_frame(self, frame):
        if not self.has_seen_first_frame:
            cv2.imwrite("debug.png", frame)
            self.first_frame = frame
            self.kp_first_frame = self.fast.detect(frame, None)
            self.kp_first_frame, self.des_first_frame = self.orb.compute(frame,
                    self.kp_first_frame)
            self.has_seen_first_frame = True

        kp = self.fast.detect(frame, None)
        kp, des = self.orb.compute(frame, kp)

        # Match descriptors.
        matches = self.bf.match(des, self.des_first_frame)

        # Sort them in the order of their distance.
        matches = sorted(matches, key = lambda x:x.distance)

        good_matches = matches[:100]
        good_matches = matches

        src_pts = np.float32([ kp[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
        dst_pts = np.float32([ self.kp_first_frame[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        # M, mask = cv2.findHomography(src_pts, dst_pts, cv2.LMEDS, 1.0)

        frame = cv2.warpPerspective(frame, M, dsize=(frame.shape[1],
            frame.shape[0]))

        return frame


class image_converter:
  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("analyzed_image", Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)

    self.stabilizer = VideoStabilizer()


  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = self.analyze_image(cv_image)
    # self.showImage(cv_image)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


  def analyze_image(self, image):
    image = self.stabilizer.stabilize_frame(image)
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
