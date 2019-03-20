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


class VideoStabilizer():
    def __init__(self, crop_locations):
        pass
        self.has_seen_first_frame = False
        self.crop_locations = crop_locations
        self.crop_height = 40
        self.crop_width = 40
        self.crops = []
        self.crop_matches = []

    
    def stabilize_frame(self, frame):
        if not self.has_seen_first_frame:
            cv2.imwrite("debug.png", frame)
            self.extract_crops(frame)
            self.has_seen_first_frame = True

        self.locate_crop_matches(frame)
        frame = self.correct_small_movements(frame)
        return frame

    
    def extract_crops(self, frame):
        self.crops = []
        for i, location in enumerate(self.crop_locations):
            x_lower = location[1] - self.crop_height
            x_upper = location[1] + self.crop_height
            y_lower = location[0] - self.crop_width
            y_upper = location[0] + self.crop_width
            temp_frame = frame[x_lower:x_upper, y_lower:y_upper]
            self.crops.append(temp_frame)


    def locate_crop_matches(self, frame):
        searchsize = 150
        self.crop_matches = []
        for i, crop in enumerate(self.crops):
            match = self.locate_window(frame, crop, self.crop_locations[i],
                    searchsize)
            self.crop_matches.append(match)


    def locate_window(self, frame, window, estimated_position, search_range):
        # Remember to switch x and y when subsetting an image
        cropped_frame = frame[estimated_position[1] - search_range:estimated_position[1] + search_range,
                              estimated_position[0] - search_range:estimated_position[0] + search_range]
        result = cv2.matchTemplate(cropped_frame, window, cv2.TM_SQDIFF)
        _, _, match, _ = cv2.minMaxLoc(result, None)
        match_adjusted = (estimated_position[0] - search_range + match[0] + window.shape[0] / 2,
                          estimated_position[1] - search_range + match[1] + window.shape[1] / 2)
        return match_adjusted


    def correct_small_movements(self, frame):
        for match in self.crop_matches:
            cv2.circle(frame, match, 50, (255, 255, 0), 10)
        return frame


class image_converter:
  def __init__(self):
    rospy.init_node('image_converter', anonymous=True)
    self.image_pub = rospy.Publisher("analyzed_image", Image, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)

    self.stabilizer = VideoStabilizer([[273, 167], [200, 508], [1042, 562],
        [1154, 152]])


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
    print("it should be active")
    main(sys.argv)
