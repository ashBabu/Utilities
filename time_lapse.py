#!/usr/bin/env python

import rospy
import cv2
import time
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError  # The CvBridge is an object that converts between OpenCV Images and ROS Image messages.
                                               # CvBridge is a ROS library that provides an interface between ROS and OpenCV.


class TimeLapse:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ceiling_camera/qhd/image_color_rect", Image, self.callback)

    def callback(self, data):
        global i
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # Now color_image is an opencv image
        except CvBridgeError as e:
            print e
        cv2.imshow("Image window", cv_image)
        path = './timeLapseImages'
        # cv2.imwrite(os.path.join(path, './tlapse' + str(i).zfill(4) + '.png'), cv_image, [int(cv2.IMWRITE_PNG_COMPRESSION), 90])
        cv2.imwrite(os.path.join(path, './tlapse' + str(i).zfill(4) + '.jpg'), cv_image,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cv2.waitKey(3)


if __name__ == '__main__':
    global i
    rospy.init_node('Mushroom_detection', anonymous=True)
    rate = rospy.Rate(15)
    i = 0
    try:
        while not rospy.is_shutdown():
            TimeLapse()
            rate.sleep()
            time.sleep(600)   # Captures an image every 10 minutes
            i = i + 1
    except rospy.ROSInterruptException:
        pass
