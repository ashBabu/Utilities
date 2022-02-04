#!/usr/bin/env python
from __future__ import print_function
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ObjectDetection:

    def __init__(self, image_topic):
        self.image_topic = image_topic
        self.bridge = CvBridge()

    def startDetection(self, image_topic=None):
        if not image_topic:
            image_topic = self.image_topic
        image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)

    def image_callback(self, image):
        try:
            colour_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            final_image = self.findRedGreen(colour_image)
            cv2.imshow('AwesomeWindow', final_image)
            k = cv2.waitKey(20) & 0xFF
        except CvBridgeError as e:
            print
            "FAILED"

    def findRedGreen(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # lower red
        lower_red = np.array([0, 125, 125])
        upper_red = np.array([5, 255, 255])

        # upper red
        lower_red2 = np.array([170, 125, 125])
        upper_red2 = np.array([190, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        res = cv2.bitwise_and(image, image, mask=mask)

        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        res2 = cv2.bitwise_and(image, image, mask=mask2)

        image3 = res + res2
        image4 = cv2.add(res, res2)

        # green_lower = np.array([25, 52, 72], np.uint8)    # original
        # green_upper = np.array([102, 255, 255], np.uint8)  # original

        # green_lower = np.array([5, 102, 150], np.uint8)   # good result1
        # green_upper = np.array([90, 255, 255], np.uint8)  # good result1

        green_lower = np.array([5, 50, 150], np.uint8)
        green_upper = np.array([90, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        red_mask = mask + mask2
        mask = cv2.bitwise_or(red_mask, green_mask)
        # mask = cv2.bitwise_or(mask1, mask2)
        target = cv2.bitwise_and(image, image, mask=mask)

        contours, hierarchy = cv2.findContours(red_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(image, (x, y),
                                           (x + w, y + h),
                                           (0, 0, 255), 2)

                cv2.putText(imageFrame, "Ripe", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))

        # Creating contour to track green color
        contours, hierarchy = cv2.findContours(green_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(image, (x, y),
                                           (x + w, y + h),
                                           (0, 255, 0), 2)

                cv2.putText(imageFrame, "Unripe", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 255, 0))

        # cv2.imshow("target", target)
        # cv2.imshow("original", image)
        # cv2.imshow("img3", img3)
        # cv2.imshow("img4", img4)

        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return image


if __name__ == '__main__':
    image_topic = "/robot_perception/vis/detection/image_raw"
    ic = ObjectDetection(image_topic)
    rospy.init_node('ObjectDetection', anonymous=True)
    ic.startDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
