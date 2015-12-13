#!/usr/bin/env python

import rospy, roslib
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def handle_sub(topic):
    rospy.init_node('image_subscriber', anonymous=True)
    cv2.namedWindow("map image", 1)
    sub = rospy.Subscriber(topic, Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

def callback(data):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("map image", cv_image)
    cv2.waitKey(3)

if __name__ == '__main__':

    if len(sys.argv) == 2:
        topic = sys.argv[1]
    else:
        topic = "/file/image"

    handle_sub(topic)