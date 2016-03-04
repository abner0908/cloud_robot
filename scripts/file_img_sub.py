#!/usr/bin/env python

import rospy
import roslib
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

KEY_ECS = 27


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
        print data.header
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    cv2.imshow("map image", cv_image)
    if 0xFF & cv2.waitKey(1) == KEY_ECS:
        sys.exit(1)


if __name__ == '__main__':

    if len(sys.argv) == 2:
        topic = sys.argv[1]
    else:
        topic = "/file/image"

    handle_sub(topic)
