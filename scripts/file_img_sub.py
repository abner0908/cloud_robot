#!/usr/bin/env python

import rospy
import roslib
import cv2
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utility import get_hostname, ExitLoop


class ImgSub:

    def __init__(self, topic):
        self.topic = topic
        self.KEY_ECS = 27
        self.bridge = CvBridge()
        rospy.on_shutdown(self.cleanup)
        self.shutdowm_msg = "Shutting down."

    def handle_sub(self):
        rospy.init_node('image_subscriber', anonymous=True)
        #cv2.namedWindow("map image", 1)

        self.sub = rospy.Subscriber(self.topic, Image, self.callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print(self.shutdowm_msg)
            cv2.destroyAllWindows()

    def callback(self, data):

        try:
            print data.header
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("map image", cv_image)
        if 0xFF & cv2.waitKey(1) == self.KEY_ECS:
            rospy.signal_shutdown("User hit q key to quit.")

    def cleanup(self):
        print self.shutdowm_msg
        cv2.destroyAllWindows()

if __name__ == '__main__':

    if len(sys.argv) == 2:
        topic = sys.argv[1]
    else:
        topic = "/file/image"

    imgSub = ImgSub(topic)
    imgSub.handle_sub()
