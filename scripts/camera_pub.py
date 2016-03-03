#!/usr/bin/env python
import rospy
import roslib
import cv2
import os.path
import sys
import getopt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from common import clock, draw_str

KEY_ECS = 27
FPS = 30


def handle_pub(video_path):
    topic = '/camera/video'
    rospy.init_node('camera_publisher')
    pub = rospy.Publisher(topic, Image, queue_size=2)

    print "publish camera to topic:%s from camera ..." % (topic)
    videoCapture = cv2.VideoCapture(0)
    bridge = CvBridge()

    #rate = rospy.Rate(FPS)
    time_start = clock()
    frame_count = 0
    success, img = videoCapture.read()
    while success:
        img_copy = img.copy()
        try:
            msg = bridge.cv2_to_imgmsg(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        pub.publish(msg)

        if show_video == True:
            time_span = clock() - time_start
            if time_span == 0:
                fps = 0
            fps = frame_count / time_span
            draw_str(img_copy, (5, 30), 'fps: %d' % fps)

            cv2.imshow('play video', img_copy)
            if 0xFF & cv2.waitKey(1) == KEY_ECS:
                break

        #rate.sleep()
        success, img = videoCapture.read()
        frame_count += 1

    cv2.destroyAllWindows()
# ....

show_video = False
help_msg = "file_vido_pub.py [-w (show video)]"

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'w', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    path = ""
    for key, value in opts:
        if key == '-w':
            show_video = True

    handle_pub(path)
