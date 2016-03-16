#!/usr/bin/env python
import rospy
import cv2
import sys
import os
import getopt
from common import draw_str
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utility import get_hostname, ExitLoop, FPS


class VideoPub:

    def __init__(self, videoCapture, topic, show_window=False, show_info=False, rate=32):
        self.show_window = show_window
        self.show_info = show_info
        self.videoCapture = videoCapture
        self.topic = topic
        self.rate = rate
        self.bridge = CvBridge()
        self.KEY_ECS = 27

    def run(self):
        self.handle_pub()

    def handle_pub(self):

        rospy.init_node('video_publisher')
        pub = rospy.Publisher(topic, Image, queue_size=2)

        count = 0
        fps = FPS()
        fps = fps.start()
        rate = rospy.Rate(self.rate)
        success, img = self.videoCapture.read()

        while success:

            try:
                msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            except CvBridgeError as e:
                print(e)

            count += 1
            msg = self.add_timestamp(msg, count)
            pub.publish(msg)

            if self.show_window:
                try:
                    self.show_image(img, fps)
                except (ExitLoop, KeyboardInterrupt, SystemExit):
                    self.cleanup()
                    break

            if self.show_info:
                self.show_data_info(msg, fps)

            success, img = self.videoCapture.read()
            rate.sleep()
        # end wile
    # ....

    def add_timestamp(self, msg, count):
        now = rospy.Time.now()
        msg.header.frame_id = get_hostname() + '_' + str(count)
        msg.header.stamp.secs = now.secs
        msg.header.stamp.nsecs = now.nsecs
        return msg

    def show_image(self, img, fps):
        fps.update()
        draw_str(img, (5, 30), 'fps: %s' % round(fps, 2))

        cv2.imshow('play video', img)
        if 0xFF & cv2.waitKey(1) == self.KEY_ECS:
            raise ExitLoop

    def show_data_info(self, msg, fps):
        import utility
        utility.show_msg_info(msg)
        print('fps: %s' % round(fps, 2))

    def cleanup(self):
        self.videoCapture.release()
        cv2.destroyAllWindows()


help_msg = "camera_pub.py [-w (show video)][-v (show message info)][-d <device number>]"
help_msg += "[-t <topic name>][-f <file path>][-r <play rate>]"
if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hwvd:t:f:r:', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    topic = None
    source = 'camera'
    show_window = False
    show_info = False
    videoCapture = cv2.VideoCapture(0)
    rate = 35
    for key, value in opts:
        if key == '-w':
            show_window = True
        elif key == '-v':
            show_info = True
        elif key == '-d':
            videoCapture = cv2.VideoCapture(int(value))
        elif key == '-f':
            if not os.path.isfile(value):
                print 'Path error %s do not exist!!' % value
                exit(0)
            videoCapture = cv2.VideoCapture(value)
            rate = 30
            source = 'file'
        elif key == '-r':
            rate = int(value)
        elif key == '-t':
            topic = value
        elif key == '-h':
            print help_msg
            exit(1)
        else:
            print help_msg
            exit(1)

    if topic == None:
        topic = '/%s/video' % source

    stars = '*' * 60
    print stars
    print "publish video to topic: %s from the %s ..." % (topic, source)
    print stars

    videoPub = VideoPub(videoCapture, topic, show_window, show_info, rate=rate)
    try:
        videoPub.run()
    finally:
        videoPub.cleanup()
