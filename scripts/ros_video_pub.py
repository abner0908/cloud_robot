#!/usr/bin/env python
import rospy
import cv2
import sys
import os
import getopt
import threading
import imtools
from imtools import FPS, ImagePlayer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utility import get_hostname, getch


class VideoPub:

    def __init__(self, capture, topic, show_window=False, show_info=False, rate=32):
        self.show_window = show_window
        self.show_info = show_info
        self.videoCapture = capture
        self.topic = topic
        self.rate = rate
        self.bridge = CvBridge()
        self.KEY_ECS = 27
        self.imgPlayer = ImagePlayer()
        self.mutex = threading.Lock()
        self.fps = FPS()
        self.msg = None

    def run(self):
        self.handle_pub()

    def handle_pub(self):

        rospy.init_node('video_publisher')
        pub = rospy.Publisher(topic, Image, queue_size=2)
        self.handle_threading()

        count = 0
        self.fps.start()
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            success, img = self.videoCapture.read()
            if not success:
                break

            try:
                msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            except CvBridgeError as e:
                print(e)
                rospy.signal_shutdown(e)

            count += 1
            self.fps.update()
            self.msg = self.add_timestamp(msg, count)
            pub.publish(self.msg)

            self.show_image(img)
            rate.sleep()
        # end wile
    # ....

    def handle_threading(self):
        threading.Thread(target=self.exit_key_event).start()
        if self.show_info:
            threading.Thread(target=self.show_data_info).start()

    def exit_key_event(self):
        while not rospy.is_shutdown():
            with self.mutex:
                ch = getch()
                if len(ch) > 0 and (ch == 'q' or ord(ch) == imtools.KEY_CTRL_C):
                    print 'user press crtl + c or q to exit'
                    rospy.signal_shutdown('user press crtl + c or q to exit')
                    break

    def add_timestamp(self, msg, count):
        now = rospy.Time.now()
        msg.header.frame_id = get_hostname() + '-' + str(count)
        msg.header.stamp.secs = now.secs
        msg.header.stamp.nsecs = now.nsecs
        return msg

    def show_image(self, img):
        if self.show_window:
            self.imgPlayer.show(img)
            key = self.imgPlayer.get_key()
            if 0xFF & key == self.KEY_ECS:
                rospy.signal_shutdown('user press ESC to exit')

    def show_data_info(self):
        import utility
        import time
        while not rospy.is_shutdown():
            time.sleep(.5)
            with self.mutex:
                utility.show_msg_info(self.msg)
                print('fps: %s' % round(self.fps, 2))
                print '%s' % (quit_msg)

    def cleanup(self):
        self.videoCapture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    help_msg = "%s [-w (show video)][-v (show message info)][-d <device number>]" % (sys.argv[0])
    help_msg += "[-t <topic name>][-f <file path>][-r <play rate>]"
    quit_msg = 'press crtl + c or q to quit'

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
    video_capture = cv2.VideoCapture(0)
    rate = 32
    for key, value in opts:
        if key == '-w':
            show_window = True
            quit_msg = 'press ESC to quit'
        elif key == '-v':
            show_info = True
        elif key == '-d':
            video_capture = cv2.VideoCapture(int(value))
        elif key == '-f':
            if not os.path.isfile(value):
                print 'Path error %s do not exist!!' % value
                exit(0)
            video_capture = cv2.VideoCapture(value)
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

    if not topic:
        topic = '/%s/video' % source

    stars = '-' * 60
    print '\n%s' % (stars)
    print "publish video to topic: %s from the %s ..." % (topic, source)
    print '%s' % (quit_msg)
    print '%s' % (stars)

    videoPub = VideoPub(video_capture, topic,
                        show_window, show_info, rate=rate)

    videoPub.run()
