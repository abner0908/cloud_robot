#!/usr/bin/env python

import rospy
import cv2
import sys
import getopt
from sensor_msgs.msg import Image
from common import clock
from cv_bridge import CvBridge, CvBridgeError
from imtools import ImagePlayer


class ImgSub:

    def __init__(self, topic, should_mirror, verbose):
        self.topic = topic
        self.KEY_ECS = 27
        self.should_mirror = should_mirror
        self.verbose = verbose
        self.bridge = CvBridge()
        rospy.on_shutdown(self.cleanup)
        self.shutdowm_msg = "Shutting down."
        self.node_name = 'image_subscriber'
        self.time_start = clock()
        self.frames = []
        self.frame_max = 90
        self.imgPlayer = ImagePlayer("show %s" % (self.topic))

    def run(self):
        print '%s node turn on' % (self.node_name)
        self.handle_sub()
        self.show_video()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.cleanup()

    def handle_sub(self):
        rospy.init_node(self.node_name, anonymous=True)
        self.sub = rospy.Subscriber(self.topic, Image, self.callback)

    def callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.verbose:
                self.show_data_info(msg)
        except CvBridgeError as e:
            print(e)

        cv_image = self.mirror_image(cv_image)
        if len(self.frames) >= self.frame_max:
            self.frames.pop(0)

        self.frames.append(cv_image)
    # ...

    def show_video(self):
        while not rospy.is_shutdown():
            if len(self.frames) > 0:
                img = self.frames.pop(0)
                self.imgPlayer.show(img)

                key = self.imgPlayer.get_key()
                if key == self.KEY_ECS:
                    rospy.signal_shutdown("User hit q key to quit.")
                elif key == ord('a'):
                    file_name = 'image_%s.jpg' % (str(int(clock())))
                    cv2.imwrite(file_name, img)
                    print '%s has saved.' % file_name

    def mirror_image(self, img):
        if self.should_mirror:
            from imtools import mirror_image
            img = mirror_image(img)
        return img

    def show_data_info(self, msg):
        import utility
        utility.show_msg_info(msg, showLatency=True)
        print 'fps: %s' % (self.imgPlayer.get_fps())

    def cleanup(self):
        print self.shutdowm_msg
        cv2.destroyAllWindows()

stars = '*' * 5
help_msg = stars
help_msg += "file_img_sub.py [-m (mirror image)]"
help_msg += "[-v (show message info)][-t <topic name>]"
help_msg += stars

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hmvt:', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    should_mirror = False
    verbose = False
    topic = '/camera/video'
    for key, value in opts:
        if key == '-m':
            should_mirror = True
        elif key == '-v':
            verbose = True
        elif key == '-t':
            topic = value
        elif key == '-h':
            print help_msg
            exit(1)
        else:
            print help_msg
            exit(1)

    imgSub = ImgSub(topic, should_mirror, verbose)
    try:
        imgSub.run()
    finally:
        imgSub.cleanup()
