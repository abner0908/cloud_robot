#!/usr/bin/env python

import rospy
import cv2
import sys
import getopt
from sensor_msgs.msg import Image
from common import clock, draw_str
from cv_bridge import CvBridge, CvBridgeError
from cloud_robot.msg import FaceDetect
from utility import FPS


class ImgSub:

    def __init__(self, topic, should_mirror, verbose, limit=1000):
        self.topic = topic
        self.KEY_ECS = 27
        self.should_mirror = should_mirror
        self.verbose = verbose
        self.bridge = CvBridge()
        rospy.on_shutdown(self.cleanup)
        self.shutdowm_msg = "Shutting down."
        self.node_name = 'image_subscriber'
        self.time_start = clock()
        self.limit = limit
        self.frame_count = 0
        self.total_latency = 0
        self.fps = FPS()
        self.hosts = {}

    def run(self):
        print '%s node turn on' % (self.node_name)
        self.fps = self.fps.start()
        self.handle_sub()

    def handle_sub(self):
        rospy.init_node(self.node_name, anonymous=True)
        cv2.namedWindow("show %s" % (self.topic), 10)

        self.sub = rospy.Subscriber(self.topic, FaceDetect, self.callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print(self.shutdowm_msg)
            cv2.destroyAllWindows()

    def callback(self, msg):
        self.show_data_info(msg)
        self.frame_count += 1
        if self.frame_count >= self.limit:
            rospy.signal_shutdown("reach the limit of frames.")
    # ...

    def show_video(self, img):
        self.fps.update()
        draw_str(img, (5, 30), 'fps: %s' % self.fps)

        cv2.imshow("show %s" % (self.topic), img)
        key = cv2.waitKey(1)
        if 0xFF & key == self.KEY_ECS:
            rospy.signal_shutdown("User hit q key to quit.")
        elif 0xFF & key == ord('a'):
            file_name = 'image_%s.jpg' % (str(int(clock())))
            cv2.imwrite(file_name, img)
            print '%s has saved.' % file_name

    def count_frames(self, msg):
        host = msg.header.frame_id.split('-')[0]
        if host in self.host:
            self.host[host] += 1
        else:
            self.host[host] = 1

    def show_data_info(self, msg):
        import utility
        result = 'header: \n'
        result += str(msg.header) + '\n'
        result = utility.pinned_prefix(result)
        secs = int(msg.header.stamp.secs)
        nsecs = int(msg.header.stamp.nsecs)
        latency = rospy.Time.now() - rospy.Time(secs, nsecs)
        latency_ms = latency.to_nsec() / 1000000.0
        self.count_frames(msg)
        print result
        print 'latency: %.3f ms\n' % (latency_ms)

    def cleanup(self):
        print self.shutdowm_msg
        cv2.destroyAllWindows()

stars = '*' * 5
help_msg = stars
help_msg += "file_img_sub.py [-m (mirror image)]"
help_msg += "[-t <topic name>][-l <frmae limit>]"
help_msg += stars

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hmt:l:', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    should_mirror = False
    verbose = True
    limit = 1000
    topic = '/camera/video'
    for key, value in opts:
        if key == '-m':
            should_mirror = True
        elif key == '-t':
            topic = value
        elif key == '-l':
            limit = int(value)
        elif key == '-h':
            print help_msg
            exit(1)
        else:
            print help_msg
            exit(1)

    imgSub = ImgSub(topic, should_mirror, verbose, limit=limit)
    try:
        imgSub.run()
    finally:
        imgSub.cleanup()
        print '%s frames has be played' % (imgSub.frame_count)
        print 'total latency: %s' % (imgSub.total_latency)
        print 'average latency: %s' % (imgSub.total_latency / imgSub.frame_count)
        print imgSub.hosts
