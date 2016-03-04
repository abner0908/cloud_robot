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
show_video = False


def handle_pub(videoCapture):
    topic = '/camera/video'
    dashes = '_' * 60
    print dashes
    print "publish video to topic:%s from camera ..." % (topic)
    print dashes
    rospy.init_node('camera_publisher')
    pub = rospy.Publisher(topic, Image, queue_size=2)

    bridge = CvBridge()

    time_start = clock()
    frame_count = 0
    success, img = videoCapture.read()
    count = 0
    while success:
        img_copy = img.copy()
        try:
            msg = bridge.cv2_to_imgmsg(img, "bgr8")
            msg = add_timestamp(msg, count)
            count += 1
            print msg.header
        except CvBridgeError as e:
            print(e)

        pub.publish(msg)

        if show_video == True:
            try:
                play_video(img_copy, time_start, frame_count)
                frame_count += 1
            except ExitLoop:
                cv2.destroyAllWindows()
                break

        # rate.sleep()
        success, img = videoCapture.read()

    # end wile
# ....


def add_timestamp(msg, count):
    from utility import get_hostname, ExitLoop
    now = rospy.Time.now()
    msg.header.frame_id = get_hostname() + '_' + str(count)
    msg.header.stamp.secs = now.secs
    msg.header.stamp.nsecs = now.nsecs
    return msg


def play_video(img, time_start, frame_count):
    time_span = clock() - time_start
    if time_span == 0:
        fps = 0
    fps = frame_count / time_span
    draw_str(img, (5, 30), 'fps: %d' % fps)

    cv2.imshow('play video', img)
    if 0xFF & cv2.waitKey(1) == KEY_ECS:
        raise ExitLoop

help_msg = "camera_pub.py [-w (show video)][-d <device number>]"

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'wd:', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    device_num = 0
    for key, value in opts:
        if key == '-w':
            show_video = True
        elif key == '-d':
            device_num = int(value)
        else:
            print help_msg
            exit(1)

    videoCapture = cv2.VideoCapture(device_num)
    try:
        handle_pub(videoCapture)
    except Exception as e:
        print 'Error occured! error message: %s' % (e)
    finally:
        videoCapture.release()
