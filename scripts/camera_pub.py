#!/usr/bin/env python
import rospy
import cv2
import sys
import getopt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from common import draw_str
from utility import get_hostname, ExitLoop, FPS

KEY_ECS = 27
show_video = False
verbose = False


def handle_pub(videoCapture):
    topic = '/camera/video'
    dashes = '_' * 60
    print dashes
    print "publish video to topic:%s from camera ..." % (topic)
    print dashes
    rospy.init_node('camera_publisher')
    pub = rospy.Publisher(topic, Image, queue_size=2)

    bridge = CvBridge()
    fps = FPS()
    fps = fps.start()
    success, img = videoCapture.read()
    count = 0
    while success:
        img_copy = img.copy()
        try:
            msg = bridge.cv2_to_imgmsg(img, "bgr8")
            count += 1
            msg = add_timestamp(msg, count)

            if verbose:
                show_data_info(msg, fps)
        except CvBridgeError as e:
            print(e)

        pub.publish(msg)

        if show_video:
            try:
                show_image(img_copy, fps)
            except (ExitLoop, KeyboardInterrupt, SystemExit):
                cleanup(videoCapture)
                break

        success, img = videoCapture.read()

    # end wile
# ....


def add_timestamp(msg, count):
    now = rospy.Time.now()
    msg.header.frame_id = get_hostname() + '_' + str(count)
    msg.header.stamp.secs = now.secs
    msg.header.stamp.nsecs = now.nsecs
    return msg


def show_image(img, fps):
    fps.update()
    draw_str(img, (5, 30), 'fps: %s' % fps)

    cv2.imshow('play video', img)
    if 0xFF & cv2.waitKey(1) == KEY_ECS:
        raise ExitLoop


def show_data_info(msg, fps):
    import utility
    utility.show_msg_info(msg)
    print('fps: %s' % fps)


def cleanup(videoCapture):
    videoCapture.release()
    del(videoCapture)
    cv2.destroyAllWindows()


help_msg = "camera_pub.py [-w (show video)][-v (show message info)][-d <device number>]"

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hwvd:', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    device_num = 0
    for key, value in opts:
        if key == '-w':
            show_video = True
        elif key == '-v':
            verbose = True
        elif key == '-d':
            device_num = int(value)
        elif key == '-h':
            print help_msg
            exit(1)
        else:
            print help_msg
            exit(1)

    try:
        videoCapture = cv2.VideoCapture(device_num)
        handle_pub(videoCapture)
    except (ExitLoop, KeyboardInterrupt, SystemExit) as e:
        print 'Error occured! error message: %s' % (e)
    finally:
        cleanup(videoCapture)
