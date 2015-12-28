#!/usr/bin/env python
import cv2
import cv2.cv as cv
import rospy
import sys
import time
import getopt
import itertools as it
from common import clock, draw_str
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

KEY_ECS = 27
show_video = False
bridge = CvBridge()
cascade_fn = "./model/haarcascades/haarcascade_frontalface_alt.xml"
nested_fn = "./model/haarcascades/haarcascade_eye.xml"

cascade = cv2.CascadeClassifier(cascade_fn)
nested = cv2.CascadeClassifier(nested_fn)


def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(
        30, 30), flags=cv.CV_HAAR_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:, 2:] += rects[:, :2]
    return rects


def draw_rects(img, rects, color):
    for x1, y1, x2, y2 in rects:
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)


def handle_sub(topic):
    rospy.init_node('face_detection', anonymous=True)

    global pub
    pub = rospy.Publisher("/opencv/detect/face", Image, queue_size=1)

    sub = rospy.Subscriber(topic, Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()


def callback(data):

    try:
        img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    time_start = clock()
    img_detected = detect_face(img)
    time_span = clock() - time_start

    if time_span == 0:
        fps = 0
    else:
        fps = 1 / time_span

    draw_str(img_detected, (5, 30), 'fps: %d' % fps)
    if show_video == True:
        cv2.imshow('face detection', img_detected)
        cv2.waitKey(1)

    try:
        pub.publish(bridge.cv2_to_imgmsg(img_detected, "bgr8"))
    except CvBridgeError as e:
        print(e)
# ....


def detect_face(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)

    rects = detect(gray, cascade)
    copy = img.copy()
    draw_rects(copy, rects, (0, 255, 0))

    for x1, y1, x2, y2 in rects:
        roi = gray[y1:y2, x1:x2]
        copy_roi = copy[y1:y2, x1:x2]
        subrects = detect(roi.copy(), nested)
        draw_rects(copy_roi, subrects, (255, 0, 0))

    return copy

help_msg = "cv_detect_face.py [-w (show video)] [-t <topic_name>]"

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'wt:', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    if len(opts) == 0:
        print help_msg
        exit(1)

    topic = None
    for key, value in opts:
        if key == '-w':
            show_video = True
        elif key == '-t':
            topic = value
        else:
            print help_msg
            exit(1)

    if topic == None:
        topic = '/file/video'
    handle_sub(topic)
