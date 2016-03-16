#!/usr/bin/env python
import cv2
import rospy
import sys, time, getopt
import itertools as it
from common import clock, draw_str
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def inside(r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh

def draw_detections(img, rects, thickness = 1):
    for x, y, w, h in rects:
        # the HOG detector returns slightly larger rectangles than the real objects.
        # so we slightly shrink the rectangles to get a nicer output.
        pad_w, pad_h = int(0.15*w), int(0.05*h)
        cv2.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), (0, 255, 0), thickness)

def detect_people(img, hog):
    found, w = hog.detectMultiScale(img, winStride=(8,8), padding=(32,32), scale=1.05)
    found_filtered = []
    for ri, r in enumerate(found):
        for qi, q in enumerate(found):
            if ri != qi and inside(r, q):
                break
        else:
            found_filtered.append(r)
            
    draw_detections(img, found)
    draw_detections(img, found_filtered, 3) 
    #print '%d (%d) found' % (len(found_filtered), len(found))       

def handle_sub(topic):
    rospy.init_node('people_detection', anonymous=True)
    sub = rospy.Subscriber(topic, Image, callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

def callback(data):
    bridge = CvBridge()
    try:
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    time_start = clock()
        
    detect_people(cv_img, hog)

    time_span = clock() - time_start
    if time_span == 0:
        fps = 0
    else:
        fps = 1 / time_span
    draw_str(cv_img, (5,30), 'fps: %d' % fps)

    if show_video == True:    
        cv2.imshow('people detection', cv_img)
        cv2.waitKey(1)

    pub = rospy.Publisher("/opencv/detect/people",Image, queue_size = 1)
    try:
      pub.publish(bridge.cv2_to_imgmsg(cv_img, "bgr8"))
    except CvBridgeError as e:
      print(e)    
# ....

show_video = False
help_msg = "cv_detect_people.py [-w (show video)] [-t <topic_name>]"

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

    topic = ""
    for key, value in opts:
        if key == '-w':
            show_video = True
        elif key == '-t':
            topic = value            
        else:
            print help_msg
            exit(1)    
 
    handle_sub(topic)
