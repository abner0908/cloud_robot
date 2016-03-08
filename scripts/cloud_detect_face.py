#!/usr/bin/env python
import cv2
import cv2.cv as cv
import rospy
import sys
import getopt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FaceDetection:

    def __init__(self, topic, show_video, compute_index, total_nodes):
        self.topic_sub = topic
        self.KEY_ECS = 27
        self.should_show_video = should_show_video
        self.compute_index = compute_index
        self.total_nodes = total_nodes
        self.bridge = CvBridge()
        self.cascade_fn = "./model/haarcascades/haarcascade_frontalface_alt.xml"
        self.nested_fn = "./model/haarcascades/haarcascade_eye.xml"
        self.cascade = cv2.CascadeClassifier(self.cascade_fn)
        self.nested = cv2.CascadeClassifier(self.nested_fn)
        self.face_color = (0, 255, 0)
        self.eye_color = (255, 0, 0)
        self.shutdowm_msg = "Shutting down."
        self.node_name = 'face_detec_' + str(compute_index)
        self.topic_pub = "/cloud/detect/face"

    def run(self):
        print '%s node turn on' % (self.node_name)
        print 'face detecting....'
        self.handle_sub()

    def handle_sub(self):
        rospy.init_node(self.node_name, anonymous=True)
        rospy.on_shutdown(self.cleanup)

        self.pub = rospy.Publisher(self.topic_pub, Image, queue_size=1)
        self.sub = rospy.Subscriber(self.topic_sub, Image, self.callback)

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

        cv2.destroyAllWindows()

    def callback(self, msg_sub):

        try:
            img = self.bridge.imgmsg_to_cv2(msg_sub, "bgr8")
        except CvBridgeError as e:
            print(e)

        should_process = int(
            msg_sub.header.seq) % self.total_nodes == self.compute_index
        if should_process:
            img_detected = self.detect_face(img)

            self.show_video(img)

            try:
                msg_pub = self.bridge.cv2_to_imgmsg(img_detected, "bgr8")
                msg_pub = self.add_header(msg_pub, msg_sub)
            except CvBridgeError as e:
                print(e)

            try:
                self.pub.publish(msg_pub)
            except CvBridgeError as e:
                print(e)
        else:
            pass
    # ....

    def detect(self, img, cascade):
        rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(
            30, 30), flags=cv.CV_HAAR_SCALE_IMAGE)
        if len(rects) == 0:
            return []
        rects[:, 2:] += rects[:, :2]
        return rects
    # ...

    def draw_rects(self, img, rects, color):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    # ...

    def detect_face(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        rects = self.detect(gray, self.cascade)
        copy = img.copy()
        self.draw_rects(copy, rects, self.face_color)

        for x1, y1, x2, y2 in rects:
            roi = gray[y1:y2, x1:x2]
            copy_roi = copy[y1:y2, x1:x2]
            subrects = self.detect(roi.copy(), self.nested)
            self.draw_rects(copy_roi, subrects, self.eye_color)

        return copy
    # end def detect_face

    def cleanup(self):
        print self.shutdowm_msg
        cv2.destroyAllWindows()
    # ...

    def show_video(self, img):
        if self.should_show_video:
            cv2.imshow('face detection', img)
            if 0xFF & cv2.waitKey(1) == self.KEY_ECS:
                rospy.signal_shutdown("User hit q key to quit.")
    # ...

    def add_header(self, msg_pub, msg_sub):
        msg_pub.header.frame_id = msg_sub.header.frame_id
        msg_pub.header.stamp.secs = msg_sub.header.stamp.secs
        msg_pub.header.stamp.nsecs = msg_sub.header.stamp.nsecs
        return msg_pub
    # ...

stars = '*' * 5
help_msg = stars
help_msg += "cv_detect_face.py [-w (show video)] [-t <topic_name>]"
help_msg += "[-i <compute index>][-o <total node number>]"

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hwt:i:o:', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    if len(opts) == 0:
        print help_msg
        exit(1)

    topic = '/file/video'
    compute_index = 0
    total_nodes = 1
    should_show_video = False
    for key, value in opts:
        if key == '-w':
            should_show_video = True
        elif key == '-t':
            topic = value
        elif key == '-i':
            compute_index = int(value)
        elif key == '-o':
            total_nodes = int(value)
        elif key == '-h':
            print help_msg
            exit(1)
        else:
            print help_msg
            exit(1)
    fd = FaceDetection(topic, should_show_video, compute_index, total_nodes)
    fd.run()
