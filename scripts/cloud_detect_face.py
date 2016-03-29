#!/usr/bin/env python
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys
import getopt
import imtools
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class FaceDetection:

    def __init__(self, topic, show_video, compute_index, total_nodes):
        self.topic_sub = topic
        self.should_show_video = should_show_video
        self.compute_index = compute_index
        self.total_nodes = total_nodes
        self.bridge = CvBridge()
        cascade_fn = "./model/haarcascades/haarcascade_frontalface_alt.xml"
        nested_fn = "./model/haarcascades/haarcascade_eye.xml"
        self.cascade = cv2.CascadeClassifier(cascade_fn)
        self.nested = cv2.CascadeClassifier(nested_fn)
        self.face_color = (0, 255, 0)
        self.eye_color = (255, 0, 0)
        self.shutdowm_msg = "Shutting down."
        self.node_name = 'face_detec_' + str(compute_index)
        self.topic_pub = "/cloud/detect/face"
        self.recognizer = cv2.createLBPHFaceRecognizer()
        self.Pause = False
        self.faces = []
        self.names = {}

    def run(self):
        print '%s node turn on' % (self.node_name)
        print 'face detecting....'
        self.buildFaceMode()
        self.handle_ros()

    def buildFaceMode(self):
        images, labels = self.get_images_and_labels("./faces")
        self.recognizer.train(images, np.array(labels))

    def handle_ros(self):
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

        is_assigned = int(
            msg_sub.header.seq) % self.total_nodes == self.compute_index
        if is_assigned:
            if not self.Pause:
                img_detected, rects = self.detect_face(img)

                self.show_video(img_detected, img, rects)

                # convert data into msg and publish them
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
                self.handle_key_event()
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

        return copy, rects
    # end def detect_face

    def cleanup(self):
        print self.shutdowm_msg
        cv2.destroyAllWindows()
    # ...

    def show_video(self, img_detected, original, rects):
        if self.should_show_video:
            FACE_DEFAULT_WIDTH = 100
            SMALL_FACE_X = 0

            for (x1, y1, x2, y2) in rects:
                # get detected face images
                face = self.fetch_face(original, (x1, y1, x2, y2))
                self.faces.append(face)

                # recognize the name of a face
                face = imtools.resize(face, width=FACE_DEFAULT_WIDTH)
                gray = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
                id, conf = self.recognizer.predict(gray)
                if conf < 200:
                    cv2.putText(img_detected, self.get_name(id), (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                # embed face into original image
                img_detected = self.embed_face(
                    img_detected, face, SMALL_FACE_X, 0)
                SMALL_FACE_X = SMALL_FACE_X + FACE_DEFAULT_WIDTH

            cv2.imshow('face detection', img_detected)
            self.handle_key_event()
    # ...

    def get_name(self, id):
        name = 'unkown'
        for key in self.names:
            if self.names[key] == id:
                return key
        return name

    def fetch_face(self, img, rect):
        (x1, y1, x2, y2) = rect
        face = img[y1:y2, x1:x2]
        return face

    def embed_face(self, img, face, X, Y):
        face_h, face_w = face.shape[:2]
        face_x1, face_y1 = X, Y
        face_x2, face_y2 = face_x1 + face_w, face_y1 + face_h
        img = imtools.embedInto(face, img, (face_x1, face_y1))
        cv2.rectangle(img, (face_x1, face_y1),
                      (face_x2, face_y2), (0, 0, 255), 2)
        return img

    def handle_key_event(self):
        key = 0xFF & cv2.waitKey(1)
        # exit the program
        if key == imtools.KEY_ECS:
            rospy.signal_shutdown("User hit q key to quit.")
        # pause video playing
        elif key == imtools.KEY_SPACE:
            self.Pause = not self.Pause
        # save face images
        elif key == ord('s'):
            if len(self.faces) > 0:
                for face in self.faces:
                    file_name = './faces/face_%s.jpg' % (
                        rospy.Time.now().to_nsec())
                    cv2.imwrite(file_name, face)
                print '%s face images has saved.' % (len(self.faces))
            else:
                print 'no face is detected!!'

    def get_images_and_labels(self, path):
        import os

        image_paths = [os.path.join(path, f) for f in os.listdir(path)]
        # images will contains face images
        images = []
        # labels will contains the label that is assigned to the image
        labels = []
        for image_path in image_paths:
            # Read the image and convert to grayscale
            image = cv2.imread(image_path)
            # Convert the image format into numpy array
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Get the label of the image
            name = os.path.split(image_path)[1].split("_")[0]
            if not name in self.names:
                self.names[name] = len(self.names) + 1
            labels.append(self.names[name])
            images.append(image)

        return images, labels

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
