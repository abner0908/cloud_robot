# coding=utf-8
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys
import getopt
import imtools
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class ROSFaceDetection:

    def __init__(self, topic, should_show_video, compute_index, total_nodes):
        self.topic_sub = topic
        self.should_show_video = should_show_video
        self.compute_index = compute_index
        self.total_nodes = total_nodes
        self.cv_init()
        self.ros_init()

    def cv_init(self):
        self.bridge = CvBridge()
        cascade_eye = "./model/haarcascades/haarcascade_eye.xml"
        cascade_1 = rospy.get_param(
            "~cascade_1", "./model/haarcascades/haarcascade_frontalface_alt2.xml")
        cascade_2 = rospy.get_param(
            "~cascade_2", "./model/haarcascades/haarcascade_frontalface_alt.xml")
        cascade_3 = rospy.get_param(
            "~cascade_3", "./model/haarcascades/haarcascade_frontalface_profileface.xml")
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)
        self.cascade_3 = cv2.CascadeClassifier(cascade_3)
        self.cascade_eye = cv2.CascadeClassifier(cascade_eye)
        self.haar_scaleFactor = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 5)
        self.haar_minSize = rospy.get_param("~haar_minSize", 30)
        # self.haar_maxSize = rospy.get_param("~haar_maxSize", 150)
        self.haar_params = dict(scaleFactor=self.haar_scaleFactor,
                                minNeighbors=self.haar_minNeighbors,
                                # flags=cv.CV_HAAR_DO_CANNY_PRUNING
                                # flags=cv2.CASCADE_SCALE_IMAGE
                                flags=cv.CV_HAAR_DO_CANNY_PRUNING,
                                minSize=(self.haar_minSize, self.haar_minSize)
                                )
        self.colors = dict(green=(0, 255, 0),
                           red=(0, 0, 255),
                           blue=(255, 0, 0))
        self.recognizer = cv2.createLBPHFaceRecognizer()
        self.img_title = "face detection"
        self.imgPlayer = imtools.ImagePlayer(title=self.img_title)
        self.save_folder = './tmp'
        self.faces = []
        self.rects = []
        self.name_id = {}
        self.name_rect = {}
        self.target = 'abner'  # 'abner' #'Taylor' #Tzuyu
        self.frames = []
        self.headers = []
        self.frame_max = 30
        self.id_hit = 0
        self.id_miss = 0
        self.face_hit = 0
        self.face_miss = 0
        self.frame_count = 0
        self.last_rects = []
        self.take_last_rects = True
        self.orignal = None
        self.detected = None
        self.sel_rect = []
        self.cropping = False
        self.Pause = False
        self.sel_rect_endpoint = []
        cv2.namedWindow(self.img_title)
        cv2.setMouseCallback(self.img_title, self.click_and_crop)

    def ros_init(self):
        self.node_name = 'face_detec_' + str(compute_index)
        self.shutdowm_msg = "%s node shutting down." % (self.node_name)
        self.topic_pub = "/cloud/detect/face"

    def run(self):
        rospy.loginfo('%s node turn on' % (self.node_name))
        rospy.loginfo('face detecting....')

        try:
            self.buildFaceMode()
            self.handle_ros()
            self.image_process()
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down %s node." % (fd.node_name))
        finally:
            cv.DestroyAllWindows()

    def buildFaceMode(self):
        images, labels = self.get_images_and_labels("./faces")
        self.recognizer.train(images, np.array(labels))

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
            self.add_names(name)

            labels.append(self.name_id[name])
            images.append(image)

            # add the mirrored image for training
            labels.append(self.name_id[name])
            images.append(imtools.mirror_image(image))
        print self.name_id
        return images, labels

    def add_names(self, name):
        if not name in self.name_id:
            self.name_id[name] = len(self.name_id) + 1
            self.name_rect[name] = None

    def handle_ros(self):
        rospy.init_node(self.node_name, anonymous=True)
        rospy.on_shutdown(self.cleanup)

        self.pub_img = rospy.Publisher(self.topic_pub, Image, queue_size=1)
        self.sub_img = rospy.Subscriber(self.topic_sub, Image, self.callback)
        self.pub_direct = rospy.Publisher("keys", String, queue_size=1)

    def callback(self, msg_sub):

        is_assigned = int(
            msg_sub.header.seq) % self.total_nodes == self.compute_index
        if is_assigned:
            try:
                img = self.bridge.imgmsg_to_cv2(msg_sub, "bgr8")
            except CvBridgeError as e:
                print(e)

        self.write_buffer(img, msg_sub.header)

    def write_buffer(self, img, header):
        if len(self.frames) >= self.frame_max:
            self.frames.pop()
            self.headers.pop()
        img = imtools.mirror_image(img)
        self.frames.append(img)
        self.headers.append(header)

    def read_buffer(self):
        if len(self.frames) > 1:
            return self.frames.pop(0), self.headers.pop(0)
        else:
            return self.frames[0], self.headers[0]

    def image_process(self):

        while not rospy.is_shutdown():
            if not self.Pause and len(self.frames) > 0:
                self.frame_count += 1
                img, header = self.read_buffer()
                img_detected = self.detect_face(img)
                self.orignal = img
                self.detected = img_detected
                self.show_video(img_detected)

                # convert data into msg and publish them
                try:
                    msg_pub = self.bridge.cv2_to_imgmsg(img_detected, "bgr8")
                    msg_pub = self.add_header(msg_pub, header)
                except CvBridgeError as e:
                    rospy.logerr(e)
                    rospy.signal_shutdown(e)

                try:
                    self.pub_img.publish(msg_pub)
                except CvBridgeError as e:
                    rospy.logerr(e)
                    rospy.signal_shutdown(e)
            else:
                self.handle_mouse_event()

    def detect_face(self, img):
        self.rects = []
        self.faces = []
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        rects = self.detect(gray, self.cascade_1)
        if len(rects) == 0:
            rects = self.detect(gray, self.cascade_3)
        if len(rects) == 0:
            rects = self.detect(gray, self.cascade_2)
        self.rects = rects
        copy = img.copy()
        self.draw_rects(copy, rects, self.colors['green'])

        if len(rects) > 0:
            self.last_rects = rects
            self.face_hit += 1
        else:
            if self.take_last_rects:
                rects = self.last_rects
            self.face_miss += 1

        for rect in rects:
            (x1, y1, x2, y2) = rect
            # get detected face images
            face = self.fetch_face(img, (x1, y1, x2, y2))
            self.faces.append(face)
            name = self.recognize_face(face)

            if name is not None:  # and name in self.name_rect:
                self.name_rect[name] = rect
                self.trace_face(name, imtools.get_width(img))
                if str(name) == str(self.target):
                    self.id_hit += 1
                    color = self.colors['red']
                else:
                    self.id_miss += 1
                    color = self.colors['green']

                cv2.rectangle(copy, (x1, y1), (x2, y2), color, 2)
                cv2.putText(copy, name, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # detect eyes and draw the roi of eyes
        # for x1, y1, x2, y2 in rects:
        #     roi = gray[y1:y2, x1:x2]
        #     copy_roi = copy[y1:y2, x1:x2]
        #     subrects = self.detect(roi.copy(), self.cascade_eye)
        #     self.draw_rects(copy_roi, subrects, self.colors['red'])

        return copy
    # end def detect_face

    def detect(self, img, cascade):
        face_rects = []
        bias = 15
        rects = cascade.detectMultiScale(img, **self.haar_params)
        if len(rects) == 0:
            return []
        rects[:, 2:] += rects[:, :2]
        for x1, y1, x2, y2 in rects:
            if x1 - bias > 0:
                x1 = x1 - bias
            else:
                x1 = x1

            if y1 - bias > 0:
                y1 = y1 - bias
            else:
                y1 = y1

            if x2 + bias < imtools.get_width(img):
                x2 = x2 + bias
            else:
                x2 = x2

            if y2 + bias < imtools.get_height(img):
                y2 = y2 + bias
            else:
                y2 = y2

            face_rects.append([x1, y1, x2, y2])

        return face_rects
    # ...

    def draw_rects(self, img, rects, color):
        for x1, y1, x2, y2 in rects:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    # ...

    def fetch_face(self, img, rect):
        (x1, y1, x2, y2) = rect
        face = img[y1:y2, x1:x2]
        return face

    def recognize_face(self, face):
        # recognize the name of a face
        gray = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
        id, conf = self.recognizer.predict(gray)
        if conf < 200:
            return self.get_name(id)
        else:
            return None

    def get_name(self, id):
        name = 'unkown'
        for key in self.name_id:
            if self.name_id[key] == id:
                return key
        return name

    def cleanup(self):
        print self.shutdowm_msg
        cv2.destroyAllWindows()
    # ...

    def show_video(self, img_detected):
        if self.should_show_video:
            FACE_DEFAULT_WIDTH = 100
            SMALL_FACE_X = 0

            for face in self.faces:
                # embed face into original image
                face = imtools.resize(face, width=FACE_DEFAULT_WIDTH)
                img_detected = self.embed_face(
                    img_detected, face, SMALL_FACE_X, 0)
                SMALL_FACE_X = SMALL_FACE_X + FACE_DEFAULT_WIDTH

            self.imgPlayer.show(
                img_detected, extraInfo='queue: %s' % (str(len(self.frames))))
            key = self.imgPlayer.get_key()
            self.handle_key_event(key)
    # ...

    def embed_face(self, img, face, X, Y):
        face_h, face_w = face.shape[:2]
        face_x1, face_y1 = X, Y
        face_x2, face_y2 = face_x1 + face_w, face_y1 + face_h
        img = imtools.embedInto(face, img, (face_x1, face_y1))
        cv2.rectangle(img, (face_x1, face_y1),
                      (face_x2, face_y2), self.colors['green'], 2)
        return img

    def trace_face(self, name, width):
        width_middle = width / 2.0
        if self.target == name:
            rect = self.name_rect[name]
            (x1, y1, x2, y2) = rect
            face_mid = x1 + (x2 - x1) / 2
            if face_mid > width_middle:
                self.robot_turn_left()
            elif face_mid < width_middle:
                self.robot_turn_right()

    def robot_turn_left(self):
        self.pub_direct.publish("a")

    def robot_turn_right(self):
        self.pub_direct.publish("d")

    def handle_key_event(self, key):
        # exit the program
        if key == imtools.KEY_ESC:
            rospy.signal_shutdown("User hit the key to quit.")
            print "frame count: %s" % self.frame_count
            face_total = self.face_hit + self.face_miss
            print "face hit rate: %s, miss rate: %s total: %s" % (float(self.face_hit) / face_total, float(self.face_miss) / face_total, face_total)
            id_total = self.id_hit + self.id_miss
            print "id hit rate: %s, miss rate: %s total: %s" % (float(self.id_hit) / id_total, float(self.id_miss) / id_total, id_total)
            rospy.signal_shutdown('user hits ESC to exit')
        # pause video playing
        elif key == imtools.KEY_SPACE:
            self.Pause = not self.Pause
            if self.Pause:
                self.sel_rect = []
        # save face images
        elif key == ord('s'):
            if len(self.faces) > 0:
                for face in self.faces:
                    file_name = self.save_folder + '/face_%s.jpg' % (
                        rospy.Time.now().to_nsec())
                    cv2.imwrite(file_name, face)
                print '%s face images has saved.' % (len(self.faces))
                self.faces = []
            else:
                print 'no face is detected!!'
        elif 0xFF & key == ord('a'):
            file_name = self.save_folder + \
                '/image_%s.jpg' % (rospy.Time.now().to_nsec())
            if self.detected is not None:
                cv2.imwrite(file_name, self.detected)
                print '%s has saved.' % file_name
            else:
                print 'no photo can be saved!!'

    def handle_mouse_event(self):
        if self.orignal is None:
            return

        if not self.cropping:
            if len(self.sel_rect) == 0:
                clone = self.detected.copy()
            else:
                clone = self.orignal.copy()
            self.show_roi(self.orignal, clone, self.sel_rect)
            cv2.imshow(self.img_title, clone)

        elif self.cropping:
            clone = self.orignal.copy()
            if self.sel_rect_endpoint is not None:
                cv2.rectangle(clone, tuple(self.sel_rect[: 2]), tuple(
                    self.sel_rect_endpoint[: 2]), (0, 255, 0), 1)
            cv2.imshow(self.img_title, clone)
        key = cv2.waitKey(1) & 0xFF
        self.handle_key_event(key)

    def click_and_crop(self, event, x, y, flags, param):
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        if event == cv2.EVENT_LBUTTONDOWN:
            self.sel_rect = [x, y]
            self.cropping = True
            self.sel_rect_endpoint = None

        # check to see if the left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            # record the ending (x, y) coordinates and indicate that
            # the cropping operation is finished
            self.sel_rect += [x, y]
            self.cropping = False

        elif event == cv2.EVENT_MOUSEMOVE and self.cropping:
            self.sel_rect_endpoint = [x, y]

    def show_roi(self, image, clone, rect):
        if len(rect) == 4:
            x1, y1, x2, y2 = rect
            # draw a rectangle around the region of interest
            cv2.rectangle(clone, (x1, y1), (x2, y2), (0, 255, 0), 2)
            minX = min(x1, x2)
            minY = min(y1, y2)
            maxX = max(x1, x2)
            maxY = max(y1, y2)
            if maxX - minX > 0 and maxY - minY > 0:
                roi = image[minY:maxY, minX: maxX]
                self.faces = [roi]

    def add_header(self, msg_pub, header):
        msg_pub.header.frame_id = header.frame_id
        msg_pub.header.stamp.secs = header.stamp.secs
        msg_pub.header.stamp.nsecs = header.stamp.nsecs
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

    fd = ROSFaceDetection(topic, should_show_video,
                          compute_index, total_nodes)
    fd.run()
    rospy.spin()
