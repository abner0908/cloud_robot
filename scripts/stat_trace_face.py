# coding=utf-8
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys
import getopt
import imtools
import os


class FaceDetection:

    def __init__(self, images_folder):
        cascade_eye = "./model/haarcascades/haarcascade_eye.xml"
        cascade_1 = "./model/haarcascades/haarcascade_frontalface_alt2.xml"
        cascade_2 = "./model/haarcascades/haarcascade_frontalface_alt.xml"
        cascade_3 = "./model/haarcascades/haarcascade_frontalface_profileface.xml"
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
        self.name_id = {}
        self.name_rect = {}
        self.target = 'abner'  # Tzuyu
        self.id_hit = 0
        self.id_miss = 0
        self.face_hit = 0
        self.face_miss = 0
        self.frame_count = 0
        if images_folder is None:
            self.images_folder = "/home/abner0908/Dropbox/Data/faces/all/"
        else:
            self.images_folder = images_folder

    def run(self):
        print('face detecting....')

        try:
            self.buildFaceMode()
            self.image_process()
        except KeyboardInterrupt:
            print("Shutting down.")
        finally:
            cv.DestroyAllWindows()

    def buildFaceMode(self):
        images, labels = self.get_images_and_labels("./faces")
        self.recognizer.train(images, np.array(labels))

    def get_images_and_labels(self, path):
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

    def get_images(self, path):
        image_paths = [os.path.join(path, f) for f in os.listdir(path)]
        return image_paths

    def image_process(self):
        print "image path: %s" % self.images_folder
        image_paths = self.get_images(self.images_folder)
        for path in image_paths:
            img = cv2.imread(path)
            self.frame_count += 1
            img_detected = self.detect_face(img)
            self.imgPlayer.show(img_detected)
            key = self.imgPlayer.get_key()
            if key == imtools.KEY_ESC:
                break
        try:
            self.show_statistic()
        except Exception as e:
            print e

    def show_statistic(self):
        print "frame count: %s" % self.frame_count
        face_total = self.face_hit + self.face_miss
        print "face hit rate: %s, hit number: %s, total: %s" % (float(self.face_hit) / face_total,  self.face_hit, face_total)
        id_total = self.id_hit + self.id_miss
        print "id hit rate: %s, hit number: %s, total: %s" % (float(self.id_hit) / id_total, self.id_hit, id_total)
        rospy.signal_shutdown('user hits ESC to exit')

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

        if len(rects) == 1:
            self.face_hit += 1
        else:
            self.face_miss += 1

        for rect in rects:
            (x1, y1, x2, y2) = rect
            # get detected face images
            face = self.fetch_face(img, (x1, y1, x2, y2))
            self.faces.append(face)
            name = self.recognize_face(face)

            if name is not None:  # and name in self.name_rect:
                self.name_rect[name] = rect
                if str(name) == str(self.target):
                    self.id_hit += 1
                    color = self.colors['red']
                else:
                    self.id_miss += 1
                    color = self.colors['green']

                cv2.rectangle(copy, (x1, y1), (x2, y2), color, 2)
                cv2.putText(copy, name, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

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

        return rects
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

stars = '*' * 5
help_msg = stars
help_msg += "%s  [-f <file_path>]" % sys.argv[0]
help_msg += stars

if __name__ == '__main__':
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hf:', '')
    except getopt.GetoptError as err:
        print str(err)
        print help_msg
        exit(1)

    images_folder = None
    for key, value in opts:
        if key == '-f':
            images_folder = value
        elif key == '-h':
            print help_msg
            exit(1)
        else:
            print help_msg
            exit(1)

    fd = FaceDetection(images_folder)
    fd.run()
