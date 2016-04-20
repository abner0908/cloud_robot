# coding=utf-8
import os
import cv2
import rospy
from matplotlib import pyplot as plt
import numpy as np
from common import clock

KEY_ESC = 27
KEY_SPACE = 32
KEY_CTRL_C = 3


def get_imlist(path):
    """ Returns a list of filenames for
        all jpg images in a directory. """
    return [os.path.join(path, f) for f in os.listdir(path) if f.endswith('.jpg')]


def mirror_image(img):
    mirroredImage = np.fliplr(img).copy()
    return mirroredImage


def switchBR(img_src):
    if len(img_src.shape) == 3:
        b, g, r = cv2.split(img_src)
        img_dest = img_src.copy()
        cv2.merge((r, g, b), img_dest)
        return img_dest
    else:
        return img_src


def toGray(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def play_video(video_path):
    """ play video from the avi file. """
    key_esc = 27
    videoCapture = cv2.VideoCapture(video_path)

    success, img = videoCapture.read()
    # Loop until there are no more frames.
    while success:
        cv2.imshow('video', img)
        if 0xFF & cv2.waitKey(5) == key_esc:
            break
        success, img = videoCapture.read()

    cv2.destroyAllWindows()


def show_hist(image):
    chans = cv2.split(image)
    colors = {'b': 'Blue', 'g': 'Green', 'r': 'Red'}

    plt.figure(figsize=(13, 5))

    plt.subplot(121)
    plt.title("Image")
    plt.axis('equal')
    plt.axis('off')
    plt.imshow(switchBR(image), aspect='auto')

    plt.subplot(122)
    plt.title('Flatenned Color Histogram')
    plt.xlabel('Bins')
    plt.ylabel('# of Pixels')
    for (chan, color) in zip(chans, colors):
        hist = cv2.calcHist([chan], [0], None, [256], [0, 256])
        plt.plot(hist, color=color, label=colors[color])
        plt.xlim([0, 256])

    plt.legend()
    plt.show()


def show_image(img, title='image', method='opencv'):
    """ open one window for one image """
    if method.lower() == 'opencv':
        cv2.imshow(title, img)
        cv2.waitKey(0)
    elif method.lower() == 'pylab':
        plt.figure(title)
        plt.axis('off')
        plt.imshow(img)
        plt.show()


def show_images(images):
    """ open windows for a list of images """
    for name, img in images:
        cv2.imshow(name, img)

    cv2.waitKey(0)


def translate(image, x, y):
    # Define the translation matrix and perform the translation
    M = np.float32([[1, 0, x], [0, 1, y]])
    shifted = cv2.warpAffine(image, M, (image.shape[1], image.shape[0]))

    # Return the translated image
    return shifted


def rotate(image, angle, center=None, scale=1.0):
    # Grab the dimensions of the image
    (h, w) = image.shape[:2]

    # If the center is None, initialize it as the center of
    # the image
    if center is None:
        center = (w / 2, h / 2)

    # Perform the rotation
    M = cv2.getRotationMatrix2D(center, angle, scale)
    rotated = cv2.warpAffine(image, M, (w, h))

    # Return the rotated image
    return rotated


def resize(image, ratio=None, width=None, height=None, iterp=cv2.INTER_AREA):
    if not ratio == None:
        (height, width) = image.shape[:2]
        dim = (int(width * ratio), int(height * ratio))
    elif width == None and height == None:
        return image
    elif width == None:
        width = image.shape[1]
        ratio = height / float(image.shape[0])
        dim = (int(width * ratio), height)
    else:
        height = image.shape[0]
        ratio = width / float(image.shape[1])
        dim = (width, int(height * ratio))
    assert dim[0] > 0 and dim[
        1] > 0, "error width or height value: width: %s height: %s" % (dim[0], dim[1])
    return cv2.resize(image, dim, interpolation=iterp)


def embedInto(src, dst, loc=(0, 0)):
    srcH, srcW = src.shape[:2]
    dstH, dstW = dst.shape[:2]
    X, Y = loc
    if Y + srcH <= dstH and X + srcW <= dstW:
        dst[Y:Y + srcH, X:X + srcW] = src
    elif Y + srcH > dstH and X + srcW <= dstW:
        dst[Y:Y + srcH, X:X + srcW] = src[:dstH - Y, :srcW]
    elif Y + srcH <= dstH and X + srcW > dstW:
        dst[Y:Y + srcH, X:X + srcW] = src[:srcH, :dstW - X]
    else:
        dst[Y:Y + srcH, X:X + srcW] = src[:dstH - Y, :dstW - X]
    return dst


def get_width(img):
    return img.shape[1]


def get_height(img):
    return img.shape[0]


class VideoManager:

    def __init__(self, videoCapture, title='play video', process=None, keyEvent=None):
        self.capture = videoCapture
        self.process = process
        self.title = title
        self.keyEvent = keyEvent
        self.play_stop = False
        if not process:
            self.process = self.notProcess
        if not keyEvent:
            self.keyEvent = self.pressKeyEvent

    def notProcess(self, img):
        return img

    def play(self, playRate=30, showFPS=False, showOrignal=False):
        fps = FPS()
        fps.start()
        rate = rospy.Rate(playRate)
        success, frame = self.capture.read()
        while success and not self.play_stop:
            processed = self.process(frame)
            if showFPS:
                self.draw_fps(processed, fps)
            if showOrignal:
                cv2.imshow(self.title, np.hstack([frame, processed]))
            else:
                cv2.imshow(self.title, processed)

            self.pressKeyEvent(cv2.waitKey(1))
            success, frame = self.capture.read()
            fps.update()
            rate.sleep()

    def pressKeyEvent(self, key):
        KEY_ESC = 27
        if 0xFF & key == KEY_ESC:
            self.play_stop = True

    def draw_fps(self, img, fps):
        from common import draw_str
        draw_str(img, (5, 30), 'fps: %s' % round(fps, 2))


class FPS:

    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0
        self._window_size = 120

    def __str__(self):
        self.stop()
        return str(self.fps())

    def __float__(self):
        self.stop()
        return self.fps()

    def start(self):
        # start the timer
        self._start = clock()
        return self

    def stop(self):
        # stop the timer
        self._end = clock()

    def update(self):
        # increment the total number of frames examined during the
        # start and end intervals
        self._numFrames += 1
        if self._numFrames == self._window_size * 2:
            self._numFrames -= 120
            self._start = self._window_start

        if self._numFrames == self._window_size:
            self._window_start = clock()

    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        if self._start == None or self._end == None:
            raise Exception(
                "to get the fps value before the fps runs start or stop function.")

        return (self._end - self._start)

    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()


class ImagePlayer:

    def __init__(self, title='Image Showing', showInfo=True):
        self.title = title
        self.key = None
        self.showFPS = showInfo
        self.fps = FPS()
        self.fps.start()

    def show(self, img, extraInfo=''):

        if self.showFPS:
            img = self.draw_info(img, self.fps, extraInfo)

        cv2.imshow(self.title, img)
        self.key = 0xFF & cv2.waitKey(1)
        self.fps.update()

    def get_key(self):
        return self.key

    def get_fps(self):
        return self.fps

    def draw_info(self, img, fps, extraInfo):
        from common import draw_str
        draw_str(img, (5, 30), 'fps: %s %s' % (round(fps, 2), extraInfo))
        return img

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == "__main__":
    import mahotas

    def canny_process(image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 3)
        canny = cv2.Canny(gray, 30, 150)
        #canny = 255 - canny
        canny = cv2.cvtColor(canny, cv2.COLOR_GRAY2RGB)
        return canny

    def threshold_process(image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (7, 7), 0)
        # gray = cv2.bilateralFilter(gray, 13, 41, 41)
        gray = cv2.medianBlur(gray, 3)
        T = mahotas.thresholding.otsu(gray)
        T = 180
        mask = gray.copy()
        mask[mask > T] = 255
        mask[mask < T] = 0
        #mask = 255 - mask
        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
        return mask

    file_path = './images/bad_apple_color.avi'
    capture = cv2.VideoCapture(file_path)
    video = VideoManager(capture, "bad apple")
    video.play(playRate=45, showFPS=True)
