# coding=utf-8
import os
import numpy as np
from PIL import Image
from pylab import *
import cv2


def get_imlist(path):
    """ Returns a list of filenames for
        all jpg images in a directory. """
    return [os.path.join(path, f) for f in os.listdir(path) if f.endswith('.jpg')]


def resize_image(im, sz):
    """ Resize an image array using PIL. """
    pil_im = Image.fromarray(uint8(im))
    return array(pil_im.resize(sz))


def compute_average(imlist):
    """ Compute the average of a list of images. """
    # open first image and make into array of type float
    averageim = array(Image.open(imlist[0]), 'f')
    for imname in imlist[1:]:
        try:
            averageim += array(Image.open(imname))
        except:
            print imname + '...skipped'
    averageim /= len(imlist)
    # return average as uint8
    return array(averageim, 'uint8')


def histeq(im, nbr_bins=256):
    """ Histogram equalization of a grayscale image. """
    # get image histogram
    imhist, bins = histogram(im.flatten(), nbr_bins, normed=True)
    cdf = imhist.cumsum()  # cumulative distribution function
    cdf = 255 * cdf / cdf[-1]  # normalize
    # use linear interpolation of cdf to find new pixel values
    im2 = interp(im.flatten(), bins[:-1], cdf)
    return im2.reshape(im.shape), cdf


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


def show_hist(img, bin=123):
    figure()
    hist(img.flatten(), bin)
    show()


def show_image(img, title='image', method='opencv'):
    """ open one window for one image """
    if method.lower() == 'opencv':
        cv2.imshow(title, img)
        cv2.waitKey(0)
    elif method.lower() == 'pylab':
        figure(title)
        axis('off')
        imshow(img)
        show()


def show_images(images):
    """ open windows for a list of images """
    for name, img in images:
        cv2.imshow(name, img)

    cv2.waitKey(0)
