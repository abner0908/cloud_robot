#!/usr/bin/env python

import cv2
import numpy as np


def read_image(im_path):
    #cv2.IMREAD_COLOR, cv2.IMREAD_GRAYSCALE , cv2.IMREAD_UNCHANGED
    #Instead of these three flags, you can simply pass integers 1, 0 or -1 respectively.
    im = cv2.imread('/home/abner0908/catkin_ws/map.jpg', 1)

    cv2.imshow('map image', im)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    if len(sys.argv) == 2:
        path = sys.argv[1]
    else:
        path = "/home/abner0908/catkin_ws/map.jpg"

    read_image(path)
