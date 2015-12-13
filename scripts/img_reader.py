#!/usr/bin/env python

import cv2
import os.path, sys
import numpy as np

im_path = '/home/abner0908/catkin_ws/map.jpg'

if not os.path.exists(im_path):
    print im_path, 'do not exist!!'
    exit(0)

#cv2.IMREAD_COLOR, cv2.IMREAD_GRAYSCALE , cv2.IMREAD_UNCHANGED
#Instead of these three flags, you can simply pass integers 1, 0 or -1 respectively.
im = cv2.imread(im_path, 1)

if (im == None):
    print "Could not open or find the image"
else:
    cv2.imshow('map image', im)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
