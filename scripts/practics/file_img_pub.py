#!/usr/bin/env python
import rospy, roslib
import cv2
import os.path, sys, time, datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def handle_pub(im_path):
    rospy.init_node('image_publisher')
    pub = rospy.Publisher('/file/image', Image, queue_size = 2)
    bridge = CvBridge()
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            im = read_image(im_path)
            msg = bridge.cv2_to_imgmsg(im, "bgr8")  
            pub.publish(msg)
        except CvBridgeError as e:
            print(e)
        rate.sleep()  

    cv2.destroyAllWindows()

def read_image(im_path):
    #cv2.IMREAD_COLOR, cv2.IMREAD_GRAYSCALE , cv2.IMREAD_UNCHANGED
    #Instead of these three flags, you can simply pass integers 1, 0 or -1 respectively.
    im = cv2.imread(im_path, 1)
    now = time.time()
    now_str = datetime.datetime.fromtimestamp(now).strftime('%Y-%m-%d %H:%M:%S')
    cv2.putText(im, now_str, (0,20), cv2.FONT_ITALIC, 0.7, 1)
    cv2.imshow('map image', im)
    cv2.waitKey(3)
    #cv2.destroyAllWindows()
    return im

if __name__ == '__main__':

    if len(sys.argv) == 2:
        path = sys.argv[1]
    else:
        path = "/home/abner0908/catkin_ws/map.jpg"

    if os.path.isfile(path):
        read_image(path)
    else:
        print path, 'do not exist!!'
        exit(0)
    
    cv2.namedWindow("map image", 1)
    handle_pub(path)

