#!/usr/bin/env python
import rospy
import utility
from cloud_robot.msg import FaceDetect
from utility import get_hostname


def add_timestamp(msg, count):
    now = rospy.Time.now()
    msg.header.frame_id = get_hostname() + '_' + str(count)
    msg.header.stamp.secs = now.secs
    msg.header.stamp.nsecs = now.nsecs
    return msg

node_name = "complex_pub_" + utility.get_hostname()
rospy.init_node(node_name)
pub = rospy.Publisher('/cloud/detect/face/result', FaceDetect, queue_size=20)
rate = rospy.Rate(2)
count = 0
while not rospy.is_shutdown():
    count += 1
    msg = FaceDetect()
    add_timestamp(msg, count)
    pub.publish(msg)
    rate.sleep()
