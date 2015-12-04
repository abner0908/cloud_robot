#!/usr/bin/env python

import roslib
import rospy
import utility
from cloud_robot.msg import Complex 
from random import random

node_name = "complex_pub_" + utility.get_hostname()
rospy.init_node(node_name)
pub = rospy.Publisher('complex', Complex, queue_size = 20)
rate = rospy.Rate(2)

while not rospy.is_shutdown(): 
    msg = Complex()
    msg.real = random()
    msg.imaginary = random()
    pub.publish(msg)
    print "publish Complex real: %f imaginary: %f" % (msg.real, msg.imaginary)
    rate.sleep()
