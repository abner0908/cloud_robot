#!/usr/bin/env python

import roslib
import rospy
import utility
from cloud_robot.msg import Complex

def callback(msg):
    print 'Real:', msg.real
    print 'Imaginary:', msg.imaginary 
    print

node_name = "complex_sub_" + utility.get_hostname()
rospy.init_node(node_name)
sub = rospy.Subscriber('complex', Complex, callback)
rospy.spin()
