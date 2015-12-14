#!/usr/bin/env python

import rospy
import sys
from cloud_robot.srv import WordCountType

def word_count_client(words):
    rospy.init_node('wordcount_clinet')
    rospy.wait_for_service('word_count')
    try:
        word_counter = rospy.ServiceProxy('word_count', WordCountType)
	rsp = word_counter(words)
        return rsp.count
    except rospy.ServiceException, e:
	print "Service call failed: %s" %e

if __name__ == "__main__":
    
    while True:
	words = raw_input("input words('end' to exit):")
	words = words.strip()

	if words == 'end':
	    sys.exit(1)

        word_count = word_count_client(words)
        print words, "->", word_count
	 

