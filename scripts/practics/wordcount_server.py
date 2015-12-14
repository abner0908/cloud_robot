#!/usr/bin/env python

import rospy
from cloud_robot.srv import WordCountType, WordCountTypeResponse

def count_words(req):
    word_count = len(req.words.split())
    return WordCountTypeResponse(word_count)

def word_count_server():
    rospy.init_node('wordcount_server')

    #rosy.Service(service name, transport message type, handle procedure)
    service = rospy.Service('word_count', WordCountType, count_words)
    print "Service is ready..."
    rospy.spin()

if __name__ == "__main__":
    word_count_server()

	
