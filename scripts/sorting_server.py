#!/usr/bin/env python

from cloud_robot.srv import *
import rospy

def handle_sorting(req):
    print "get size %s a series of random integers and return the sorted list"%len(req.nums)
    sorted = sorting(req.nums)
    return SortingIntsResponse(sorted)

def sorting(nums):
    return sorted(nums)

def sorting_server():
    rospy.init_node('sorting_server')
    s = rospy.Service('sorting_ints', SortingInts, handle_sorting)
    print "Ready to sort random integers..."
    rospy.spin()

if __name__ == "__main__":
    sorting_server()
