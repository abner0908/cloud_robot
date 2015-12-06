#!/usr/bin/env python

import sys
import rospy
import random
from cloud_robot.srv import *

def generate_rand_nums(max):
    nums = range(1, max + 1)
    random.shuffle(nums)
    return nums

def sorting_by_server(nums):
    rospy.wait_for_service('sorting_ints')
    try:
        sorting_ints = rospy.ServiceProxy('sorting_ints', SortingInts)
        responce = sorting_ints(nums)
        return responce.sorted
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def suggest_info():
    return "%s [max: maxium random integer range]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        max = int(sys.argv[1])
    else:
        print suggest_info()
        sys.exit(1)
    print "generating random integer numbers(from 1 to %s)" %max
    nums = generate_rand_nums(max)
    print "before sorting...."
    print nums
    print "after sorting...."
    sorted = sorting_by_server(nums)
    print sorted
