#!/usr/bin/env python
import rospy
import socket
from std_msgs.msg import Time

def talker():
    node_name = "talker_ros_" + socket.gethostname().split("-")[1]
    pub = rospy.Publisher('latency', Time, queue_size=10)
    rospy.init_node(node_name, anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(rospy.Time.now())
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
