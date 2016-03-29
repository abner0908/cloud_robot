#!/usr/bin/env python
import sys
import select
import tty
import termios
import rospy
from std_msgs.msg import String
import imtools


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist == [sys.stdin]:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
    return key

if __name__ == '__main__':
    key_pub = rospy.Publisher('keys', String, queue_size=5)
    rospy.init_node("keyboard_driver")
    old_attr = termios.tcgetattr(sys.stdin)

    print "Publishing keystrokes. Press Ctrl-C to exit..."

    while not rospy.is_shutdown():
        key = getKey()
        if not key == '':
            if (ord(key) == imtools.KEY_CTRL_C):
                break

            key_pub.publish(key)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
