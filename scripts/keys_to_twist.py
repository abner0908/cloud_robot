#!/usr/bin/env python

import rospy
import sys
import select
import termios
import tty
import imtools
from utility import ExitLoop
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Keys_to_Twist:

    def __init__(self):
        self.node_name = 'keys_to_twist'
        self.topic_pub = 'cmd_vel'
        self.topic_sub = 'keys'
        self.speed = 0.2
        self.turn = 1
        self.keys = []
        self.settings = termios.tcgetattr(sys.stdin)
        self.moveBindings = {
            'w': (1, 0),
            'e': (1, -1),
            'a': (0, 1),
            'd': (0, -1),
            'q': (1, 1),
            's': (-1, 0),
            'c': (-1, 1),
            'z': (-1, -1),
        }

        self.speedBindings = {
            'u': (1.1, 1.1),
            'm': (.9, .9),
            'i': (1.1, 1),
            ',': (.9, 1),
            'o': (1, 1.1),
            '.': (1, .9),
        }

        self.op_msg = """
        Control Your Turtlebot!
        ---------------------------
        Moving around:
        q    w    e
        a    x    d
        z    s    c

        u/m : increase/decrease max speeds by 10%
        i/, : increase/decrease only linear speed by 10%
        o/. : increase/decrease only angular speed by 10%
        space key, x : force stop
        anything else : stop smoothly

        CTRL-C to quit
        """
    # end  def __init__(self):

    def run(self):
        self.init_ros()
        self.keys_process()

    def init_ros(self):
        rospy.init_node(self.node_name)
        self.pub = rospy.Publisher(self.topic_pub, Twist, queue_size=5)
        self.sub = rospy.Subscriber(self.topic_sub, String, self.callback)

    def callback(self, msg):
        if len(msg.data):
            self.add_key(msg.data[0])

    def vels_msg(self):
        return "currently:\tspeed %s\tturn %s " % (self.speed, self.turn)

    def getKey(self):
        if len(self.keys) > 0:
            return self.keys.pop(0)
        else:
            return ''

    def monitor_keypress(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            self.add_key(sys.stdin.read(1))

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def add_key(self, key):
        self.keys.append(key)

    def keys_process(self):
        print self.op_msg
        print self.vels_msg()
        while True:
            try:
                self.send_twist()
            except ExitLoop:
                break

    def send_twist(self):

        x = 0
        th = 0
        status = 0
        count = 0
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        try:
            while True:
                self.monitor_keypress()
                key = self.getKey()
                if key in self.moveBindings.keys():
                    x = self.moveBindings[key][0]
                    th = self.moveBindings[key][1]
                    count = 0
                elif key in self.speedBindings.keys():
                    self.speed = self.speed * self.speedBindings[key][0]
                    self.turn = self.turn * self.speedBindings[key][1]
                    count = 0

                    print self.vels_msg()
                    if (status == 14):
                        print self.op_msg
                    status = (status + 1) % 15
                elif key == ' ' or key == 'x':
                    x = 0
                    th = 0
                    control_speed = 0
                    control_turn = 0
                else:
                    count = count + 1
                    if count > 4:
                        x = 0
                        th = 0

                target_speed = self.speed * x
                target_turn = self.turn * th

                if target_speed > control_speed:
                    control_speed = min(target_speed, control_speed + 0.02)
                elif target_speed < control_speed:
                    control_speed = max(target_speed, control_speed - 0.02)
                else:
                    control_speed = target_speed

                if target_turn > control_turn:
                    control_turn = min(target_turn, control_turn + 0.1)
                elif target_turn < control_turn:
                    control_turn = max(target_turn, control_turn - 0.1)
                else:
                    control_turn = target_turn

                twist = Twist()
                twist.linear.x = control_speed
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = control_turn
                self.pub.publish(twist)
                if len(key) > 0 and ord(key) == imtools.KEY_CTRL_C:
                    raise ExitLoop
                if control_speed == 0 and control_turn == 0:
                    break

                # print("loop: {0}".format(count))
                # print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
                # print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x,
                # twist.angular.z))

        except ExitLoop as e:
            raise e
        finally:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    # end def send_twist(self)

if __name__ == "__main__":

    ros_srv = Keys_to_Twist()
    ros_srv.run()
