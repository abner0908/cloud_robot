#!/usr/bin/env python
import rospy


class ROS_Manager:

    def __init__(self, node_name, bufferSize=90):
        self.node_name = node_name
        self.bufferSize = bufferSize
        self.buffer = []
        rospy.init_node(self.node_name)

    def handle_reader(self):
        pass

    def handle_writer(self):
        pass

    def write_buffer(self, data):
        if len(buffer) == self.bufferSize:
            self.buffer.pop(0)

        self.buffer.append(data)

    def read_buffer(self):
        if len(buffer) == 0:
            data = None
        elif len(buffer) == 1:
            data = buffer[0]
        else:
            data = self.buffer.pop(0)
        return data

    def run(self):
        self.handle_reader()
        self.handle_writer()

if __name__ == '__main__':
    ros_mngr = ROS_Manager()
    ros_mngr.run()
