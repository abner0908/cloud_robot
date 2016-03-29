#!/usr/bin/env python
import socket
from common import clock


def get_hostname():
    hostname = socket.gethostname()
    if hostname.split('-') > 1:
        hostname = hostname.split('-')[0] + "_" + hostname.split('-')[1]
    return hostname


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]


def show_msg_info(msg, pinned=True, showLatency=False):
    import rospy
    result = 'header: \n'
    result += str(msg.header) + '\n'
    result += 'height: %s\n' % (msg.height)
    result += 'width: %s\n' % (msg.width)
    result += 'encoding: %s\n' % (msg.encoding)
    result += 'is_bigendian: %s\n' % (msg.is_bigendian)
    result += 'step: %s\n' % (msg.step)
    result += 'data size: %s\n' % (len(msg.data))

    secs = int(msg.header.stamp.secs)
    nsecs = int(msg.header.stamp.nsecs)
    latency = rospy.Time.now() - rospy.Time(secs, nsecs)
    latency_ms = latency.to_nsec() / 1000000.0

    if showLatency:
        result += 'latency: %.3f ms\n' % (latency_ms)

    if pinned:
        result = pinned_prefix(result)

    print(result)

    return latency_ms


def pinned_prefix(content):
    prefix = '\033[2J\033[;H'
    return (prefix + str(content))


class ExitLoop(Exception):
    pass


if __name__ == "__main__":
    pass
