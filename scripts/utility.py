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
        pinned_prefix(result)

    print(result)

    return latency_ms


def pinned_prefix(content):
    prefix = '\033[2J\033[;H'
    print(prefix + str(content))


class FPS:

    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0
        self._window_size = 120

    def __str__(self):
        self.stop()
        return str(self.fps())

    def __float__(self):
        self.stop()
        return self.fps()

    def start(self):
        # start the timer
        self._start = clock()
        return self

    def stop(self):
        # stop the timer
        self._end = clock()

    def update(self):
        # increment the total number of frames examined during the
        # start and end intervals
        self._numFrames += 1
        if self._numFrames == self._window_size * 2:
            self._numFrames -= 120
            self._start = self._window_start

        if self._numFrames == self._window_size:
            self._window_start = clock()

    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        return (self._end - self._start)

    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()


class ExitLoop(Exception):
    pass


if __name__ == "__main__":
    import time
    fps = FPS()
    fps.start()

    while True:
        fps.update()
        time.sleep(0.03)
        pinned_prefix('fps: %s' % (fps))
