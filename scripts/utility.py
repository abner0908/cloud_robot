import socket


def get_hostname():
    hostname = socket.gethostname()
    if hostname.split('-') > 1:
        hostname = hostname.split('-')[0] + "_" + hostname.split('-')[1]
    return hostname


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]


def show_msg_info(msg):
    import sys
    prefix = '\033[2J\033[;H'
    result = 'header: \n'
    result += str(msg.header) + '\n'
    result += 'height: %s\n' % (msg.height)
    result += 'width: %s\n' % (msg.width)
    result += 'encoding: %s\n' % (msg.encoding)
    result += 'is_bigendian: %s\n' % (msg.is_bigendian)
    result += 'step: %s\n' % (msg.step)
    result += 'data size: %s\n' % (len(msg.data))
    sys.stdout.write(prefix + result)


# import the necessary packages
from common import clock


class FPS:

    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0

    def __str__(self):
        self.stop()
        return str(self.fps())

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

    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        return (self._end - self._start)

    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()


class ExitLoop(Exception):
    pass


class _Getch:
    """
    Gets a single character from standard input.  Does not echo to the
    screen.
    """

    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()


class _GetchUnix:

    def __init__(self):
        import tty
        import sys

    def __call__(self):
        import sys
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:

    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()
