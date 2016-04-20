import socket
import sys


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


try:
    import tty
    import select
    import termios
except ImportError:
    # Probably Windows.
    try:
        import msvcrt
    except ImportError:
        # FIXME what to do on other platforms?
        # Just give up here.
        raise ImportError('getch not available')
    else:
        getch = msvcrt.getch
else:
    def getch():
        """getch() -> key character

        Read a single keypress from stdin and return the resulting character.
        Nothing is echoed to the console. This call will block if a keypress
        is not already available, but will not wait for Enter to be pressed.

        If the pressed key was a modifier key, nothing will be detected; if
        it were a special function key, it may return the first character of
        of an escape sequence, leaving additional characters in the buffer.
        """

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        ch = ''
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


if __name__ == "__main__":
    pass
    print getch()
