from common import clock


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
        if self._start == None or self._end == None:
            raise Exception(
                "to get the fps value before the fps runs start or stop function.")

        return (self._end - self._start)

    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()

if __name__ == "__main__":
    fps = FPS()
    print fps
