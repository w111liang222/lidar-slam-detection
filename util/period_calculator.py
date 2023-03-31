import time

class PeriodCalculator():
    def __init__(self):
        self.last_time = time.monotonic()
        self.fps = 0
    def hit(self):
        current_time = time.monotonic()
        self.fps = (1.0 / (current_time - self.last_time)) * 0.5 + 0.5 * self.fps
        self.last_time = current_time
        return self.fps