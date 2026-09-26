"""Independent temporal gate. All times supplied in monotonic seconds."""


class GreenGate:
    def __init__(self, hold=0.5, min_frames=3, timeout=1.0):
        if hold < 0 or min_frames < 2 or timeout <= 0:
            raise ValueError('Invalid temporal gate settings')
        self.hold, self.min_frames, self.timeout = hold, min_frames, timeout
        self.reset()

    def reset(self):
        self.state = 'unknown'
        self.last = None
        self.start = None
        self.count = 0

    def update(self, state, received):
        if self.last is not None and received <= self.last:
            return
        if self.last is not None and received-self.last > self.timeout:
            self.reset()
        if state == 'green':
            if self.state != 'green':
                self.start, self.count = received, 0
            self.count += 1
        else:
            self.start, self.count = None, 0
        self.state, self.last = state, received

    def snapshot(self, now):
        if self.last is None or now-self.last > self.timeout:
            return 'unknown', False
        allowed = (self.state == 'green' and self.count >= self.min_frames
                   and self.last-self.start >= self.hold)
        return self.state, allowed
