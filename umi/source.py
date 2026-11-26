"""Sensor thread base: stop, measured rate, fixed-period loop, error on exception."""
import threading
import time


class SourceThread(threading.Thread):
    def __init__(self, period=None, label="source"):
        super().__init__(daemon=True)
        self.label = label
        self.period = period      # s; None: no sleep (camera read() paces the loop)
        self.rate = 0.0
        self.error = None
        self._halt = threading.Event()
        self._reset_clock()

    def run(self):
        """Zero rate on exit so a stale Hz doesn't look healthy; cleanup goes in _run()'s finally."""
        try:
            self._run()
        except Exception as e:
            self.error = "%s stopped: %s" % (self.label, e)
        finally:
            self.rate = 0.0

    def _reset_clock(self):
        """Call right before the loop so setup time doesn't skew rate/period."""
        self._n, self._t_rate = 0, time.monotonic()
        self._next_t = time.monotonic()

    def tick(self):
        """Call once per loop iteration."""
        self._n += 1
        now = time.monotonic()
        if now - self._t_rate >= 1.0:
            self.rate, self._n, self._t_rate = self._n / (now - self._t_rate), 0, now
        if self.period:
            self._next_t += self.period
            sl = self._next_t - time.monotonic()
            if sl > 0:
                time.sleep(sl)
            else:
                self._next_t = time.monotonic()

    def stop(self):
        self._halt.set()
