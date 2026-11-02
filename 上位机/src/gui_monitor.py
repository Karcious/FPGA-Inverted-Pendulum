import math
from collections import deque

def q16(raw):

    raw = int(raw) & 0xFFFFFFFF
    if raw & 0x80000000:
        raw -= 0x100000000
    return raw / 65536.0

def error_series(rows, params):

    center = params[18]
    target = q16(params[13])
    angle = [(r[0] - center) * 360 / 4096 for r in rows]
    position = [(r[2] - target) * 360 / 1040 for r in rows]
    return angle, position

def window_error(values, sample_hz, times, seconds):

    seconds = float(seconds)
    if not values or sample_hz <= 0 or seconds <= 0:
        return None
    if times is not None:
        if len(times) != len(values):
            raise ValueError('时间轴与样本长度不一致')
        if len(times) < 2:
            return None
        cutoff = times[-1] - seconds
        start = next((i for i, t in enumerate(times) if t >= cutoff), len(times) - 1)
        duration = times[-1] - times[start]
    else:
        start = max(0, len(values) - int(seconds * sample_hz))
        duration = (len(values) - start - 1) / sample_hz
    window = values[start:]
    if len(window) < 2:
        return None
    rms = math.sqrt(sum(v * v for v in window) / len(window))
    mean_abs = sum(abs(v) for v in window) / len(window)
    return {'rms': rms, 'mean_abs': mean_abs,
            'duration': duration, 'count': len(window)}

def balanced(params, status):

    return (params[16] in (3, 4) and (status & 7) == 4 and not status & 0x18)

class StabilityStats:

    def __init__(self, band=1.5, dwell=1.0, max_gap=0.25, center_window=2.0):
        self.band = band
        self.dwell = dwell
        self.max_gap = max_gap
        self.center_window = center_window
        self.longest = 0.0
        self.total_delay = 0.0
        self.successes = 0
        self.attempts = 0
        self.interrupt()

    def interrupt(self):
        self.last_time = None
        self.was_balanced = None
        self.entered = None
        self.run_start = None
        self.current = 0.0
        self.converged = False
        self.center = None
        self._history = deque()

    @property
    def mean_delay(self):
        return self.total_delay / self.successes if self.successes else None

    def observe(self, t, error, balanced):
        if self.last_time is not None:
            dt = t - self.last_time
            if dt == 0:
                return
            if dt < 0 or dt > self.max_gap:
                self.interrupt()
        if balanced and self.was_balanced is False:
            self.entered = t
            self.attempts += 1
            self.converged = False
        if not balanced:
            self.entered = None
            self.run_start = None
            self.current = 0.0
            self.converged = False
            self.center = None
            self._history.clear()
        else:
            if math.isfinite(error):
                self._history.append((t, error))
                while t - self._history[0][0] > self.center_window:
                    self._history.popleft()
                values = sorted(v for _, v in self._history)
                self.center = values[len(values) // 2]
            if self.center is not None and abs(error - self.center) <= self.band:
                if self.run_start is None:
                    self.run_start = t
                self.current = t - self.run_start
                self.longest = max(self.longest, self.current)
                if self.entered is not None and not self.converged and self.current + 1e-9 >= self.dwell:
                    self.total_delay += self.run_start - self.entered
                    self.successes += 1
                    self.converged = True
            else:
                self.run_start = None
                self.current = 0.0
        self.last_time = t
        self.was_balanced = bool(balanced)
