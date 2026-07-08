from __future__ import annotations

import time


class Rate:
    def __init__(self, hz: float) -> None:
        if hz <= 0:
            raise ValueError("Rate must be positive")
        self.period_s = 1.0 / hz
        self.next_time = time.monotonic()

    def sleep(self) -> None:
        self.next_time += self.period_s
        delay = self.next_time - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self.next_time = time.monotonic()


class StableTimer:
    def __init__(self, duration_s: float) -> None:
        self.duration_s = duration_s
        self._start: float | None = None

    def update(self, condition: bool) -> bool:
        now = time.monotonic()
        if not condition:
            self._start = None
            return False
        if self._start is None:
            self._start = now
        return now - self._start >= self.duration_s

    def reset(self) -> None:
        self._start = None
