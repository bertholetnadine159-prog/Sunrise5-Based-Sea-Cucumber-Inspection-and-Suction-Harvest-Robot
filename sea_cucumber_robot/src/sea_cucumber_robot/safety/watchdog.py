from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass
class Watchdog:
    name: str
    timeout_s: float
    last_heartbeat_s: float = 0.0

    def __post_init__(self) -> None:
        self.kick()

    def kick(self) -> None:
        self.last_heartbeat_s = time.monotonic()

    def expired(self) -> bool:
        return time.monotonic() - self.last_heartbeat_s > self.timeout_s

    def remaining_s(self) -> float:
        return max(0.0, self.timeout_s - (time.monotonic() - self.last_heartbeat_s))
