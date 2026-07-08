from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class SensorReading:
    name: str
    ok: bool
    timestamp_s: float = field(default_factory=time.time)
    values: dict[str, Any] = field(default_factory=dict)
    message: str = ""


class BaseSensor:
    def __init__(self, name: str, config: dict[str, Any], simulation: bool = False) -> None:
        self.name = name
        self.config = config
        self.simulation = simulation
        self.is_open = False

    def open(self) -> None:
        self.is_open = True

    def read(self) -> SensorReading:
        raise NotImplementedError

    def close(self) -> None:
        self.is_open = False

    def error(self, message: str) -> SensorReading:
        return SensorReading(name=self.name, ok=False, message=message)
