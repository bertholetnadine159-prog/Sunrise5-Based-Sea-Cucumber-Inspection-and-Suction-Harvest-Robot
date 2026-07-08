from __future__ import annotations

import math
from typing import Any

from .base import BaseSensor, SensorReading


REG_ALS_CONF = 0x00
REG_ALS = 0x04
REG_WHITE = 0x05


class VEML7700Sensor(BaseSensor):
    def __init__(self, name: str, config: dict[str, Any], simulation: bool = False) -> None:
        super().__init__(name, config, simulation)
        self.bus = None
        self._sim_phase = 0.0

    def open(self) -> None:
        if self.simulation:
            self.is_open = True
            return
        try:
            from smbus2 import SMBus
        except Exception as exc:
            raise RuntimeError("smbus2 is required for VEML7700") from exc

        self.bus = SMBus(int(self.config["bus"]))
        self._write_word(REG_ALS_CONF, 0x0000)
        self.is_open = True

    def _write_word(self, register: int, value: int) -> None:
        assert self.bus is not None
        # VEML7700 registers are little-endian over SMBus word operations.
        self.bus.write_word_data(int(self.config["address"]), register, value & 0xFFFF)

    def _read_word(self, register: int) -> int:
        assert self.bus is not None
        return int(self.bus.read_word_data(int(self.config["address"]), register)) & 0xFFFF

    def read(self) -> SensorReading:
        if self.simulation:
            self._sim_phase += 0.12
            lux = 180.0 + 40.0 * math.sin(self._sim_phase)
            return SensorReading(self.name, True, values={"lux": lux, "als_raw": int(lux / 0.0576)})

        if not self.is_open:
            return self.error("sensor is not open")
        try:
            als_raw = self._read_word(REG_ALS)
            white_raw = self._read_word(REG_WHITE)
            lux = als_raw * 0.0576
            return SensorReading(
                self.name,
                True,
                values={"lux": lux, "als_raw": als_raw, "white_raw": white_raw},
            )
        except Exception as exc:
            return self.error(str(exc))

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = None
        self.is_open = False
