from __future__ import annotations

import time
from typing import Any

from .base import BaseSensor, SensorReading


CMD_RESET = 0x1E
CMD_ADC_READ = 0x00
CMD_CONVERT_D1_8192 = 0x4A
CMD_CONVERT_D2_8192 = 0x5A
CMD_PROM_READ_BASE = 0xA0
GRAVITY_M_S2 = 9.80665


class MS5837Sensor(BaseSensor):
    def __init__(self, name: str, config: dict[str, Any], simulation: bool = False) -> None:
        super().__init__(name, config, simulation)
        self.bus = None
        self.coefficients: list[int] = []
        self.surface_pressure_mbar: float | None = None

    def open(self) -> None:
        if self.simulation:
            self.is_open = True
            self.surface_pressure_mbar = 1013.25
            return
        try:
            from smbus2 import SMBus
        except Exception as exc:
            raise RuntimeError("smbus2 is required for MS5837") from exc
        self.bus = SMBus(int(self.config["bus"]))
        self._write_byte(CMD_RESET)
        time.sleep(0.02)
        self.coefficients = self._read_prom()
        self.is_open = True

        first = self._read_pressure_temperature()
        self.surface_pressure_mbar = first[0]

    def _write_byte(self, value: int) -> None:
        assert self.bus is not None
        self.bus.write_byte(int(self.config["address"]), value)

    def _read_block(self, command: int, length: int) -> list[int]:
        assert self.bus is not None
        return list(self.bus.read_i2c_block_data(int(self.config["address"]), command, length))

    def _read_prom(self) -> list[int]:
        values = []
        for index in range(8):
            data = self._read_block(CMD_PROM_READ_BASE + index * 2, 2)
            values.append((data[0] << 8) | data[1])
        if all(value == 0 for value in values[1:7]):
            raise RuntimeError("MS5837 PROM C1-C6 are all zero")
        return values

    def _read_adc(self) -> int:
        data = self._read_block(CMD_ADC_READ, 3)
        return (data[0] << 16) | (data[1] << 8) | data[2]

    def _convert_and_read(self, command: int) -> int:
        self._write_byte(command)
        time.sleep(0.02)
        return self._read_adc()

    def _read_pressure_temperature(self) -> tuple[float, float, int, int]:
        d1 = self._convert_and_read(CMD_CONVERT_D1_8192)
        d2 = self._convert_and_read(CMD_CONVERT_D2_8192)
        c = self.coefficients

        dt = d2 - c[5] * 256.0
        temp = 2000.0 + dt * c[6] / 8388608.0
        off = c[2] * 65536.0 + c[4] * dt / 128.0
        sens = c[1] * 32768.0 + c[3] * dt / 256.0

        ti = offi = sensi = 0.0
        if temp < 2000.0:
            ti = 3.0 * dt * dt / 8589934592.0
            offi = 3.0 * (temp - 2000.0) * (temp - 2000.0) / 2.0
            sensi = 5.0 * (temp - 2000.0) * (temp - 2000.0) / 8.0
            if temp < -1500.0:
                offi += 7.0 * (temp + 1500.0) * (temp + 1500.0)
                sensi += 4.0 * (temp + 1500.0) * (temp + 1500.0)

        temp -= ti
        off -= offi
        sens -= sensi
        pressure_raw = (d1 * sens / 2097152.0 - off) / 8192.0
        pressure_mbar = pressure_raw / 10.0
        return pressure_mbar, temp / 100.0, d1, d2

    def read(self) -> SensorReading:
        if self.simulation:
            return SensorReading(
                self.name,
                True,
                values={"pressure_mbar": 1018.7, "temperature_c": 18.4, "depth_m": 0.54},
            )
        if not self.is_open:
            return self.error("sensor is not open")
        try:
            pressure_mbar, temperature_c, d1, d2 = self._read_pressure_temperature()
            surface = self.surface_pressure_mbar or pressure_mbar
            density = float(self.config.get("fluid_density_kg_m3", 1029.0))
            depth_m = max(0.0, (pressure_mbar - surface) * 100.0 / (density * GRAVITY_M_S2))
            return SensorReading(
                self.name,
                True,
                values={
                    "pressure_mbar": pressure_mbar,
                    "temperature_c": temperature_c,
                    "depth_m": depth_m,
                    "raw_d1": d1,
                    "raw_d2": d2,
                },
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
