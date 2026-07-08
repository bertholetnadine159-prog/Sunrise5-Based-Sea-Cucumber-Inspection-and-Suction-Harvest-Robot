from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any

from .base import BaseSensor, SensorReading
from .ds18b20 import DS18B20Sensor
from .ms5837 import MS5837Sensor
from .ultrasonic_usb import UltrasonicUSBSensor
from .veml7700 import VEML7700Sensor


LOGGER = logging.getLogger(__name__)


@dataclass
class SensorSnapshot:
    readings: dict[str, SensorReading] = field(default_factory=dict)

    def ok(self, name: str) -> bool:
        reading = self.readings.get(name)
        return bool(reading and reading.ok)

    def value(self, name: str, key: str, default: Any = None) -> Any:
        reading = self.readings.get(name)
        if not reading or not reading.ok:
            return default
        return reading.values.get(key, default)

    @property
    def front_distance_m(self) -> float | None:
        return self.value("front", "distance_m")


class SensorManager:
    def __init__(self, hardware_config: dict[str, Any], simulation: bool = False) -> None:
        self.config = hardware_config
        self.simulation = simulation
        self.sensors: dict[str, BaseSensor] = {}
        self._build_sensors()

    def _build_sensors(self) -> None:
        rdk = self.config.get("rdk_x5", {})
        for key, cfg in rdk.get("i2c", {}).items():
            if not cfg.get("enabled", True):
                continue
            name = cfg.get("name", key)
            if key.startswith("veml7700"):
                self.sensors[key] = VEML7700Sensor(name, cfg, self.simulation)
            elif key.startswith("ms5837"):
                self.sensors[key] = MS5837Sensor(name, cfg, self.simulation)

        for key, cfg in rdk.get("one_wire", {}).items():
            if cfg.get("enabled", True):
                self.sensors[key] = DS18B20Sensor(cfg.get("name", key), cfg, self.simulation)

        for key, cfg in self.config.get("ultrasonic_usb", {}).items():
            if cfg.get("enabled", True):
                self.sensors[key] = UltrasonicUSBSensor(cfg.get("name", key), cfg, self.simulation)

    def open_all(self) -> None:
        for key, sensor in self.sensors.items():
            try:
                sensor.open()
                LOGGER.info("Opened sensor %s", key)
            except Exception as exc:
                LOGGER.error("Failed to open sensor %s: %s", key, exc)

    def read_all(self) -> SensorSnapshot:
        snapshot = SensorSnapshot()
        for key, sensor in self.sensors.items():
            try:
                snapshot.readings[key] = sensor.read()
            except Exception as exc:
                snapshot.readings[key] = SensorReading(sensor.name, False, message=str(exc))
        return snapshot

    def close_all(self) -> None:
        for sensor in self.sensors.values():
            sensor.close()
