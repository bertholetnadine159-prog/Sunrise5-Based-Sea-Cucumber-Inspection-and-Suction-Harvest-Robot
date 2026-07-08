from __future__ import annotations

from pathlib import Path
from typing import Any

from .base import BaseSensor, SensorReading


class DS18B20Sensor(BaseSensor):
    def __init__(self, name: str, config: dict[str, Any], simulation: bool = False) -> None:
        super().__init__(name, config, simulation)
        self.device_file: Path | None = None

    def open(self) -> None:
        if self.simulation:
            self.is_open = True
            return

        root = Path(self.config.get("sysfs_root", "/sys/bus/w1/devices"))
        device_id = self.config.get("device_id")
        if device_id:
            candidate = root / str(device_id) / "w1_slave"
        else:
            matches = sorted(root.glob("28-*/w1_slave"))
            candidate = matches[0] if matches else root / "missing" / "w1_slave"

        if not candidate.exists():
            raise FileNotFoundError(
                f"DS18B20 sysfs file not found for {self.name}: {candidate}. "
                "Enable 1-Wire and verify the configured GPIO/data pin."
            )
        self.device_file = candidate
        self.is_open = True

    def read(self) -> SensorReading:
        if self.simulation:
            return SensorReading(self.name, True, values={"temperature_c": 18.8})
        if not self.is_open or self.device_file is None:
            return self.error("sensor is not open")
        try:
            lines = self.device_file.read_text(encoding="utf-8").splitlines()
            if len(lines) < 2 or not lines[0].strip().endswith("YES"):
                return self.error("CRC is not ready")
            marker = "t="
            if marker not in lines[1]:
                return self.error("temperature marker not found")
            milli_c = int(lines[1].split(marker, 1)[1])
            return SensorReading(self.name, True, values={"temperature_c": milli_c / 1000.0})
        except Exception as exc:
            return self.error(str(exc))
