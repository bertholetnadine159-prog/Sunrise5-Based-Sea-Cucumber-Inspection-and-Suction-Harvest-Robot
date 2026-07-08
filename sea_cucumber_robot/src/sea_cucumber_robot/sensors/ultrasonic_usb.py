from __future__ import annotations

import time
from typing import Any

from .base import BaseSensor, SensorReading


UART_HEADER = 0xFF
OUT_OF_WATER_VALUE = 0xFFFB


def uart_checksum(data_h: int, data_l: int) -> int:
    return (UART_HEADER + data_h + data_l) & 0xFF


def parse_ff_uart_frame(raw: bytes) -> tuple[float | None, str]:
    if len(raw) < 4:
        return None, "short frame"
    for index in range(len(raw) - 3):
        if raw[index] != UART_HEADER:
            continue
        frame = raw[index:index + 4]
        _, data_h, data_l, checksum = frame
        if uart_checksum(data_h, data_l) != checksum:
            continue
        distance_mm = data_h * 256 + data_l
        if distance_mm == OUT_OF_WATER_VALUE:
            return None, "out of water value"
        if distance_mm <= 0:
            return None, f"invalid distance {distance_mm} mm"
        return distance_mm / 1000.0, "ok"
    return None, "valid FF frame not found"


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_modbus_crc(payload: bytes) -> bytes:
    crc = modbus_crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def parse_modbus_distance(frame: bytes, address: int) -> tuple[float | None, str]:
    if len(frame) < 7:
        return None, "short modbus frame"
    if frame[0] != address or frame[1] != 0x03:
        return None, "unexpected modbus address/function"
    received_crc = frame[-2] | (frame[-1] << 8)
    if modbus_crc16(frame[:-2]) != received_crc:
        return None, "modbus crc mismatch"
    distance_mm = frame[3] << 8 | frame[4]
    if distance_mm <= 0:
        return None, f"invalid distance {distance_mm} mm"
    return distance_mm / 1000.0, "ok"


class UltrasonicUSBSensor(BaseSensor):
    def __init__(self, name: str, config: dict[str, Any], simulation: bool = False) -> None:
        super().__init__(name, config, simulation)
        self.serial = None
        self._sim_distance_m = 0.35 if config.get("role") == "front_distance_to_target" else 0.7

    def open(self) -> None:
        if self.simulation:
            self.is_open = True
            return
        try:
            import serial
        except Exception as exc:
            raise RuntimeError("pyserial is required for ultrasonic USB sensors") from exc
        self.serial = serial.Serial(
            port=self.config["port"],
            baudrate=int(self.config.get("baudrate", 9600)),
            timeout=float(self.config.get("timeout_s", 0.08)),
        )
        self.is_open = True

    def read(self) -> SensorReading:
        if self.simulation:
            if self.config.get("role") == "front_distance_to_target":
                self._sim_distance_m = max(0.05, self._sim_distance_m - 0.002)
            return SensorReading(self.name, True, values={"distance_m": self._sim_distance_m})
        if not self.is_open or self.serial is None:
            return self.error("sensor is not open")
        try:
            protocol = str(self.config.get("protocol", "ff_uart")).lower()
            if protocol == "ff_uart":
                raw = self.serial.read(8)
                distance_m, message = parse_ff_uart_frame(raw)
            elif protocol == "modbus":
                address = int(self.config.get("address", 1))
                request = append_modbus_crc(bytes([address, 0x03, 0x01, 0x01, 0x00, 0x01]))
                self.serial.write(request)
                time.sleep(0.02)
                raw = self.serial.read(7)
                distance_m, message = parse_modbus_distance(raw, address)
            else:
                return self.error(f"unsupported ultrasonic protocol: {protocol}")

            if distance_m is None:
                return self.error(message)
            min_valid = float(self.config.get("min_valid_m", 0.03))
            max_valid = float(self.config.get("max_valid_m", 4.5))
            if not (min_valid <= distance_m <= max_valid):
                return self.error(f"distance out of range: {distance_m:.3f} m")
            return SensorReading(self.name, True, values={"distance_m": distance_m, "protocol": protocol})
        except Exception as exc:
            return self.error(str(exc))

    def close(self) -> None:
        if self.serial is not None:
            try:
                self.serial.close()
            except Exception:
                pass
        self.serial = None
        self.is_open = False
