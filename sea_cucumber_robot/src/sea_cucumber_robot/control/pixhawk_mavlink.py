from __future__ import annotations

import logging
import time
from typing import Any


LOGGER = logging.getLogger(__name__)


class PixhawkInterface:
    def connect(self) -> None:
        raise NotImplementedError

    def connected(self) -> bool:
        raise NotImplementedError

    def set_servo_pwm(self, channel: int, pwm: int) -> None:
        raise NotImplementedError

    def set_many_pwm(self, pwm_by_channel: dict[int, int]) -> None:
        for channel, pwm in pwm_by_channel.items():
            self.set_servo_pwm(channel, pwm)

    def close(self) -> None:
        pass


class SimulatedPixhawk(PixhawkInterface):
    def __init__(self) -> None:
        self.outputs: dict[int, int] = {}
        self._connected = False

    def connect(self) -> None:
        self._connected = True
        LOGGER.info("Simulated Pixhawk connected")

    def connected(self) -> bool:
        return self._connected

    def set_servo_pwm(self, channel: int, pwm: int) -> None:
        if not self._connected:
            raise RuntimeError("simulated Pixhawk is not connected")
        self.outputs[int(channel)] = int(pwm)


class MavlinkPixhawk(PixhawkInterface):
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.master = None
        self.target_system = 1
        self.target_component = 1
        self._last_heartbeat_s = 0.0

    def connect(self) -> None:
        try:
            from pymavlink import mavutil
        except Exception as exc:
            raise RuntimeError("pymavlink is required for Pixhawk control") from exc
        self.mavutil = mavutil
        self.master = mavutil.mavlink_connection(
            self.config.get("connection", "/dev/ttyACM0"),
            baud=int(self.config.get("baud", 115200)),
        )
        LOGGER.info("Waiting for Pixhawk heartbeat on %s", self.config.get("connection"))
        self.master.wait_heartbeat(timeout=float(self.config.get("heartbeat_timeout_s", 3.0)))
        self.target_system = self.master.target_system
        self.target_component = self.master.target_component
        self._last_heartbeat_s = time.monotonic()
        LOGGER.info("Pixhawk heartbeat received system=%s component=%s", self.target_system, self.target_component)

    def connected(self) -> bool:
        if self.master is None:
            return False
        timeout = float(self.config.get("heartbeat_timeout_s", 3.0))
        message = self.master.recv_match(type="HEARTBEAT", blocking=False)
        if message is not None:
            self._last_heartbeat_s = time.monotonic()
        return time.monotonic() - self._last_heartbeat_s <= timeout

    def set_servo_pwm(self, channel: int, pwm: int) -> None:
        if self.master is None:
            raise RuntimeError("Pixhawk is not connected")
        self.master.mav.command_long_send(
            self.target_system,
            self.target_component,
            self.mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            float(channel),
            float(pwm),
            0,
            0,
            0,
            0,
            0,
        )

    def close(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
        self.master = None


def create_pixhawk(config: dict[str, Any], simulation: bool = False) -> PixhawkInterface:
    if simulation or config.get("simulation", False) or not config.get("enabled", True):
        return SimulatedPixhawk()
    return MavlinkPixhawk(config)
