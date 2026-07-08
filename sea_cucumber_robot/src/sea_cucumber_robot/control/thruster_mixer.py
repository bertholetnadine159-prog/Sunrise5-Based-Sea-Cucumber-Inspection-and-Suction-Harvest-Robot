from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from sea_cucumber_robot.utils.math_utils import Vec3, clamp


AXES = ("surge", "sway", "heave", "roll", "pitch", "yaw")


@dataclass(frozen=True)
class MotorConfig:
    motor_id: str
    pixhawk_output: str
    position: Vec3
    direction: Vec3
    type: str
    reversed: bool
    neutral_pwm: int
    min_pwm: int
    max_pwm: int

    @property
    def pixhawk_channel(self) -> int:
        output = self.pixhawk_output.upper()
        if output.startswith("MAIN"):
            return int(output.replace("MAIN", ""))
        if output.startswith("AUX"):
            return 8 + int(output.replace("AUX", ""))
        raise ValueError(f"Unsupported Pixhawk output name: {self.pixhawk_output}")


class ThrusterMixer:
    def __init__(self, motors_config: dict[str, Any]) -> None:
        self.command_pwm_span = int(motors_config.get("mixer", {}).get("command_pwm_span", 350))
        axis_scale = motors_config.get("mixer", {}).get("axes", {})
        self.axis_scale = np.array([float(axis_scale.get(axis, 1.0)) for axis in AXES], dtype=float)
        self.thrusters = [
            self._parse_motor(item)
            for item in motors_config.get("pixhawk_outputs", {}).get("main_out", [])
            if item.get("type") == "thruster"
        ]
        if not self.thrusters:
            raise ValueError("No MAIN OUT thrusters configured")
        self._allocation = self._build_allocation(self.thrusters)
        self._pinv = np.linalg.pinv(self._allocation)

    @staticmethod
    def _parse_motor(config: dict[str, Any]) -> MotorConfig:
        return MotorConfig(
            motor_id=str(config["motor_id"]),
            pixhawk_output=str(config["pixhawk_output"]),
            position=Vec3.from_mapping(config["position"]),
            direction=Vec3.from_mapping(config["direction"]),
            type=str(config["type"]),
            reversed=bool(config.get("reversed", False)),
            neutral_pwm=int(config["neutral_pwm"]),
            min_pwm=int(config["min_pwm"]),
            max_pwm=int(config["max_pwm"]),
        )

    @staticmethod
    def _build_allocation(thrusters: list[MotorConfig]) -> np.ndarray:
        columns = []
        for motor in thrusters:
            torque = motor.position.cross(motor.direction)
            columns.append(
                [
                    motor.direction.x,
                    motor.direction.y,
                    motor.direction.z,
                    torque.x,
                    torque.y,
                    torque.z,
                ]
            )
        return np.array(columns, dtype=float).T

    def mix(self, wrench: dict[str, float]) -> dict[str, float]:
        desired = np.array([float(wrench.get(axis, 0.0)) for axis in AXES], dtype=float)
        desired = desired * self.axis_scale
        commands = self._pinv @ desired
        max_abs = float(np.max(np.abs(commands))) if commands.size else 0.0
        if max_abs > 1.0:
            commands = commands / max_abs
        return {
            motor.motor_id: float(clamp((-value if motor.reversed else value), -1.0, 1.0))
            for motor, value in zip(self.thrusters, commands)
        }

    def commands_to_pwm(self, commands: dict[str, float]) -> dict[int, int]:
        pwm: dict[int, int] = {}
        by_id = {motor.motor_id: motor for motor in self.thrusters}
        for motor_id, command in commands.items():
            motor = by_id[motor_id]
            value = motor.neutral_pwm + int(round(clamp(command, -1.0, 1.0) * self.command_pwm_span))
            pwm[motor.pixhawk_channel] = int(clamp(value, motor.min_pwm, motor.max_pwm))
        return pwm

    def neutral_pwm(self) -> dict[int, int]:
        return {motor.pixhawk_channel: motor.neutral_pwm for motor in self.thrusters}

    def mix_to_pwm(self, wrench: dict[str, float]) -> dict[int, int]:
        return self.commands_to_pwm(self.mix(wrench))
