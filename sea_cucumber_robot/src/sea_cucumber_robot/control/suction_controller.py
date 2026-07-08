from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from sea_cucumber_robot.utils.math_utils import clamp

from .pixhawk_mavlink import PixhawkInterface
from .thruster_mixer import MotorConfig, ThrusterMixer


@dataclass
class AuxiliaryOutput:
    motor_id: str
    channel: int
    type: str
    reversed: bool
    neutral_pwm: int
    min_pwm: int
    max_pwm: int


class SuctionController:
    def __init__(self, motors_config: dict[str, Any], pixhawk: PixhawkInterface) -> None:
        self.pixhawk = pixhawk
        self.outputs = []
        for item in motors_config.get("pixhawk_outputs", {}).get("aux_out", []):
            parsed = ThrusterMixer._parse_motor(item)
            self.outputs.append(
                AuxiliaryOutput(
                    motor_id=parsed.motor_id,
                    channel=parsed.pixhawk_channel,
                    type=parsed.type,
                    reversed=parsed.reversed,
                    neutral_pwm=parsed.neutral_pwm,
                    min_pwm=parsed.min_pwm,
                    max_pwm=parsed.max_pwm,
                )
            )

    def _percent_to_pwm(self, output: AuxiliaryOutput, percent: float) -> int:
        power = clamp(percent / 100.0, 0.0, 1.0)
        if output.reversed:
            power = 1.0 - power
        return int(round(output.min_pwm + power * (output.max_pwm - output.min_pwm)))

    def set_suction_power(self, percent: float) -> None:
        for output in self.outputs:
            if output.type == "suction_motor":
                self.pixhawk.set_servo_pwm(output.channel, self._percent_to_pwm(output, percent))

    def stop_suction(self) -> None:
        for output in self.outputs:
            if output.type == "suction_motor":
                self.pixhawk.set_servo_pwm(output.channel, output.neutral_pwm)

    def servo_safe(self, pwm: int | None = None) -> None:
        for output in self.outputs:
            if output.type == "servo":
                self.pixhawk.set_servo_pwm(output.channel, int(pwm or output.neutral_pwm))
