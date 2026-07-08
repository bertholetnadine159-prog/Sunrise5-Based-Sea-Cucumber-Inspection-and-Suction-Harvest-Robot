from __future__ import annotations

from dataclasses import dataclass

from sea_cucumber_robot.utils.math_utils import clamp


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float | None = None

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.previous_error: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def update_error(self, error: float, dt_s: float) -> float:
        dt = max(1e-3, dt_s)
        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = clamp(self.integral, -self.integral_limit, self.integral_limit)
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / dt
        self.previous_error = error
        value = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(value, -self.output_limit, self.output_limit)

    @classmethod
    def from_config(cls, config: dict) -> "PID":
        return cls(
            kp=float(config.get("kp", 0.0)),
            ki=float(config.get("ki", 0.0)),
            kd=float(config.get("kd", 0.0)),
            output_limit=float(config.get("output_limit", 1.0)),
            integral_limit=config.get("integral_limit"),
        )
