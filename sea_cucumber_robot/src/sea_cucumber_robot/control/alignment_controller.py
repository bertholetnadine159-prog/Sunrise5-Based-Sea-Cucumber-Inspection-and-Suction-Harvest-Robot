from __future__ import annotations

from dataclasses import dataclass

from sea_cucumber_robot.utils.math_utils import norm2
from sea_cucumber_robot.utils.timing import StableTimer
from sea_cucumber_robot.vision.mask_geometry import MaskGeometry

from .pid import PID


@dataclass
class AlignmentCommand:
    surge: float
    sway: float
    heave: float
    yaw: float
    stable: bool
    error_px: float

    def as_wrench(self) -> dict[str, float]:
        return {
            "surge": self.surge,
            "sway": self.sway,
            "heave": self.heave,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": self.yaw,
        }


class AlignmentController:
    def __init__(self, camera_config: dict) -> None:
        self.horizontal_pid = PID.from_config(camera_config["horizontal_pid"])
        self.vertical_pid = PID.from_config(camera_config["vertical_pid"])
        self.yaw_pid = PID.from_config(camera_config["yaw_pid"])
        self.stable_error_px = float(camera_config.get("stable_error_px", 30))
        self.stable_timer = StableTimer(float(camera_config.get("stable_duration_s", 1.0)))

    def reset(self) -> None:
        self.horizontal_pid.reset()
        self.vertical_pid.reset()
        self.yaw_pid.reset()
        self.stable_timer.reset()

    def update(self, geometry: MaskGeometry, dt_s: float, use_vertical: bool = False, surge: float = 0.0) -> AlignmentCommand:
        error_x_px = geometry.error_x_px
        error_y_px = geometry.error_y_px if use_vertical else 0.0
        error_norm_x = geometry.normalized_error_x
        error_norm_y = geometry.normalized_error_y if use_vertical else 0.0

        sway = self.horizontal_pid.update_error(error_norm_x, dt_s)
        yaw = self.yaw_pid.update_error(error_norm_x, dt_s)
        heave = self.vertical_pid.update_error(error_norm_y, dt_s) if use_vertical else 0.0
        error_px = norm2(error_x_px, error_y_px)
        stable = self.stable_timer.update(error_px <= self.stable_error_px)
        return AlignmentCommand(surge=surge, sway=sway, heave=heave, yaw=yaw, stable=stable, error_px=error_px)
