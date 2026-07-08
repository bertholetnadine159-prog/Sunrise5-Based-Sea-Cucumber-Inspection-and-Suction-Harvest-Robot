from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

from sea_cucumber_robot.control.alignment_controller import AlignmentController
from sea_cucumber_robot.control.pixhawk_mavlink import PixhawkInterface
from sea_cucumber_robot.control.suction_controller import SuctionController
from sea_cucumber_robot.control.thruster_mixer import ThrusterMixer
from sea_cucumber_robot.safety.emergency_stop import EmergencyStop
from sea_cucumber_robot.sensors.sensor_manager import SensorManager, SensorSnapshot
from sea_cucumber_robot.utils.math_utils import clamp
from sea_cucumber_robot.utils.timing import StableTimer
from sea_cucumber_robot.vision.camera_manager import CameraManager
from sea_cucumber_robot.vision.mask_geometry import choose_largest_mask
from sea_cucumber_robot.vision.segmenter import BaseSegmenter, Detection

from .states import MissionState


LOGGER = logging.getLogger(__name__)


@dataclass
class MissionStepResult:
    state: MissionState
    snapshot: SensorSnapshot | None
    detection: Detection | None
    message: str = ""


class MissionStateMachine:
    def __init__(
        self,
        config: dict[str, Any],
        sensor_manager: SensorManager,
        camera_manager: CameraManager,
        segmenter: BaseSegmenter,
        pixhawk: PixhawkInterface,
        mixer: ThrusterMixer,
        suction: SuctionController,
        emergency_stop: EmergencyStop,
    ) -> None:
        self.config = config
        self.sensor_manager = sensor_manager
        self.camera_manager = camera_manager
        self.segmenter = segmenter
        self.pixhawk = pixhawk
        self.mixer = mixer
        self.suction = suction
        self.emergency_stop = emergency_stop

        self.state = MissionState.INIT
        self.state_enter_time_s = time.monotonic()
        self.last_step_time_s = time.monotonic()
        self.initialized = False
        self.suction_start_s: float | None = None

        control = config["control"]
        self.align_camera_1 = AlignmentController(control["alignment"]["camera_1"])
        self.align_camera_2 = AlignmentController(control["alignment"]["camera_2"])
        approach = control["approach"]
        self.approach_hold_timer = StableTimer(float(approach.get("hold_time_s", 0.4)))

    def transition(self, next_state: MissionState, reason: str = "") -> None:
        if next_state == self.state:
            return
        LOGGER.info("Mission transition %s -> %s %s", self.state.value, next_state.value, reason)
        self.state = next_state
        self.state_enter_time_s = time.monotonic()
        if next_state == MissionState.ALIGN_WITH_CAMERA_1:
            self.align_camera_1.reset()
        elif next_state == MissionState.ALIGN_WITH_CAMERA_2:
            self.align_camera_2.reset()
        elif next_state == MissionState.APPROACH_TO_5_5CM:
            self.approach_hold_timer.reset()
        elif next_state == MissionState.SUCTION_CAPTURE:
            self.suction_start_s = time.monotonic()
            power = float(self.config["control"]["suction"].get("power_percent", 50))
            self.suction.set_suction_power(power)
        elif next_state == MissionState.SWITCH_TO_CAMERA_2:
            self.camera_manager.switch_to("camera_2")
        elif next_state == MissionState.COMPLETE:
            self._stop_motion_and_suction()
        elif next_state == MissionState.EMERGENCY_STOP:
            self._stop_motion_and_suction()

    def step(self) -> MissionStepResult:
        now = time.monotonic()
        dt_s = max(1e-3, now - self.last_step_time_s)
        self.last_step_time_s = now

        if self.emergency_stop.latched and self.state != MissionState.EMERGENCY_STOP:
            self.transition(MissionState.EMERGENCY_STOP, self.emergency_stop.reason)

        if self.state != MissionState.EMERGENCY_STOP and self._state_timed_out():
            self.emergency_stop.trigger(f"state timeout: {self.state.value}")
            self.transition(MissionState.EMERGENCY_STOP, "timeout")

        if self.state == MissionState.INIT:
            return self._handle_init()

        snapshot = self.sensor_manager.read_all()

        if self.state == MissionState.SEARCH_WITH_CAMERA_1:
            return self._handle_search(snapshot)
        if self.state == MissionState.ALIGN_WITH_CAMERA_1:
            return self._handle_align_camera_1(snapshot, dt_s)
        if self.state == MissionState.APPROACH_TO_5_5CM:
            return self._handle_approach(snapshot, dt_s)
        if self.state == MissionState.SWITCH_TO_CAMERA_2:
            return self._handle_switch_to_camera_2(snapshot)
        if self.state == MissionState.ALIGN_WITH_CAMERA_2:
            return self._handle_align_camera_2(snapshot, dt_s)
        if self.state == MissionState.SUCTION_CAPTURE:
            return self._handle_suction(snapshot)
        if self.state == MissionState.COMPLETE:
            return MissionStepResult(self.state, snapshot, None, "mission complete")
        if self.state == MissionState.EMERGENCY_STOP:
            return MissionStepResult(self.state, snapshot, None, self.emergency_stop.reason)
        raise RuntimeError(f"Unhandled mission state: {self.state}")

    def _handle_init(self) -> MissionStepResult:
        try:
            self.sensor_manager.open_all()
            self.pixhawk.connect()
            if not self.pixhawk.connected():
                raise RuntimeError("Pixhawk connection failed")
            self.camera_manager.initialize_defaults()
            self.suction.servo_safe(int(self.config["control"]["suction"].get("servo_safe_pwm", 1500)))
            self.pixhawk.set_many_pwm(self.mixer.neutral_pwm())
            self.initialized = True
            self.transition(MissionState.SEARCH_WITH_CAMERA_1, "init complete")
            return MissionStepResult(self.state, None, None, "initialized")
        except Exception as exc:
            self.emergency_stop.trigger(f"INIT failed: {exc}")
            self.transition(MissionState.EMERGENCY_STOP, "init failure")
            return MissionStepResult(self.state, None, None, str(exc))

    def _handle_search(self, snapshot: SensorSnapshot) -> MissionStepResult:
        detection = self._detect_from_camera("camera_1")
        if detection is not None:
            self.transition(MissionState.ALIGN_WITH_CAMERA_1, "front camera detected target")
            return MissionStepResult(self.state, snapshot, detection, "target detected")
        search_cfg = self.config["control"]["search"]
        self._send_wrench(
            {
                "surge": float(search_cfg.get("surge_command", 0.0)),
                "sway": 0.0,
                "heave": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": float(search_cfg.get("yaw_scan_command", 0.0)),
            }
        )
        return MissionStepResult(self.state, snapshot, None, "searching")

    def _handle_align_camera_1(self, snapshot: SensorSnapshot, dt_s: float) -> MissionStepResult:
        detection = self._detect_from_camera("camera_1")
        if detection is None:
            self.transition(MissionState.SEARCH_WITH_CAMERA_1, "target lost")
            return MissionStepResult(self.state, snapshot, None, "target lost")
        command = self.align_camera_1.update(detection.geometry, dt_s, use_vertical=False)
        self._send_wrench(command.as_wrench())
        if command.stable:
            self.transition(MissionState.APPROACH_TO_5_5CM, "camera 1 aligned")
        return MissionStepResult(self.state, snapshot, detection, f"align error {command.error_px:.1f}px")

    def _handle_approach(self, snapshot: SensorSnapshot, dt_s: float) -> MissionStepResult:
        detection = self._detect_from_camera("camera_1")
        if detection is None:
            self.transition(MissionState.SEARCH_WITH_CAMERA_1, "target lost during approach")
            return MissionStepResult(self.state, snapshot, None, "target lost")

        approach = self.config["control"]["approach"]
        target = float(approach.get("target_distance_m", 0.055))
        tolerance = float(approach.get("distance_tolerance_m", 0.008))
        slow_zone = float(approach.get("slow_zone_m", 0.16))
        min_cmd = float(approach.get("min_forward_command", 0.06))
        max_cmd = float(approach.get("max_forward_command", 0.22))
        distance = snapshot.front_distance_m

        if distance is None:
            surge = min_cmd
            reached = False
            message = "front ultrasonic unavailable, cautious approach"
        else:
            distance_error = distance - target
            reached = distance_error <= tolerance
            if reached:
                surge = 0.0
            else:
                ratio = clamp(distance_error / max(1e-3, slow_zone - target), 0.0, 1.0)
                surge = min_cmd + ratio * (max_cmd - min_cmd)
            message = f"approach distance={distance:.3f}m target={target:.3f}m"

        aligned = self.align_camera_1.update(detection.geometry, dt_s, use_vertical=False, surge=surge)
        self._send_wrench(aligned.as_wrench())
        if self.approach_hold_timer.update(reached):
            self.camera_manager.close_camera("camera_1")
            self.camera_manager.open_camera("camera_2")
            self.transition(MissionState.SWITCH_TO_CAMERA_2, "reached 5.5cm")
        return MissionStepResult(self.state, snapshot, detection, message)

    def _handle_switch_to_camera_2(self, snapshot: SensorSnapshot) -> MissionStepResult:
        low_speed = float(self.config["control"]["suction"].get("low_speed_surge", 0.05))
        self._send_wrench({"surge": low_speed, "sway": 0.0, "heave": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
        detection = self._detect_from_camera("camera_2")
        if detection is not None:
            self.transition(MissionState.ALIGN_WITH_CAMERA_2, "suction camera detected target")
            return MissionStepResult(self.state, snapshot, detection, "camera 2 target detected")
        return MissionStepResult(self.state, snapshot, None, "waiting for camera 2 target")

    def _handle_align_camera_2(self, snapshot: SensorSnapshot, dt_s: float) -> MissionStepResult:
        detection = self._detect_from_camera("camera_2")
        if detection is None:
            return MissionStepResult(self.state, snapshot, None, "camera 2 target not visible")
        command = self.align_camera_2.update(detection.geometry, dt_s, use_vertical=True)
        self._send_wrench(command.as_wrench())
        if command.stable:
            self.transition(MissionState.SUCTION_CAPTURE, "camera 2 aligned")
        return MissionStepResult(self.state, snapshot, detection, f"camera 2 align error {command.error_px:.1f}px")

    def _handle_suction(self, snapshot: SensorSnapshot) -> MissionStepResult:
        low_speed = float(self.config["control"]["suction"].get("low_speed_surge", 0.05))
        self._send_wrench({"surge": low_speed, "sway": 0.0, "heave": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
        duration = float(self.config["mission"]["mission"].get("suction_duration_s", 5.0))
        elapsed = time.monotonic() - (self.suction_start_s or time.monotonic())
        if elapsed >= duration:
            self.transition(MissionState.COMPLETE, "suction duration complete")
        return MissionStepResult(self.state, snapshot, None, f"suction elapsed={elapsed:.1f}s")

    def _detect_from_camera(self, camera_id: str) -> Detection | None:
        frame = self.camera_manager.read(camera_id)
        if not frame.ok:
            LOGGER.warning("Camera %s read failed: %s", camera_id, frame.message)
            return None
        detections = self.segmenter.predict(frame.image)
        return choose_largest_mask(detections)

    def _send_wrench(self, wrench: dict[str, float]) -> None:
        pwm = self.mixer.mix_to_pwm(wrench)
        self.pixhawk.set_many_pwm(pwm)

    def _stop_motion_and_suction(self) -> None:
        try:
            self.pixhawk.set_many_pwm(self.mixer.neutral_pwm())
        except Exception as exc:
            LOGGER.error("Failed to neutral thrusters: %s", exc)
        try:
            self.suction.stop_suction()
            self.suction.servo_safe(int(self.config["control"]["suction"].get("servo_safe_pwm", 1500)))
        except Exception as exc:
            LOGGER.error("Failed to stop suction outputs: %s", exc)

    def _state_timed_out(self) -> bool:
        timeouts = self.config["mission"]["mission"].get("max_state_duration_s", {})
        limit = timeouts.get(self.state.value)
        if not limit:
            return False
        return time.monotonic() - self.state_enter_time_s > float(limit)
