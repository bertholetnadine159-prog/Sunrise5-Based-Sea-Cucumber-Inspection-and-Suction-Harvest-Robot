from __future__ import annotations

import copy
import logging
from pathlib import Path
from typing import Any

from sea_cucumber_robot.config_loader import ConfigBundle, load_config
from sea_cucumber_robot.control.pixhawk_mavlink import create_pixhawk
from sea_cucumber_robot.control.suction_controller import SuctionController
from sea_cucumber_robot.control.thruster_mixer import ThrusterMixer
from sea_cucumber_robot.logger import setup_logging
from sea_cucumber_robot.mission.mission_state_machine import MissionStateMachine
from sea_cucumber_robot.mission.states import MissionState
from sea_cucumber_robot.safety.emergency_stop import EmergencyStop
from sea_cucumber_robot.sensors.sensor_manager import SensorManager
from sea_cucumber_robot.utils.timing import Rate
from sea_cucumber_robot.vision.camera_manager import CameraManager
from sea_cucumber_robot.vision.segmenter import create_segmenter


LOGGER = logging.getLogger(__name__)


class RobotApp:
    def __init__(self, config: ConfigBundle, log_dir: str | Path = "logs", simulation: bool | None = None) -> None:
        self.config = self._with_simulation_override(config, simulation)
        setup_logging(log_dir)
        self.project_root = self.config.root.parent
        self.simulation = bool(self.config.hardware.get("runtime", {}).get("simulation", False))

        self.sensor_manager = SensorManager(self.config.hardware, simulation=self.simulation)
        self.camera_manager = CameraManager(self.config.hardware.get("cameras", {}), simulation=self.simulation)
        self.segmenter = create_segmenter(self.config.vision, self.project_root, simulation=self.simulation)
        self.pixhawk = create_pixhawk(self.config.control.get("pixhawk", {}), simulation=self.simulation)
        self.mixer = ThrusterMixer(self.config.motors)
        self.suction = SuctionController(self.config.motors, self.pixhawk)
        estop_file = self.config.mission.get("mission", {}).get("emergency_stop_file", "logs/EMERGENCY_STOP")
        self.emergency_stop = EmergencyStop(self.project_root / estop_file)
        self.state_machine = MissionStateMachine(
            self.config.as_dict(),
            self.sensor_manager,
            self.camera_manager,
            self.segmenter,
            self.pixhawk,
            self.mixer,
            self.suction,
            self.emergency_stop,
        )

    @staticmethod
    def _with_simulation_override(config: ConfigBundle, simulation: bool | None) -> ConfigBundle:
        if simulation is None:
            return config
        hardware = copy.deepcopy(config.hardware)
        control = copy.deepcopy(config.control)
        hardware.setdefault("runtime", {})["simulation"] = simulation
        control.setdefault("pixhawk", {})["simulation"] = simulation
        return ConfigBundle(config.root, hardware, config.motors, config.vision, control, config.mission)

    @classmethod
    def from_config_dir(cls, config_dir: str | Path = "config", log_dir: str | Path = "logs", simulation: bool | None = None) -> "RobotApp":
        return cls(load_config(config_dir), log_dir=log_dir, simulation=simulation)

    def run(self, max_steps: int | None = None) -> MissionState:
        loop_hz = float(self.config.mission.get("mission", {}).get("loop_hz", 15))
        rate = Rate(loop_hz)
        step_count = 0
        LOGGER.info("Robot app starting simulation=%s", self.simulation)
        try:
            while True:
                result = self.state_machine.step()
                LOGGER.debug("Mission state=%s message=%s", result.state.value, result.message)
                step_count += 1
                if result.state.terminal:
                    return result.state
                if max_steps is not None and step_count >= max_steps:
                    return result.state
                rate.sleep()
        finally:
            self.camera_manager.close_all()
            self.sensor_manager.close_all()
            self.pixhawk.close()


def run_from_config(config_dir: str | Path = "config", log_dir: str | Path = "logs", simulation: bool | None = None) -> MissionState:
    return RobotApp.from_config_dir(config_dir, log_dir=log_dir, simulation=simulation).run()
