from __future__ import annotations

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from sea_cucumber_robot.app import RobotApp
from sea_cucumber_robot.config_loader import load_config
from sea_cucumber_robot.mission.states import MissionState


class StateMachineTest(unittest.TestCase):
    def test_state_machine_reaches_complete_in_simulation(self) -> None:
        config = load_config(Path(__file__).resolve().parents[1] / "config")
        config.vision["segmenter"]["simulation_mask"]["center_x_ratio"] = 0.5
        config.vision["segmenter"]["simulation_mask"]["center_y_ratio"] = 0.5
        config.control["alignment"]["camera_1"]["stable_duration_s"] = 0.0
        config.control["alignment"]["camera_2"]["stable_duration_s"] = 0.0
        config.control["alignment"]["camera_1"]["stable_error_px"] = 1000
        config.control["alignment"]["camera_2"]["stable_error_px"] = 1000
        config.control["approach"]["hold_time_s"] = 0.0
        config.mission["mission"]["suction_duration_s"] = 0.0
        app = RobotApp(config, log_dir=Path(__file__).resolve().parents[1] / "logs", simulation=True)

        try:
            for _ in range(40):
                if "front" in app.sensor_manager.sensors:
                    app.sensor_manager.sensors["front"]._sim_distance_m = 0.055
                result = app.state_machine.step()
                if result.state == MissionState.COMPLETE:
                    break

            self.assertEqual(app.state_machine.state, MissionState.COMPLETE)
        finally:
            app.camera_manager.close_all()
            app.sensor_manager.close_all()


if __name__ == "__main__":
    unittest.main()
