from __future__ import annotations

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from sea_cucumber_robot.config_loader import load_config
from sea_cucumber_robot.control.thruster_mixer import ThrusterMixer


class ThrusterMixerTest(unittest.TestCase):
    def test_mixer_returns_all_thruster_pwm_channels(self) -> None:
        config = load_config(Path(__file__).resolve().parents[1] / "config")
        mixer = ThrusterMixer(config.motors)
        pwm = mixer.mix_to_pwm({"surge": 0.1, "sway": 0.0, "heave": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0})
        self.assertEqual(sorted(pwm), [1, 2, 3, 4, 5, 6, 7, 8])
        self.assertTrue(all(1100 <= value <= 1900 for value in pwm.values()))

    def test_neutral_pwm_is_1500_for_all_thrusters(self) -> None:
        config = load_config(Path(__file__).resolve().parents[1] / "config")
        mixer = ThrusterMixer(config.motors)
        self.assertEqual(set(mixer.neutral_pwm().values()), {1500})


if __name__ == "__main__":
    unittest.main()
