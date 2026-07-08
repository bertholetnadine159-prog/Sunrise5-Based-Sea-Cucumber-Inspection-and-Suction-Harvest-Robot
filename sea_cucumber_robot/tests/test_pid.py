from __future__ import annotations

from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from sea_cucumber_robot.control.pid import PID


class PIDTest(unittest.TestCase):
    def test_pid_clamps_output(self) -> None:
        pid = PID(kp=10.0, ki=0.0, kd=0.0, output_limit=0.25)
        self.assertEqual(pid.update_error(1.0, 0.1), 0.25)
        self.assertEqual(pid.update_error(-1.0, 0.1), -0.25)

    def test_pid_derivative_first_update_is_zero(self) -> None:
        pid = PID(kp=0.0, ki=0.0, kd=1.0, output_limit=10.0)
        self.assertEqual(pid.update_error(5.0, 0.1), 0.0)


if __name__ == "__main__":
    unittest.main()
