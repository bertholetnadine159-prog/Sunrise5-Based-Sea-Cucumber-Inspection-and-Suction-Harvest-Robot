#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from sea_cucumber_robot.config_loader import load_config
from sea_cucumber_robot.control.pixhawk_mavlink import create_pixhawk
from sea_cucumber_robot.control.thruster_mixer import ThrusterMixer


def main() -> None:
    parser = argparse.ArgumentParser(description="Check Pixhawk connection and send neutral PWM")
    parser.add_argument("--config-dir", default=str(PROJECT_ROOT / "config"))
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config_dir)
    pixhawk = create_pixhawk(config.control.get("pixhawk", {}), simulation=args.simulate)
    mixer = ThrusterMixer(config.motors)
    pixhawk.connect()
    try:
        print(f"connected={pixhawk.connected()}")
        pixhawk.set_many_pwm(mixer.neutral_pwm())
        print("neutral PWM sent to MAIN OUT thrusters")
    finally:
        pixhawk.close()


if __name__ == "__main__":
    main()
