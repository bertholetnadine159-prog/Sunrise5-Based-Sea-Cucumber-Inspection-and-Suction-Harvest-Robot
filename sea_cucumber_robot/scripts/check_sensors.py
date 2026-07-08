#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from sea_cucumber_robot.config_loader import load_config
from sea_cucumber_robot.sensors.sensor_manager import SensorManager


def main() -> None:
    parser = argparse.ArgumentParser(description="Check configured sensors once")
    parser.add_argument("--config-dir", default=str(PROJECT_ROOT / "config"))
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config_dir)
    manager = SensorManager(config.hardware, simulation=args.simulate or config.hardware.get("runtime", {}).get("simulation", False))
    manager.open_all()
    try:
        snapshot = manager.read_all()
        for key, reading in snapshot.readings.items():
            print(f"{key}: ok={reading.ok} values={reading.values} message={reading.message}")
    finally:
        manager.close_all()


if __name__ == "__main__":
    main()
