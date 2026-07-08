#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from sea_cucumber_robot.config_loader import load_config
from sea_cucumber_robot.vision.camera_manager import CameraManager


def main() -> None:
    parser = argparse.ArgumentParser(description="Open configured USB cameras and read one frame")
    parser.add_argument("--config-dir", default=str(PROJECT_ROOT / "config"))
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config_dir)
    manager = CameraManager(config.hardware.get("cameras", {}), simulation=args.simulate)
    try:
        for camera_id in config.hardware.get("cameras", {}):
            manager.open_camera(camera_id)
            frame = manager.read(camera_id)
            print(f"{camera_id}: ok={frame.ok} shape={frame.image.shape} message={frame.message}")
            manager.close_camera(camera_id)
    finally:
        manager.close_all()


if __name__ == "__main__":
    main()
