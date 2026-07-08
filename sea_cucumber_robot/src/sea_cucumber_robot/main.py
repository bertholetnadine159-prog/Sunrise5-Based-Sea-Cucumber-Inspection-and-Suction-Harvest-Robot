from __future__ import annotations

import argparse
from pathlib import Path

from sea_cucumber_robot.app import RobotApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sea cucumber inspection and suction harvest robot")
    parser.add_argument("--config-dir", default="config", help="Directory containing hardware/motors/vision/control/mission YAML files")
    parser.add_argument("--log-dir", default="logs", help="Directory for runtime logs")
    parser.add_argument("--simulate", action="store_true", help="Use simulated sensors, camera frames and Pixhawk outputs")
    parser.add_argument("--max-steps", type=int, default=None, help="Stop after N state-machine steps for debugging")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app = RobotApp.from_config_dir(Path(args.config_dir), log_dir=args.log_dir, simulation=args.simulate)
    final_state = app.run(max_steps=args.max_steps)
    print(final_state.value)


if __name__ == "__main__":
    main()
