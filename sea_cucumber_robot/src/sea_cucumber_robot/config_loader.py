from __future__ import annotations

import copy
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


CONFIG_FILES = ("hardware.yaml", "motors.yaml", "vision.yaml", "control.yaml", "mission.yaml")


@dataclass(frozen=True)
class ConfigBundle:
    root: Path
    hardware: dict[str, Any]
    motors: dict[str, Any]
    vision: dict[str, Any]
    control: dict[str, Any]
    mission: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return {
            "hardware": self.hardware,
            "motors": self.motors,
            "vision": self.vision,
            "control": self.control,
            "mission": self.mission,
        }


def deep_get(data: Mapping[str, Any], path: str, default: Any = None) -> Any:
    current: Any = data
    for part in path.split("."):
        if not isinstance(current, Mapping) or part not in current:
            return default
        current = current[part]
    return current


def deep_set(data: dict[str, Any], path: str, value: Any) -> None:
    current = data
    parts = path.split(".")
    for part in parts[:-1]:
        current = current.setdefault(part, {})
    current[parts[-1]] = value


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        value = yaml.safe_load(handle) or {}
    if not isinstance(value, dict):
        raise ValueError(f"Config file must contain a mapping: {path}")
    return value


def _coerce_env_value(raw: str) -> Any:
    text = raw.strip()
    lower = text.lower()
    if lower in {"true", "1", "yes", "on"}:
        return True
    if lower in {"false", "0", "no", "off"}:
        return False
    try:
        if "." in text:
            return float(text)
        return int(text)
    except ValueError:
        return raw


def apply_environment_overrides(bundle: ConfigBundle) -> ConfigBundle:
    hardware = copy.deepcopy(bundle.hardware)
    control = copy.deepcopy(bundle.control)
    vision = copy.deepcopy(bundle.vision)

    mapping = {
        "SEA_ROBOT_SIMULATION": ("hardware.runtime.simulation", "control.pixhawk.simulation"),
        "SEA_ROBOT_MAVLINK_CONNECTION": ("control.pixhawk.connection",),
        "SEA_ROBOT_MAVLINK_BAUD": ("control.pixhawk.baud",),
        "SEA_ROBOT_FRONT_CAMERA": ("hardware.cameras.camera_1.device",),
        "SEA_ROBOT_SUCTION_CAMERA": ("hardware.cameras.camera_2.device",),
        "SEA_ROBOT_MODEL_PATH": ("vision.segmenter.model_path",),
    }
    targets = {"hardware": hardware, "control": control, "vision": vision}

    for env_name, paths in mapping.items():
        if env_name not in os.environ:
            continue
        value = _coerce_env_value(os.environ[env_name])
        for path in paths:
            group, local_path = path.split(".", 1)
            deep_set(targets[group], local_path, value)

    return ConfigBundle(
        root=bundle.root,
        hardware=hardware,
        motors=bundle.motors,
        vision=vision,
        control=control,
        mission=bundle.mission,
    )


def load_config(config_dir: str | os.PathLike[str] | None = None) -> ConfigBundle:
    root = Path(config_dir or os.environ.get("SEA_ROBOT_CONFIG_DIR", "config")).resolve()
    loaded = {name: _load_yaml(root / name) for name in CONFIG_FILES}
    bundle = ConfigBundle(
        root=root,
        hardware=loaded["hardware.yaml"],
        motors=loaded["motors.yaml"],
        vision=loaded["vision.yaml"],
        control=loaded["control.yaml"],
        mission=loaded["mission.yaml"],
    )
    return apply_environment_overrides(bundle)
