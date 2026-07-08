from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any

import numpy as np


LOGGER = logging.getLogger(__name__)


@dataclass
class CameraFrame:
    camera_id: str
    image: np.ndarray
    ok: bool
    message: str = ""


class CameraDevice:
    def __init__(self, camera_id: str, config: dict[str, Any], simulation: bool = False) -> None:
        self.camera_id = camera_id
        self.config = config
        self.simulation = simulation
        self.capture = None
        self.is_open = False
        self._frame_count = 0

    def open(self) -> None:
        if self.is_open:
            return
        if self.simulation:
            self.is_open = True
            return
        import cv2

        device = self.config.get("device", 0)
        self.capture = cv2.VideoCapture(device)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.config.get("width", 1280)))
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.config.get("height", 720)))
        self.capture.set(cv2.CAP_PROP_FPS, int(self.config.get("fps", 30)))
        if not self.capture.isOpened():
            raise RuntimeError(f"camera {self.camera_id} failed to open device {device}")
        self.is_open = True

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
        self.capture = None
        self.is_open = False

    def read(self) -> CameraFrame:
        if not self.is_open:
            return CameraFrame(self.camera_id, np.zeros((1, 1, 3), dtype=np.uint8), False, "camera is closed")
        if self.simulation:
            return CameraFrame(self.camera_id, self._simulated_frame(), True)
        assert self.capture is not None
        ok, frame = self.capture.read()
        if not ok or frame is None:
            return CameraFrame(self.camera_id, np.zeros((1, 1, 3), dtype=np.uint8), False, "read failed")
        return CameraFrame(self.camera_id, frame, True)

    def _simulated_frame(self) -> np.ndarray:
        import cv2

        width = int(self.config.get("width", 1280))
        height = int(self.config.get("height", 720))
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:, :] = (38, 58, 72)
        self._frame_count += 1

        offset = max(0, 160 - self._frame_count * 4)
        if self.camera_id == "camera_1":
            center = (width // 2 + offset, height // 2 + 40)
            axes = (90, 45)
        else:
            center = (width // 2 + max(0, offset // 3), height // 2)
            axes = (120, 70)
        cv2.ellipse(image, center, axes, 0, 0, 360, (28, 120, 180), -1)
        cv2.line(image, (width // 2, 0), (width // 2, height), (80, 130, 80), 1)
        cv2.circle(image, (width // 2, height // 2), 6, (80, 220, 80), -1)
        return image


class CameraManager:
    def __init__(self, camera_config: dict[str, Any], simulation: bool = False) -> None:
        self.cameras = {
            key: CameraDevice(key, cfg, simulation)
            for key, cfg in camera_config.items()
            if cfg.get("enabled", True)
        }

    def initialize_defaults(self) -> None:
        for camera_id, camera in self.cameras.items():
            if camera.config.get("default_open", False):
                camera.open()
                LOGGER.info("Opened default camera %s", camera_id)

    def open_camera(self, camera_id: str) -> None:
        self.cameras[camera_id].open()

    def close_camera(self, camera_id: str) -> None:
        self.cameras[camera_id].close()

    def switch_to(self, camera_id: str) -> None:
        for key, camera in self.cameras.items():
            if key == camera_id:
                camera.open()
            else:
                camera.close()

    def read(self, camera_id: str) -> CameraFrame:
        return self.cameras[camera_id].read()

    def close_all(self) -> None:
        for camera in self.cameras.values():
            camera.close()
