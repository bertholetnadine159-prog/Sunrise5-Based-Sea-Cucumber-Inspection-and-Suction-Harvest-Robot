from __future__ import annotations

import importlib.util
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .mask_geometry import MaskGeometry, mask_to_geometry


LOGGER = logging.getLogger(__name__)


@dataclass
class Detection:
    class_id: int
    class_name: str
    score: float
    mask: np.ndarray
    geometry: MaskGeometry

    @property
    def area_px(self) -> int:
        return self.geometry.area_px


class BaseSegmenter:
    def predict(self, frame: np.ndarray) -> list[Detection]:
        raise NotImplementedError


class SimulationSegmenter(BaseSegmenter):
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config

    def predict(self, frame: np.ndarray) -> list[Detection]:
        height, width = frame.shape[:2]
        yy, xx = np.ogrid[:height, :width]
        center_x = width * float(self.config.get("center_x_ratio", 0.52))
        center_y = height * float(self.config.get("center_y_ratio", 0.55))
        radius = min(width, height) * float(self.config.get("radius_ratio", 0.12))
        mask = ((xx - center_x) ** 2 + (yy - center_y) ** 2) <= radius ** 2
        geometry = mask_to_geometry(mask, frame.shape)
        if geometry is None:
            return []
        return [Detection(0, "sea_cucumber", 0.99, mask.astype(np.uint8), geometry)]


class ColorThresholdSegmenter(BaseSegmenter):
    def __init__(self, config: dict[str, Any], class_names: list[str]) -> None:
        self.config = config
        self.class_names = class_names or ["sea_cucumber"]

    def predict(self, frame: np.ndarray) -> list[Detection]:
        import cv2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(self.config.get("hsv_lower", [0, 20, 20]), dtype=np.uint8)
        upper = np.array(self.config.get("hsv_upper", [35, 255, 255]), dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper) > 0
        min_area = int(self.config.get("min_area_px", 300))
        geometry = mask_to_geometry(mask, frame.shape)
        if geometry is None or geometry.area_px < min_area:
            return []
        return [Detection(0, self.class_names[0], 0.50, mask.astype(np.uint8), geometry)]


class UltralyticsSegmenter(BaseSegmenter):
    def __init__(self, model_path: str, config: dict[str, Any]) -> None:
        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError("ultralytics is required for backend=ultralytics") from exc
        self.model = YOLO(model_path)
        self.config = config
        self.class_names = config.get("class_names", ["sea_cucumber"])

    def predict(self, frame: np.ndarray) -> list[Detection]:
        import cv2

        conf = float(self.config.get("confidence_threshold", 0.25))
        results = self.model.predict(frame, conf=conf, verbose=False)
        detections: list[Detection] = []
        if not results:
            return detections
        result = results[0]
        if result.masks is None or result.boxes is None:
            return detections

        masks = result.masks.data.cpu().numpy()
        boxes = result.boxes
        for index, mask_small in enumerate(masks):
            mask = cv2.resize(mask_small.astype(np.float32), (frame.shape[1], frame.shape[0])) > 0.5
            geometry = mask_to_geometry(mask, frame.shape)
            if geometry is None:
                continue
            class_id = int(boxes.cls[index].item()) if boxes.cls is not None else 0
            score = float(boxes.conf[index].item()) if boxes.conf is not None else 0.0
            class_name = self.class_names[class_id] if class_id < len(self.class_names) else str(class_id)
            detections.append(Detection(class_id, class_name, score, mask.astype(np.uint8), geometry))
        return detections


class RDKBinSegmenter(BaseSegmenter):
    def __init__(self, model_path: str, config: dict[str, Any], project_root: Path) -> None:
        script_path = Path(config.get("rdk_yolo_script", "../2.py"))
        if not script_path.is_absolute():
            script_path = (project_root / script_path).resolve()
        if not script_path.exists():
            raise FileNotFoundError(f"RDK YOLO script not found: {script_path}")
        spec = importlib.util.spec_from_file_location("rdk_yolo11_seg_runtime", script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Failed to load RDK YOLO script: {script_path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        model_file = Path(model_path)
        if not model_file.is_absolute():
            model_file = (project_root / model_file).resolve()
        self.model = module.YOLO11_Segment(
            str(model_file),
            float(config.get("confidence_threshold", 0.25)),
            float(config.get("iou_threshold", 0.45)),
            mask_thres=float(config.get("mask_threshold", 0.5)),
        )
        self.class_names = config.get("class_names", ["sea_cucumber"])

    def predict(self, frame: np.ndarray) -> list[Detection]:
        input_tensor = self.model.bgr2nv12(frame)
        outputs = self.model.c2numpy(self.model.forward(input_tensor))
        ids, scores, _bboxes, masks = self.model.postProcess(outputs)
        detections: list[Detection] = []
        for class_id, score, mask in zip(ids, scores, masks):
            geometry = mask_to_geometry(mask.astype(bool), frame.shape)
            if geometry is None:
                continue
            class_index = int(class_id)
            class_name = self.class_names[class_index] if class_index < len(self.class_names) else str(class_index)
            detections.append(Detection(class_index, class_name, float(score), mask.astype(np.uint8), geometry))
        return detections


def create_segmenter(config: dict[str, Any], project_root: Path, simulation: bool = False) -> BaseSegmenter:
    seg_cfg = config.get("segmenter", {})
    if simulation or seg_cfg.get("simulation_mask", {}).get("enabled", False):
        return SimulationSegmenter(seg_cfg.get("simulation_mask", {}))

    backend = str(seg_cfg.get("backend", "color_threshold")).lower()
    model_path = str(seg_cfg.get("model_path", "../YOLO11_LBL.bin"))
    if backend == "color_threshold":
        return ColorThresholdSegmenter(seg_cfg.get("color_threshold", {}), seg_cfg.get("class_names", ["sea_cucumber"]))
    if backend == "ultralytics":
        model_file = Path(model_path)
        if not model_file.is_absolute():
            model_file = (project_root / model_file).resolve()
        return UltralyticsSegmenter(str(model_file), seg_cfg)
    if backend == "rdk_bin":
        return RDKBinSegmenter(model_path, seg_cfg, project_root)
    raise ValueError(f"Unsupported segmenter backend: {backend}")
