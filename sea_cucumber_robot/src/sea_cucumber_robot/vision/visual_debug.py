from __future__ import annotations

from pathlib import Path

import numpy as np

from .segmenter import Detection


def draw_debug_overlay(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
    import cv2

    output = frame.copy()
    height, width = output.shape[:2]
    cv2.line(output, (width // 2, 0), (width // 2, height), (0, 220, 0), 1)
    cv2.line(output, (0, height // 2), (width, height // 2), (0, 220, 0), 1)
    for detection in detections:
        mask = detection.mask.astype(bool)
        color = np.zeros_like(output)
        color[:, :] = (0, 120, 255)
        output[mask] = (0.55 * output[mask] + 0.45 * color[mask]).astype(np.uint8)
        x1, y1, x2, y2 = detection.geometry.bbox
        center = (int(detection.geometry.center_x), int(detection.geometry.center_y))
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 180, 255), 2)
        cv2.circle(output, center, 5, (0, 0, 255), -1)
        cv2.putText(
            output,
            f"{detection.class_name} {detection.score:.2f}",
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
    return output


def save_debug_frame(path: str | Path, frame: np.ndarray) -> None:
    import cv2

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(target), frame)
