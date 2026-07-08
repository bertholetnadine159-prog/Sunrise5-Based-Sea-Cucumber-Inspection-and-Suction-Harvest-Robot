from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class MaskGeometry:
    center_x: float
    center_y: float
    area_px: int
    bbox: tuple[int, int, int, int]
    width: int
    height: int

    @property
    def error_x_px(self) -> float:
        return self.center_x - self.width / 2.0

    @property
    def error_y_px(self) -> float:
        return self.center_y - self.height / 2.0

    @property
    def normalized_error_x(self) -> float:
        return self.error_x_px / max(1.0, self.width / 2.0)

    @property
    def normalized_error_y(self) -> float:
        return self.error_y_px / max(1.0, self.height / 2.0)


def mask_to_geometry(mask: np.ndarray, image_shape: tuple[int, int] | tuple[int, int, int]) -> MaskGeometry | None:
    if mask is None:
        return None
    height, width = int(image_shape[0]), int(image_shape[1])
    mask_bool = np.asarray(mask).astype(bool)
    if mask_bool.shape[:2] != (height, width):
        raise ValueError(f"mask shape {mask_bool.shape[:2]} does not match image shape {(height, width)}")

    ys, xs = np.nonzero(mask_bool)
    if len(xs) == 0:
        return None
    area = int(len(xs))
    x1, x2 = int(xs.min()), int(xs.max())
    y1, y2 = int(ys.min()), int(ys.max())
    return MaskGeometry(
        center_x=float(xs.mean()),
        center_y=float(ys.mean()),
        area_px=area,
        bbox=(x1, y1, x2, y2),
        width=width,
        height=height,
    )


def choose_largest_mask(detections: list[Any]) -> Any | None:
    if not detections:
        return None
    return max(detections, key=lambda item: getattr(item, "area_px", 0))
