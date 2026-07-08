from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from sea_cucumber_robot.vision.mask_geometry import mask_to_geometry


class MaskGeometryTest(unittest.TestCase):
    def test_mask_center_and_error(self) -> None:
        mask = np.zeros((100, 200), dtype=np.uint8)
        mask[40:60, 90:110] = 1
        geometry = mask_to_geometry(mask, (100, 200, 3))
        self.assertIsNotNone(geometry)
        assert geometry is not None
        self.assertEqual(geometry.area_px, 400)
        self.assertLess(abs(geometry.center_x - 99.5), 1e-6)
        self.assertLess(abs(geometry.error_x_px + 0.5), 1e-6)

    def test_empty_mask_returns_none(self) -> None:
        self.assertIsNone(mask_to_geometry(np.zeros((10, 10), dtype=np.uint8), (10, 10, 3)))


if __name__ == "__main__":
    unittest.main()
