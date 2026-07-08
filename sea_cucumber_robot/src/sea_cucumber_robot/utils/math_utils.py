from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def deadband(value: float, threshold: float) -> float:
    return 0.0 if abs(value) < threshold else value


def norm2(x: float, y: float) -> float:
    return math.sqrt(x * x + y * y)


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    @classmethod
    def from_mapping(cls, value: dict[str, float]) -> "Vec3":
        return cls(float(value["x"]), float(value["y"]), float(value["z"]))

    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
