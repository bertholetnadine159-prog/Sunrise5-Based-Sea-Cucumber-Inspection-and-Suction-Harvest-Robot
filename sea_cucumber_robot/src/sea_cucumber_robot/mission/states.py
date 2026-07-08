from __future__ import annotations

from enum import Enum


class MissionState(str, Enum):
    INIT = "INIT"
    SEARCH_WITH_CAMERA_1 = "SEARCH_WITH_CAMERA_1"
    ALIGN_WITH_CAMERA_1 = "ALIGN_WITH_CAMERA_1"
    APPROACH_TO_5_5CM = "APPROACH_TO_5_5CM"
    SWITCH_TO_CAMERA_2 = "SWITCH_TO_CAMERA_2"
    ALIGN_WITH_CAMERA_2 = "ALIGN_WITH_CAMERA_2"
    SUCTION_CAPTURE = "SUCTION_CAPTURE"
    COMPLETE = "COMPLETE"
    EMERGENCY_STOP = "EMERGENCY_STOP"

    @property
    def terminal(self) -> bool:
        return self in {MissionState.COMPLETE, MissionState.EMERGENCY_STOP}
