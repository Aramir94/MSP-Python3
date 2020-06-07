from typing import List

from msp.data_structures.data_structure import DataStructure


class RCTuning(DataStructure):
    def __init__(self):
        self.rc_rate = 0
        self.rc_expo = 0
        self.roll_pitch_rate = 0
        self.yaw_rate = 0
        self.dyn_thr_pid = 0
        self.throttle_mid = 0
        self.throttle_expo = 0

    @staticmethod
    def parse(data: List[int]) -> DataStructure:
        rc_tuning = RCTuning()

        rc_tuning.rc_rate = 0
        rc_tuning.rc_expo = 0
        rc_tuning.roll_pitch_rate = 0
        rc_tuning.yaw_rate = 0
        rc_tuning.dyn_thr_pid = 0
        rc_tuning.throttle_mid = 0
        rc_tuning.throttle_expo = 0

        return rc_tuning

    def to_array(self) -> List[int]:
        rc_tuning = [
            self.rc_rate,
            self.rc_expo,
            self.roll_pitch_rate,
            self.yaw_rate,
            self.dyn_thr_pid,
            self.throttle_mid,
            self.throttle_expo
        ]

        return rc_tuning
