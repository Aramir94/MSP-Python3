from typing import List

from msp.data_structures.data_structure import DataStructure


class Analog(DataStructure):
    def __init__(self):
        self.vbat = 0
        self.intPowerMeterSum = 0
        self.rssi = 0
        self.amperage = 0

    @staticmethod
    def parse(data: List[int]) -> DataStructure:
        analog = Analog()

        analog.vbat = data[0]
        analog.intPowerMeterSum = data[1]
        analog.rssi = data[2]
        analog.amperage = data[3]

        return analog
