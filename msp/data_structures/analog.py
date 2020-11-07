from typing import List
from struct import unpack

from msp.data_structures.data_structure import DataStructure
from msp.message_ids import MessageIDs


class Analog(DataStructure):
    def __init__(self):
        super().__init__(MessageIDs.ANALOG)
        self.vbat = 0
        self.intPowerMeterSum = 0
        self.rssi = 0
        self.amperage = 0

    @staticmethod
    def parse(data: List[int]) -> DataStructure:
        analog = Analog()

        analog.vbat = unpack('<b', bytes([data[0]]))[0]
        analog.intPowerMeterSum = unpack('<h', bytes(data[1:3]))[0]
        analog.rssi = unpack('<h', bytes(data[3:5]))[0]
        analog.amperage = unpack('<h'< bytes(data[5:7]))[0]

        return analog
