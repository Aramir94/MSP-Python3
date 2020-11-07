from struct import unpack

from msp.data_structures.data_structure import DataStructure


class Attitude(DataStructure):
    def __init__(self):
        self.angx = 0       # Range [-1800, 1800] 1/10°
        self.angy = 0       # Range [-900, 900] 1/10°
        self.heading = 0    # Range [-180, 180]

        self.timestamp = None

    @staticmethod
    def parse(data):
        attitude = Attitude()

        attitude.angx = unpack('<h', bytes(data[:2]))[0]
        attitude.angy = unpack('<h', bytes(data[2:4]))[0]
        attitude.heading = unpack('<h', bytes(data[4:6]))[0]

        return attitude