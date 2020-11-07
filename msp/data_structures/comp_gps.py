from struct import unpack

from msp.data_structures.data_structure import DataStructure
from msp.message_ids import MessageIDs


class CompGPS(DataStructure):
    def __init__(self):
        super().__init__(MessageIDs.COMP_GPS)
        self.distance_to_home = 0   # meters
        self.direction_to_home = 0  # degree (-180,180)
        self.update = False         # boolean

    @staticmethod
    def parse(data):
        comp_gps = CompGPS()

        comp_gps.distance_to_home = unpack('<H', bytes(data[0:2]))[0]
        comp_gps.direction_to_home = unpack('<H', bytes(data[2:4]))[0]
        comp_gps.update = unpack('<H', bytes([data[4]]))[0]

        return comp_gps
