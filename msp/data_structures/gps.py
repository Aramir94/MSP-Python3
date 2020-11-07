from struct import unpack

from msp.data_structures.data_structure import DataStructure
from msp.message_ids import MessageIDs

class GPS(DataStructure):
    def __init__(self):
        super().__init__(MessageIDs.RAW_GPS)
        self.fix = False
        self.numSat = 0
        self.lat = 0
        self.lon = 0
        self.altitude = 0       # meter
        self.speed = 0          # cm/s
        self.ground_course = 0  # degree*10

    @staticmethod
    def parse(data):
        gps = GPS()

        gps.fix = data[0]
        gps.numSat = data[1]
        gps.lat = data[2]
        gps.lon = data[3]
        gps.altitude = data[4]
        gps.speed = data[5]
        gps.ground_course = data[6]

        return gps

    def serialize(self, setter=False) -> bytes:
        # Check if Setting or Getting
        if not setter:
            # If getting use super's serialize
            return super().serialize()

        # Serialize Data
        result = int(14).to_bytes(1, 'little')
        result += int(MessageIDs.SET_RAW_GPS).to_bytes(1, 'little')

        # Serialize Checksum
        result = DataStructure.get_header() + result + DataStructure.perform_checksum(result)

        # Return result
        return result
