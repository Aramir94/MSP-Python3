from struct import unpack, pack

from msp.data_structures.data_structure import DataStructure
from msp.message_ids import MessageIDs

class Motor(DataStructure):
    def __init__(self):
        super().__init__(MessageIDs.MOTOR)
        self.m1 = 0
        self.m2 = 0
        self.m3 = 0
        self.m4 = 0

    @staticmethod
    def parse(data):
        motor = Motor()


        motor.m1 = unpack('<H', bytes(data[0:2]))[0]
        motor.m2 = unpack('<H', bytes(data[2:4]))[0]
        motor.m3 = unpack('<H', bytes(data[4:6]))[0]
        motor.m4 = unpack('<H', bytes(data[6:8]))[0]
        # Motor 5 - 8 not used

        return motor

    def serialize(self, setter=False) -> bytes:
        # Check if Setting or Getting
        if not setter:
            # If getting use super's serialize
            return super().serialize()

        # Serialize Data
        result = int(16).to_bytes(1, 'little')
        result += int(MessageIDs.SET_MOTOR).to_bytes(1, 'little')

        # Serialize Checksum
        result = DataStructure.get_header() + result + DataStructure.perform_checksum(result)

        # Return result
        return result
