from struct import pack, unpack

from msp.data_structures.data_structure import DataStructure
from msp.message_ids import MessageIDs


class RCTuning(DataStructure):
    def __init__(self):
        super().__init__(MessageIDs.RC_TUNING)
        self.rc_rate = 0
        self.rc_expo = 0
        self.roll_pitch_rate = 0
        self.yaw_rate = 0
        self.dyn_thr_pid = 0
        self.throttle_mid = 0
        self.throttle_expo = 0

    @staticmethod
    def parse(data) -> DataStructure:
        rc_tuning = RCTuning()

        if len(data) != 0:
            rc_tuning.rc_rate = unpack('<B', bytes([data[0]]))[0]
            rc_tuning.rc_expo = unpack('<B', bytes([data[1]]))[0]
            rc_tuning.roll_pitch_rate = unpack('<B', bytes([data[2]]))[0]
            rc_tuning.yaw_rate = unpack('<B', bytes([data[3]]))[0]
            rc_tuning.dyn_thr_pid = unpack('<B', bytes([data[4]]))[0]
            rc_tuning.throttle_mid = unpack('<B', bytes([data[5]]))[0]
            rc_tuning.throttle_expo = unpack('<B', bytes([data[6]]))[0]

        return rc_tuning

    def serialize(self, data=None) -> bytes:
        # Check if Setting or Getting
        if not data:
            # If getting use super's serialize
            return super().serialize()

        # Serialize Data
        result = int(7).to_bytes(1, 'little')
        result += int(MessageIDs.SET_RC_TUNING).to_bytes(1, 'little')

        # Serialize Checksum
        result = DataStructure.get_header() + result + DataStructure.perform_checksum(result)

        # Return result
        return result