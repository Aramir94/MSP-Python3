from struct import pack, unpack

from msp.data_structures.data_structure import DataStructure
from msp.message_ids import MessageIDs


class WP(DataStructure):
    def __init__(self):
        super().__init__(MessageIDs.WP)
        self.wp_no = 0
        self.lat = 0
        self.lon = 0
        self.alt_hold = 0
        self.heading = 0
        self.time_to_stay = 0
        self.nav_flag = 0

    @staticmethod
    def parse(data) -> DataStructure:
        wp = WP()

        wp.wp_no = unpack('<b', bytes([data[0]]))[0]
        wp.lat = unpack('<i', bytes(data[1:5]))[0]
        wp.lon = unpack('<i', bytes(data[5:9]))[0]
        wp.alt_hold = unpack('<i', bytes(data[9:13]))[0]
        wp.heading = unpack('<h', bytes(data[13:15]))[0]
        wp.time_to_stay = unpack('<h', bytes(data[15:17]))[0]
        wp.nav_flag = unpack('<b', bytes(data[17]))[0]

        return wp

    def serialize(self, setter=False) -> bytes:
        # Check if Setting or Getting
        if not setter:
            # If getting use super's serialize
            return super().serialize()

        # Serialize Data
        result = int(18).to_bytes(1, 'little')
        result += int(MessageIDs.SET_WP).to_bytes(1, 'little')

        # Serialize Checksum
        result = DataStructure.get_header() + result + DataStructure.perform_checksum(result)

        # Return result
        return result



