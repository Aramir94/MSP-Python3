from struct import unpack

from msp.data_structures.data_structure import DataStructure
from msp.message_ids import MessageIDs

class Identification(DataStructure):
    def __init__(self):
        super().__init__(MessageIDs.IDENT)
        self.version = 0
        self.multi_type = 0
        self.msp_version = 0
        self.capability = 0

    @staticmethod
    def parse(data):
        identification = Identification()

        identification.version = unpack('<B', bytes([data[0]]))[0]
        identification.multi_type = unpack('<B', bytes([data[1]]))[0]
        identification.msp_version = unpack('<B', bytes([data[2]]))[0]
        identification.capability = unpack('<I', bytes(data[3:6]))[0]

        return identification
