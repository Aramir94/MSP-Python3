from msp.data_structures.data_structure import DataStructure


class Identification(DataStructure):
    def __init__(self):
        self.version = 0
        self.multi_type = 0
        self.msp_version = 0
        self.capability = 0

        self.timestamp = None

    @staticmethod
    def parse(data):
        identification = Identification()

        identification.version = data[0]
        identification.multi_type = data[1]
        identification.msp_version = data[2]
        identification.capability = data[3]

        return identification
