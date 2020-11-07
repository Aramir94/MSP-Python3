import struct

from msp.data_structures.data_structure import DataStructure


class Altitude(DataStructure):
    def __init__(self):
        self.estalt = 0 # cm
        self.vario = 0  # cm/s

    @staticmethod
    def parse(data):
        altitude = Altitude()

        altitude.estalt = struct.unpack('<i', bytes(data[:4]))[0]
        altitude.vario = struct.unpack('<h', bytes(data[4:]))[0]

        return altitude
