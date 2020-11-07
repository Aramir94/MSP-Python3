from msp.data_structures.data_structure import DataStructure


class Status(DataStructure):
    def __init__(self):
        self.cycleTime = 0
        """
        | type: uint8   
        | unit: microseconds
        """
        self.i2c_errors_count = 0
        self.sensor = 0
        self.flag = 0
        self.global_conf = 0

        self.timestamp = 0

    @staticmethod
    def parse(data):
        raise NotImplemented
        status = Status()

        # TODO Turn data into array
        import struct

        temp = bytearray(data[10])

        status.cycleTime = struct.unpack('<H', bytes(data[:2]))[0]
        status.i2c_errors_count = struct.unpack('<H', b'' + bytes(data[2:4]))[0]
        status.sensor = struct.unpack('<H', b'' + bytes(data[4:6]))[0]
        status.flag = struct.unpack('<HH', b'' + bytes(data[6:10]))[0]
        temp = b'' + bytes(data[10])
        status.global_conf = struct.unpack('<B', temp)[0]

        return status
