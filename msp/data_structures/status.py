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
        status = Status()
        status.cycleTime = data[0]
        status.i2c_errors_count = data[1]
        status.sensor = data[2]
        status.flag = data[3]
        status.global_conf = data[4]

        return status

    def to_array(self):
        status = [
            self.cycleTime,
            self.i2c_errors_count,
            self.sensor,
            self.flag,
            self.global_conf
        ]

        return status
