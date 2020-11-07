from msp.data_structures.data_structure import DataStructure


class GPS(DataStructure):
    def __init__(self):
        self.fix = False
        self.numSat = 0
        self.lat = 0
        self.lon = 0
        self.altitude = 0       # meter
        self.speed = 0          # cm/s
        self.ground_course = 0  # degree*10

        self.timestamp = 0

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