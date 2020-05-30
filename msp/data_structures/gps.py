
class GPS:
    def __init__(self):
        self.fix = False
        self.numSat = 0
        self.lat = 0
        self.lon = 0
        self.altitude = 0       # meter
        self.speed = 0          # cm/s
        self.ground_course = 0  # degree*10

        self.timestamp = 0

    def parse(self, data):
        self.fix = data[0]
        self.numSat = data[1]
        self.lat = data[2]
        self.lon = data[3]
        self.altitude = data[4]
        self.speed = data[5]
        self.ground_course = data[6]

        self.timestamp = 0

    def get(self):
        gps = [
            self.fix,
            self.numSat,
            self.lat,
            self.lon,
            self.altitude,
            self.speed,
            self.ground_course
        ]

        return gps
