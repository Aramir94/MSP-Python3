
class IMU:
    def __init__(self):
        self.ax = 0
        self.ay = 0
        self.az = 0

        self.gx = 0
        self.gy = 0
        self.gz = 0

        self.mx = 0
        self.my = 0
        self.mz = 0

        self.timestamp = None

    def parse(self, data):
        self.ax = data[0]
        self.ay = data[1]
        self.az = data[2]

        self.gx = data[3]
        self.gy = data[4]
        self.gz = data[5]

        self.mx = data[6]
        self.my = data[7]
        self.mz = data[8]

        self.timestamp = None

    def get(self):
        imu = [
            self.ax,
            self.ay,
            self.az,
            self.gx,
            self.gy,
            self.gz,
            self.mx,
            self.my,
            self.mz
        ]

        return imu

