from msp.data_structures.data_structure import DataStructure


class IMU(DataStructure):
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

    @staticmethod
    def parse(data):
        imu = IMU()

        imu.ax = data[0]
        imu.ay = data[1]
        imu.az = data[2]

        imu.gx = data[3]
        imu.gy = data[4]
        imu.gz = data[5]

        imu.mx = data[6]
        imu.my = data[7]
        imumz = data[8]

        return imu

    def to_array(self):
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

