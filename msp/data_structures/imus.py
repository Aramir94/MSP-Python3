from struct import unpack

from msp.data_structures.data_structure import DataStructure
from msp.message_ids import MessageIDs


class IMU(DataStructure):
    def __init__(self):
        super().__init__(MessageIDs.RAW_IMU)
        self.ax = 0
        self.ay = 0
        self.az = 0

        self.gx = 0
        self.gy = 0
        self.gz = 0

        self.mx = 0
        self.my = 0
        self.mz = 0

    @staticmethod
    def parse(data):
        imu = IMU()

        imu.ax = unpack('<h', bytes(data[:2]))[0]
        imu.ay = unpack('<h', bytes(data[2:4]))[0]
        imu.az = unpack('<h', bytes(data[4:6]))[0]

        imu.gx = unpack('<h', bytes(data[6:8]))[0]
        imu.gy = unpack('<h', bytes(data[8:10]))[0]
        imu.gz = unpack('<h', bytes(data[10:12]))[0]

        imu.mx = unpack('<h', bytes(data[12:14]))[0]
        imu.my = unpack('<h', bytes(data[14:16]))[0]
        imu.mz = unpack('<h', bytes(data[16:18]))[0]

        return imu