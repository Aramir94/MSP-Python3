from msp.data_structures.data_structure import DataStructure


class Motor(DataStructure):
    def __init__(self):
        self.m1 = 0
        self.m2 = 0
        self.m3 = 0
        self.m4 = 0

        self.timestamp = None

    @staticmethod
    def parse(data):
        motor = Motor()
        motor.m1 = data[0]
        motor.m2 = data[1]
        motor.m3 = data[2]
        motor.m4 = data[3]

        return motor

    def to_array(self):
        motor = [
            self.m1,
            self.m2,
            self.m3,
            self.m4
        ]

        return motor
