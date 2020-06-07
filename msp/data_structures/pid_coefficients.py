from msp.data_structures.data_structure import DataStructure


class PIDCoefficients(DataStructure):
    def __init__(self):
        self.rp = 0
        self.ri = 0
        self.rd = 0

        self.pp = 0
        self.pi = 0
        self.pd = 0

        self.yp = 0
        self.yi = 0
        self.yd = 0

        self.timestamp = 0

    @staticmethod
    def parse(data):
        pid_coefficients = PIDCoefficients()

        pid_coefficients.rp = data[0]
        pid_coefficients.ri = data[1]
        pid_coefficients.rd = data[2]

        pid_coefficients.pp = data[3]
        pid_coefficients.pi = data[4]
        pid_coefficients.pd = data[5]

        pid_coefficients.yp = data[6]
        pid_coefficients.yi = data[7]
        pid_coefficients.yd = data[8]

        return pid_coefficients

    def to_array(self):
        pid = [
            self.rp,
            self.ri,
            self.rd,
            self.pp,
            self.pi,
            self.pd,
            self.yp,
            self.yi,
            self.yd
        ]

        return pid
