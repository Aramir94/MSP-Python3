class PIDCoefficients:
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

    def parse(self, data):
        self.rp = data[0]
        self.ri = data[1]
        self.rd = data[2]

        self.pp = data[3]
        self.pi = data[4]
        self.pd = data[5]

        self.yp = data[6]
        self.yi = data[7]
        self.yd = data[8]

        self.timestamp = 0
        pass

    def get(self):
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
