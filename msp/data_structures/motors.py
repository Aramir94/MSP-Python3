
class Motor:
    def __init__(self):
        self.m1 = 0
        self.m2 = 0
        self.m3 = 0
        self.m4 = 0

        self.timestamp = None

    def parse(self, data):
        self.m1 = data[0]
        self.m2 = data[1]
        self.m3 = data[2]
        self.m4 = data[3]

        self.timestamp = 0

    def get(self):
        motor = [
            self.m1,
            self.m2,
            self.m3,
            self.m4
        ]

        return motor
