
class Status:
    def __init__(self):
        self.cycleTime = 0
        self.i2c_errors_count = 0
        self.sensor = 0
        self.flag = 0
        self.global_conf = 0

        self.timestamp = 0

    def parse(self, data):
        self.cycleTime = data[0]
        self.i2c_errors_count = data[1]
        self.sensor = data[2]
        self.flag = data[3]
        self.global_conf = data[4]

        self.timestamp = 0

    def get(self):
        status = [
            self.cycleTime,
            self.i2c_errors_count,
            self.sensor,
            self.flag,
            self.global_conf
        ]
