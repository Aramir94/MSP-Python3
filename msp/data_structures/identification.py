
class Identification:
    def __init__(self):
        self.version = 0
        self.multi_type = 0
        self.msp_version = 0
        self.capability = 0

        self.timestamp = None

    def parse(self, data):
        self.version = data[0]
        self.multi_type = data[1]
        self.msp_version = data[2]
        self.capability = data[3]

        self.timestamp = None

    def get(self):
        identification = [
            self.version,
            self.multi_type,
            self.msp_version,
            self.capability
        ]

        return identification
