
class Attitude:
    def __init__(self):
        self.angx = 0
        self.angy = 0
        self.heading = 0

        self.timestamp = None

    def parse(self, data):
        self.angx = data[0]
        self.angy = data[1]
        self.heading = data[2]

    def get(self):
        attitude = [
            self.angx,
            self.angy,
            self.heading
        ]
        return attitude
