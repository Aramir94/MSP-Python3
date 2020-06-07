from msp.data_structures.data_structure import DataStructure


class Attitude(DataStructure):
    def __init__(self):
        self.angx = 0
        self.angy = 0
        self.heading = 0

        self.timestamp = None

    @staticmethod
    def parse(data):
        attitude = Attitude()

        attitude.angx = data[0]
        attitude.angy = data[1]
        attitude.heading = data[2]

        return attitude

    def to_array(self):
        attitude = [
            self.angx,
            self.angy,
            self.heading
        ]
        return attitude
