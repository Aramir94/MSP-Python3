
class Altitude:
    def __init__(self):
        self.estalt = 0
        self.vario = 0

        self.timestamp = None

    def parse(self, data):
        self.estalt = data[0]
        self.vario = data[1]

    def get(self):
        """

        :return: [Estimated_altitude:cm,
        """
        altitude = [
            self.estalt,
            self.vario
        ]

        return altitude
