from msp.data_structures.data_structure import DataStructure


class Altitude(DataStructure):
    def __init__(self):
        self.estalt = 0
        self.vario = 0


    @staticmethod
    def parse(data):
        altitude = Altitude()

        altitude.estalt = data[0]
        altitude.vario = data[1]

        return altitude

    def to_array(self):
        """

        :return: [Estimated_altitude:cm,
        """
        altitude = [
            self.estalt,
            self.vario
        ]

        return altitude
