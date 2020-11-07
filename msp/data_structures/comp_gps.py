from msp.data_structures.data_structure import DataStructure


class CompGPS(DataStructure):
    def __init__(self):
        self.distance_to_home = 0   # meters
        self.direction_to_home = 0  # degree (-180,180)
        self.update = False         # boolean

        self.timestamp = None

    @staticmethod
    def parse(data):
        comp_gps = CompGPS()

        comp_gps.distance_to_home = data[0]
        comp_gps.direction_to_home = data[1]
        comp_gps.update = data[2]

        return comp_gps
