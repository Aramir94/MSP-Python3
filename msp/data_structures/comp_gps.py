
class CompGPS:
    def __init__(self):
        self.distance_to_home = 0   # meters
        self.direction_to_home = 0  # degree (-180,180)
        self.update = False         # boolean

        self.timestamp = None

    def parse(self, data):
        self.distance_to_home = data[0]
        self.direction_to_home = data[1]
        self.update = data[2]

    def get(self):
        comp_gps = [
            self.distance_to_home,
            self.direction_to_home,
            self.update
        ]
        return comp_gps
