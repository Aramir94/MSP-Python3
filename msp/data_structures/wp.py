from typing import List

from msp.data_structures.data_structure import DataStructure


class WP(DataStructure):
    def __init__(self):
        self.wp_no = 0
        self.lat = 0
        self.lon = 0
        self.alt_hold = 0
        self.heading = 0
        self.time_to_stay = 0
        self.nav_flag = 0

    @staticmethod
    def parse(data: List[int]) -> DataStructure:
        wp = WP()

        wp.wp_no = data[0]
        wp.lat = data[1]
        wp.lon = data[2]
        wp.alt_hold = data[3]
        wp.heading = data[4]
        wp.time_to_stay = data[5]
        wp.nav_flag = data[6]

        return wp


