from typing import List

from msp.data_structures.data_structure import DataStructure


class Misc(DataStructure):
    def __init__(self):
        self.int_power_trigger1 = 0
        self.conf_mini_throttle = 0
        self.max_throttle = 0
        self.min_command = 0
        self.conf_failsafe_throttle = 0
        self.plog_arm = 0
        self.plog_lifetime = 0
        self.conf_mag_declination = 0
        self.vbat_scale = 0
        self.vbat_level_warn1 = 0
        self.vbat_level_warn2 = 0
        self.vbat_level_crit = 0

    @staticmethod
    def parse(data: List[int]) -> DataStructure:
        misc = Misc()

        misc.int_power_trigger1 = 0
        misc.conf_mini_throttle = 0
        misc.max_throttle = 0
        misc.min_command = 0
        misc.conf_failsafe_throttle = 0
        misc.plog_arm = 0
        misc.plog_lifetime = 0
        misc.conf_mag_declination = 0
        misc.vbat_scale = 0
        misc.vbat_level_warn1 = 0
        misc.vbat_level_warn2 = 0
        misc.vbat_level_crit = 0

        return misc
