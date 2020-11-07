from struct import unpack

from msp.data_structures.data_structure import DataStructure


MAX_VALUE = 2100
MIN_VALUE = 950
MID_VALUE = 1500
ARM_VALUE = 2050
OFF_VALUE = 850


class Channel(DataStructure):
    def __init__(self):
        self.roll = MID_VALUE
        self.pitch = MID_VALUE
        self.yaw = MID_VALUE
        self.throttle = MID_VALUE
        self.arm = MIN_VALUE
        self.angle = MIN_VALUE
        self.failsafe = MIN_VALUE

    @staticmethod
    def degree_to_value(degree):
        conversion_offset = 1500
        diff = degree - 180
        result = conversion_offset + (diff * 2.77)

        return result

    @staticmethod
    def percent_to_value(percent):
        conversion_offset = 1000
        return percent/100 * conversion_offset + conversion_offset


    @staticmethod
    def parse(data):
        channel = Channel()

        channel.roll = unpack('<H', bytes(data[:2]))[0]
        channel.pitch = unpack('<H', bytes(data[2:4]))[0]
        channel.yaw = unpack('<H', bytes(data[4:6]))[0]
        channel.throttle = unpack('<H', bytes(data[6:8]))[0]
        channel.arm = unpack('<H', bytes(data[8:10]))[0]
        channel.angle = unpack('<H', bytes(data[10:12]))[0]
        channel.failsafe = unpack('<H', bytes(data[12:14]))[0]

        return channel

    def arm(self):
        """Sets the channel to an armed value"""
        self.arm = ARM_VALUE

    def disarm(self):
        """Sets the channel to a disarmed value"""
        self.arm = MIN_VALUE

    def is_armed(self) -> bool:
        """
        True if armed, False otherwise

        :return: Whether the channel is armed or not
        """
        return self.arm == ARM_VALUE

class ArmedMissMatchError(Exception):
    pass


class AngleMissMatchError(Exception):
    pass


class FailsafeMissMatchError(Exception):
    pass

