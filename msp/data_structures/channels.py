from __future__ import annotations
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
    def parse(data):
        channel = Channel()

        channel.roll = data[0]
        channel.pitch = data[1]
        channel.yaw = data[2]
        channel.throttle = data[3]
        channel.arm = data[4]
        channel.angle = data[5]
        channel.failsafe = data[6]

        return channel

    def __add__(self, other: Channel) -> Channel:
        if self.arm != other.arm:
            raise ArmedMissMatchError
        elif self.angle != other.angle:
            raise AngleMissMatchError
        elif self.failsafe != other.failsafe:
            raise FailsafeMissMatchError

        new = Channel()
        new.roll = self.roll + other.roll
        new.pitch = self.pitch + other.pitch
        new.yaw = self.yaw + other.yaw
        new.throttle = self.throttle + other.throttle

        return new

    def armed(self, armed: bool):

        if armed:
            self.arm = self.ARM_VALUE
        else:
            self.arm = self.OFF_VALUE

    def to_array(self):
        return [
            self.roll,
            self.pitch,
            self.yaw,
            self.throttle,
            self.arm,
            self.angle,
            self.failsafe
        ]


class ArmedMissMatchError(Exception):
    pass


class AngleMissMatchError(Exception):
    pass


class FailsafeMissMatchError(Exception):
    pass

