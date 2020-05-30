from __future__ import annotations


class Channel:
    MAX_VALUE = 2100
    MIN_VALUE = 950
    MID_VALUE = 1500
    ARM_VALUE = 2050
    OFF_VALUE = 850

    def __init__(self):
        self.roll = self.MID_VALUE
        self.pitch = self.MID_VALUE
        self.yaw = self.MID_VALUE
        self.throttle = self.MID_VALUE
        self.arm = self.MIN_VALUE
        self.angle = self.MIN_VALUE
        self.failsafe = self.MIN_VALUE

    def parse(self, data):
        self.roll = data[0]
        self.pitch = data[1]
        self.yaw = data[2]
        self.throttle = data[3]
        self.arm = data[4]
        self.angle = data[5]
        self.failsafe = data[6]

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

