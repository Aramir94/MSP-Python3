from unittest import TestCase

from msp.multiwii import MultiWii
from msp.multiwii import MSP_Message
from msp.message_ids import MessageIDs


class TestMultiWii(TestCase):
    msp = None
    @classmethod
    def setUpClass(self) -> None:
        self.msp = MultiWii("Com3")

    @classmethod
    def tearDownClass(self) -> None:
        self.msp.shutdown()

    def test_get_ident(self):
        msg = MSP_Message(MessageIDs.IDENT)
        response = self.msp.command(msg)
        print(response)

    def test_get_status(self):
        msg = MSP_Message(MessageIDs.STATUS)
        response = self.msp.command(msg)
        print(response)

    def test_get_raw_imu(self):
        msg = MSP_Message(MessageIDs.RAW_IMU)
        response = self.msp.command(msg)
        print(response)

    def test_get_rc(self):
        msg = MSP_Message(MessageIDs.RC)
        response = self.msp.command(msg)
        print(response)

    def test_set_rc(self):
        values = [1550, 1600, 1650, 1700, 1750, 1800, 1850, 1950]
        msg = MSP_Message(MessageIDs.SET_RAW_RC, values)
        response = self.msp.command(msg)
        print(response)

    def test_attitude(self):
        msg = MSP_Message(MessageIDs.ATTITUDE)
        response = self.msp.command(msg)
        print(response)

    def test_altitude(self):
        msg = MSP_Message(MessageIDs.ALTITUDE)
        response = self.msp.command(msg)
        print(response)