from unittest import TestCase

from msp.multiwii import MultiWii
from msp.message_ids import MessageIDs


class TestMultiWii(TestCase):
    msp = None
    @classmethod
    def setUpClass(self) -> None:
        self.msp = MultiWii("Com3")

    @classmethod
    def tearDownClass(self) -> None:
        self.msp.close()
        del self.msp

    # Getters
    def test_get_ident(self):
        response = self.msp.get_attribute(MessageIDs.IDENT)
        print(response)

    def test_get_status(self):
        response = self.msp.get_attribute(MessageIDs.STATUS)
        print(response)

    def test_get_raw_imu(self):
        response = self.msp.get_attribute(MessageIDs.RAW_IMU)
        print(response)

    def test_get_attitude(self):
        response = self.msp.get_attribute(MessageIDs.ATTITUDE)
        print(response)

    def test_get_altitude(self):
        response = self.msp.get_attribute(MessageIDs.ALTITUDE)
        print(response)

    def test_get_rc(self):
        response = self.msp.get_attribute(MessageIDs.RC)
        print(response)

    def test_get_gps(self):
        response = self.msp.get_attribute(MessageIDs.RAW_GPS)
        print(response)

    # Setters
    def test_set_rc(self):
        values = [
            1500,
            1550,
            1600,
            1650,
            1700,
            1750,
            1800,
            1850
        ]
        response = self.msp.set_attribute(MessageIDs.SET_RAW_RC, values)
        print(response)

    def test_set_gps(self):
        values = [0, 0, 0, 0, 0, 0]
        response = self.msp.set_attribute(MessageIDs.SET_RAW_GPS, values)
        print(response)