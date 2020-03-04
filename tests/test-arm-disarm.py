#!/usr/bin/env python

"""test-arm-disarm.py: Test script to send RC commands to a MultiWii Board."""

from msp.multiwii import MultiWii
import time

if __name__ == "__main__":

    #board = MultiWii("/dev/tty.usbserial-AM016WP4")
    fc = MultiWii("/dev/ttyS0")

    print("Arming!")
    fc.arm()
    print("Board is armed now!")
    print("In 3 seconds it will disarm...")
    time.sleep(3)
    fc.disarm()
    print("Disarmed.")

    fc.shutdown()
