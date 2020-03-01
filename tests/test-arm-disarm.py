#!/usr/bin/env python

"""test-send.py: Test script to send RC commands to a MultiWii Board."""

from msp.multiwii import MultiWii
import time

if __name__ == "__main__":

    #board = MultiWii("/dev/tty.usbserial-AM016WP4")
    board = MultiWii("/dev/ttyS0")

    print("Arming!")
    board.arm()
    print("Board is armed now!")
    print("In 3 seconds it will disarm...")
    time.sleep(3)
    board.disarm()
    print("Disarmed.")
