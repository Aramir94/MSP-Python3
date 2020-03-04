#!/usr/bin/env python

"""test-arm-disarm.py: Test script to send RC commands to a MultiWii Board."""
import time
import sys

from msp.multiwii import MultiWii

if __name__ == "__main__":

    #board = MultiWii("/dev/tty.usbserial-AM016WP4")
    print_debug = sys.argv[1].lower() == 'true'
    fc = MultiWii("/dev/ttyS0", print_debug)
    fc.start()

    print("Arming!")
    fc.arm()
    print("Board is armed now!")
    print("In 3 seconds it will disarm...")
    time.sleep(3)
    fc.disarm()
    print("Disarmed.")

    fc.shutdown()
