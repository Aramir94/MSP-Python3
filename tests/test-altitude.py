#!/usr/bin/env python

"""test-altitude.py: Test script to send RC commands to a MultiWii Board."""

import sys

from msp.multiwii import MultiWii

if __name__ == "__main__":
    print_debug = sys.argv[1].lower() == 'true'
    fc = MultiWii("/dev/ttyS0", print_debug)
    fc.start() # Start HIL Thread
    try:
        while True:
            print(fc.get_altitude())
            
    except Exception as err:
        print("Error on Main: " + str(err))
