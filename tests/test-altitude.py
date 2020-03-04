#!/usr/bin/env python

"""test-altitude.py: Test script to send RC commands to a MultiWii Board."""

from msp.multiwii import MultiWii

if __name__ == "__main__":
    fc = MultiWii("/dev/ttyS0")
    try:
        fc.start()
        while True:
            print(fc.get_altitude())
            
    except Exception as err:
        print("Error on Main: " + str(err))
