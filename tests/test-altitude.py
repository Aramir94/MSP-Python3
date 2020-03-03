#!/usr/bin/env python

"""test-altitude.py: Test script to send RC commands to a MultiWii Board."""

from msp.multiwii import MultiWii
from msp.message_ids import MessageIDs

if __name__ == "__main__":
    fc = MultiWii("/dev/ttyS0")
    try:
        fc.start()
        while True:
            print(fc.altitude)
    except Exception as err:
        print(err)
