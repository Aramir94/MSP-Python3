#!/usr/bin/env python

"""test-altitude.py: Test script to send RC commands to a MultiWii Board."""

from msp.multiwii import MultiWii
from msp.message_ids import MessageIDs

if __name__ == "__main__":
    fc = MultiWii("/dev/ttyS0")
    try:

        fc.send(0, MessageIDs.ALTITUDE)
        fc.receive()
        print(fc.altitude)

    except Exception as error:
        print("Error on Main: " + str(error))
