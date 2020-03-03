#!/usr/bin/env python

"""test-attitude.py: Test script to send RC commands to a MultiWii Board."""

from msp.multiwii import MultiWii
from msp.message_ids import MessageIDs
import time

if __name__ == "__main__":
    fc = MultiWii("/dev/ttyS0")
    try:
        fc.start()
        while True:
            print(fc.attitude)

    except Exception as error:
        import traceback
        print("Error on Main: " + str(error))
        traceback.print_exc()
