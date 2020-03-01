#!/usr/bin/env python

"""test-send.py: Test script to send RC commands to a MultiWii Board."""

from msp.multiwii import MultiWii
from msp.message_ids import MessageIDs
import time

if __name__ == "__main__":
    fc = MultiWii("/dev/ttyS0")
    try:
        while True:
            fc.send(0, MessageIDs.ATTITUDE)
            fc.receive()
            print(fc.attitude)
            time.sleep(2)

    except Exception as error:
        import traceback
        print("Error on Main: " + str(error))
        traceback.print_exc()
