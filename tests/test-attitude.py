#!/usr/bin/env python

"""test-attitude.py: Test script to send RC commands to a MultiWii Board."""
import time
from msp.multiwii import MultiWii

if __name__ == "__main__":
    try:
        fc = MultiWii("/dev/ttyS0")
        fc.start()
        while True:
            print(fc.get_attitude())
            time.sleep(.5)


    except Exception as error:
        import traceback
        print("Error on Main: " + str(error))
        traceback.print_exc()
