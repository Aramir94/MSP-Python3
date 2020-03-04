#!/usr/bin/env python

"""test-attitude.py: Test script to send RC commands to a MultiWii Board."""
import time
import sys

from msp.multiwii import MultiWii

if __name__ == "__main__":
    try:
        print_debug = sys.argv[1].lower() == 'true'
        fc = MultiWii("/dev/ttyS0", print_debug)
        fc.start()
        while True:
            print(fc.get_attitude())
            time.sleep(.5)


    except Exception as error:
        import traceback
        print("Error on Main: " + str(error))
        traceback.print_exc()
