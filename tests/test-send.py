#!/usr/bin/env python

"""test-send.py: Test script to send RC commands to a MultiWii Board."""


from msp.multiwii import MultiWii
from msp.message_ids import MessageIDs

import time

if __name__ == "__main__":
    board = MultiWii("/dev/ttyS0")
    try:
        while True:
            # example of 8 RC channels to be send
            data = [1500, 1550, 1600, 1560, 1000, 1040, 1000, 1000]

            board.sendCMDreceiveATT(16, MessageIDs.SET_RAW_RC, data)
            
            print(board.attitude)
            time.sleep(2)
    except Exception as error:
        print("Error on Main: " + str(error))
        