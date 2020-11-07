from unittest import TestCase

from msp.multiwii import MSP_Message
from msp.message_ids import MessageIDs

class TestMSP_Message(TestCase):
    def test_serialize(self):
        test_msg = MSP_Message(MessageIDs.IDENT)
        print(test_msg.serialize())
        self.assertEqual(b'$M<\x00dd', test_msg.serialize(), "Message didn't serialize correctly")

        test_msg = MSP_Message(MessageIDs.RAW_IMU)

        print(test_msg.serialize())
        self.assertEqual(b'$M<\x00ff', test_msg.serialize(), "Message didn't serialize correctly")