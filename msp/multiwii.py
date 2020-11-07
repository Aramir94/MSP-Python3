import serial
import time

from typing import List, Dict
from struct import pack, unpack

from msp.message_ids import MessageIDs

from msp.data_structures.analog import Analog
from msp.data_structures.misc import Misc
from msp.data_structures.rc_tuning import RCTuning
from msp.data_structures.wp import WP
from msp.data_structures.channels import Channel, ARM_VALUE
from msp.data_structures.identification import Identification
from msp.data_structures.status import Status
from msp.data_structures.motors import Motor
from msp.data_structures.gps import GPS
from msp.data_structures.comp_gps import CompGPS
from msp.data_structures.imus import IMU
from msp.data_structures.attitude import Attitude
from msp.data_structures.altitude import Altitude
from msp.data_structures.servos import Servo
from msp.data_structures.pid_coefficients import PIDCoefficients

from msp.data_structures.data_structure import DataStructure


# class MSP_Message:
#     def __init__(self, code: MessageIDs, data: []):
#         self.code: MessageIDs = code
#         self.data: [] = data
#         self.length: int = len(self.data)
#
#     def serialize(self):
#         # Serialize header
#         header = '$'.encode('utf-8') + 'M'.encode('utf-8') + '<'.encode('utf-8')
#
#         # Serialize Data
#         result = header + \
#                  self.length.to_bytes(1, 'little') + \
#                  int(self.code).to_bytes(1, 'little')
#
#
#
#
#         # Serialize Checksum
#         checksum = 0
#         for i in result[3:]:
#             checksum = checksum ^ i
#         result += bytes([checksum])
#
#         # Return result
#         return result


class MultiWii:
    """
    MultiWii Protocol Class (MSP) used for interfacing with MSP flight controller boards
    """

    __FAILSAFE_VALUE = 2050
    __ANGLE_VALUE = 2050

    def __init__(self, ser_port: str, print_debug: bool = False):
        """
        Initializes the protocol to use the given serial port

        :param ser_port: /dev/ttyS0
        :param print_debug: Whether to print the debugging values
        """
        #: bool: Determines whether the protocol is armed or not
        self.is_armed = None

        #: The 2-way serial port used to communicate with the flight controller
        self.ser = None
        self.__init_comms(ser_port)

        #: A Dictionary which maps all the action codes to their respective function
        self.__rx_action_map = self.__create_rx_action_map()
        self.__tx_action_map = self.__create_tx_action_map()

    def __create_rx_action_map(self) -> Dict:

        """
        Creates an action map between the `MessageIDs <msp.message_ids.html>`_

        :return: The action mapped dictionary where the keys are the action code and the values are the action functions
        """

        action_map = {
            # Getters
            MessageIDs.IDENT: Identification.parse,
            MessageIDs.STATUS: Status.parse,
            MessageIDs.RAW_IMU: IMU.parse,
            # MessageIDs.SERVO: None,
            # MessageIDs.MOTOR: self.__motor.parse,
            MessageIDs.RC: Channel.parse,
            MessageIDs.RAW_GPS: GPS.parse,
            # MessageIDs.COMP_GPS: self.__comp_gps.parse,
            MessageIDs.ATTITUDE: Attitude.parse,
            MessageIDs.ALTITUDE: Altitude.parse,
            # MessageIDs.ANALOG: self.__analog.parse,
            # MessageIDs.RC_TUNING: Channel.parse,
            # MessageIDs.PID: None,
            # MessageIDs.BOX: None,
            # MessageIDs.MISC: self.__misc.parse,
            # MessageIDs.MOTOR_PINS: None,
            # MessageIDs.BOXNAMES: None,
            # MessageIDs.PIDNAMES: None,
            MessageIDs.WP: WP.parse,
            # MessageIDs.BOXIDS: None,
            # MessageIDs.SERVO_CONF: None,

            # Setters
            MessageIDs.SET_RAW_RC: Channel.parse,
            MessageIDs.SET_RAW_GPS: GPS.parse,
            # MessageIDs.SET_PID = 202
            # MessageIDs.SET_BOX = 203
            # MessageIDs.SET_RC_TUNING = 204
            # MessageIDs.ACC_CALIBRATION = 205
            # MessageIDs.MAG_CALIBRATION = 206
            # MessageIDs.SET_MISC = 207
            # MessageIDs.RESET_CONF = 208
            # MessageIDs.SET_WP = 209
            # MessageIDs.SWITCH_RC_SERIAL = 210
            # MessageIDs.SET_HEAD = 211
            # MessageIDs.SET_SERVO_CONF = 212
        }

        return action_map

    def __create_tx_action_map(self) -> Dict:

        action_map = {
            # Getters
            MessageIDs.IDENT: Identification().serialize,
            MessageIDs.STATUS: Status().serialize,
            MessageIDs.RAW_IMU: IMU().serialize,
            # MessageIDs.SERVO: None,
            # MessageIDs.MOTOR: self.__motor.parse,
            MessageIDs.RC: Channel().serialize,
            MessageIDs.RAW_GPS: GPS().serialize,
            # MessageIDs.COMP_GPS: self.__comp_gps.parse,
            MessageIDs.ATTITUDE: Attitude().serialize,
            MessageIDs.ALTITUDE: Altitude().serialize,
            # MessageIDs.ANALOG: self.__analog.parse,
            # MessageIDs.RC_TUNING: Channel.parse,
            # MessageIDs.PID: None,
            # MessageIDs.BOX: None,
            # MessageIDs.MISC: self.__misc.parse,
            # MessageIDs.MOTOR_PINS: None,
            # MessageIDs.BOXNAMES: None,
            # MessageIDs.PIDNAMES: None,
            MessageIDs.WP: WP().serialize,
            # MessageIDs.BOXIDS: None,
            # MessageIDs.SERVO_CONF: None,

            # Setters
            MessageIDs.SET_RAW_RC: Channel().serialize,
            MessageIDs.SET_RAW_GPS: GPS().serialize,
            # MessageIDs.SET_PID = 202
            # MessageIDs.SET_BOX = 203
            # MessageIDs.SET_RC_TUNING = 204
            # MessageIDs.ACC_CALIBRATION = 205
            # MessageIDs.MAG_CALIBRATION = 206
            # MessageIDs.SET_MISC = 207
            # MessageIDs.RESET_CONF = 208
            # MessageIDs.SET_WP = 209
            # MessageIDs.SWITCH_RC_SERIAL = 210
            # MessageIDs.SET_HEAD = 211
            # MessageIDs.SET_SERVO_CONF = 212
        }
        return action_map

    def __init_comms(self, ser_port) -> None:
        """
        Initializes the serial communications port and establishes connection with the Flight Controller.

        :param ser_port: Example: /dev/ttyS0
        :return: None
        """
        self.ser = serial.Serial()
        self.ser.port = ser_port
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = None
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        # self.__ser.writeTimeout = 2

        # Time to wait until the board becomes operational
        wakeup = 2
        try:
            self.ser.open()
            print("Waking up board on " + self.ser.port + "...")
            for i in range(1, wakeup):
                print(wakeup - i)
                time.sleep(1)
        except Exception as error:
            print("\n\nError opening " + self.ser.port + " port.\n" + str(error) + "\n\n")

        print("Serial Communication Initialized")

    def __send(self, msg: bytes) -> bool:
        """
        Crafts and sends the command packet to be sent over the serial interface to the Flight Controller.

        :param data_length: The number of 'shorts' required to transmit the data.
        :param code: The MessageID of the command to be sent
        :param data: The data (if required) to be transmitted. Can be left blank if the data_length = 0.
        :return: If the write was successful or not
        """
        try:
            # Clean Output Buffer
            self.ser.reset_output_buffer()

            return self.ser.write(msg) == len(msg)
        except Exception as error:
            import traceback
            print("\n\nError in send.")
            print("(" + str(error) + ")")
            traceback.print_exc()

    def __receive(self):
        """
        Waits for feedback from the flight controller, encodes them into useable python objects
        :return: None
        """

        # Get message header information
        header = self.ser.read().decode('utf-8')
        if header == '$':
            header += self.ser.read(1).decode('utf-8')
            header += self.ser.read(1).decode('utf-8')

        # Determine if an error was thrown
        is_error = '!' in header

        # Get message length
        length = self.ser.read(1)
        length = unpack('<B', length)[0]

        # Get message code
        code = self.ser.read(1)
        code = unpack('<B', code)[0]

        # Get message data

        data = self.ser.read(length)

        # # Get message checksum
        # expected_chksum = unpack('<B', DataStructure.perform_checksum(data))[0]
        actual_chksum = unpack('<B', self.ser.read())[0]
        # if expected_chksusm != actual_chksum:
        #     raise DataStructure.ChecksumMismatch(code, expected_chksum, actual_chksum)

        # Clear input buffer
        self.ser.reset_input_buffer()


        # TODO Add logging

        if is_error:
            raise Exception("FC Responded with an error "
                            "Code: {}, Length:{}, Data: {}".format(code, length, data)
                            )

        return self.__rx_action_map[code](data)

    # Public Methods

    def close(self):
        self.ser.close()

    def command(self, msg: bytes):
        if not self.ser.is_open:
            raise Exception("Serial Port not open yet")

        value = self.__send(msg)
        response = self.__receive()
        return response

    def get_attribute(self, msg_id: MessageIDs) -> DataStructure:
        # Create message
        msg: bytes = self.__tx_action_map[msg_id]([])

        # Command Message
        return self.command(msg)

    def set_attribute(self, msg_id: MessageIDs, data: []) -> DataStructure:
        # Create message
        msg: bytes = self.__tx_action_map[msg_id](data)

        # Command message
        return self.command(msg)