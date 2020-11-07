import serial
import struct
import time

from typing import List, Dict

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


class MSP_Message:
    def __init__(self, code: MessageIDs, data: [] = []):
        self.code: MessageIDs = code
        self.data: [] = data
        self.length: int = len(self.data)

    def serialize(self):
        # Serialize header
        result = '$'.encode('utf-8') + 'M'.encode('utf-8') + '<'.encode('utf-8')

        # Serialize Data
        result += self.length.to_bytes(1, 'little') + int(self.code).to_bytes(1, 'little')
        for item in self.data:
            result += int(item).to_bytes(2, 'little')

        # Serialize Checksum
        checksum = 0
        for i in result[3:]:
            checksum = checksum ^ i
        result += checksum.to_bytes(1, 'little')

        # Return result
        return result


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
        self.__ser = None
        self.__init_comms(ser_port)

        self.__rc_actual = Channel()
        self.__rc_target = Channel()
        self.__attitude = Attitude()
        self.__altitude = Altitude()
        self.__imu = IMU()
        self.__gps = GPS()
        self.__comp_gps = CompGPS()
        self.__motor = Motor()
        self.__analog = Analog()
        self.__rc_tuning = RCTuning()
        self.__misc = Misc()
        self.__wp = WP()

        # Public Attributes
        self.identification = Identification()
        self.status = Status()
        self.servo = Servo()
        self.motor = Motor()
        self.pid_coef = PIDCoefficients()

        self.vtx_config = {
            'device': 0,
            'band': 0,
            'channel': 0,
            'power': 0,
            'pit': 0,
            'unknown': 0
        }

        #: A Dictionary which maps all the action codes to their respective function
        self.__code_action_map = self.__create_action_map()

    # Private Methods
    @property
    def is_armed(self) -> bool:
        """Checks to see what the last report from hardware says the value was"""
        return self.__rc_actual.is_armed()

    @is_armed.setter
    def is_armed(self, value):
        """Dummy setter"""
        pass

    def __create_action_map(self) -> Dict:
        """
        Creates an action map between the `MessageIDs <msp.message_ids.html>`_

        :return: The action mapped dictionary where the keys are the action code and the values are the action functions
        """
        code_action_map = {
            MessageIDs.IDENT: self.identification.parse,
            MessageIDs.STATUS: self.status.parse,
            MessageIDs.RAW_IMU: self.__imu.parse,
            MessageIDs.SERVO: None,
            MessageIDs.MOTOR: self.__motor.parse,
            MessageIDs.RC: self.__rc_actual.parse,
            MessageIDs.RAW_GPS: self.__gps.parse,
            MessageIDs.COMP_GPS: self.__comp_gps.parse,
            MessageIDs.ATTITUDE: self.__attitude.parse,
            MessageIDs.ALTITUDE: self.__altitude.parse,
            MessageIDs.ANALOG: self.__analog.parse,
            MessageIDs.RC_TUNING: self.__rc_tuning.parse,
            MessageIDs.PID: None,
            MessageIDs.BOX: None,
            MessageIDs.MISC: self.__misc.parse,
            MessageIDs.MOTOR_PINS: None,
            MessageIDs.BOXNAMES: None,
            MessageIDs.PIDNAMES: None,
            MessageIDs.WP: self.__wp.parse,
            MessageIDs.BOXIDS: None,
            MessageIDs.SERVO_CONF: None,

            MessageIDs.SET_RAW_RC: None
        }

        return code_action_map

    def __init_comms(self, ser_port) -> None:
        """
        Initializes the serial communications port and establishes connection with the Flight Controller.

        :param ser_port: Example: /dev/ttyS0
        :return: None
        """
        self.__ser = serial.Serial()
        self.__ser.port = ser_port
        self.__ser.baudrate = 115200
        self.__ser.bytesize = serial.EIGHTBITS
        self.__ser.parity = serial.PARITY_NONE
        self.__ser.stopbits = serial.STOPBITS_ONE
        self.__ser.timeout = None
        self.__ser.xonxoff = False
        self.__ser.rtscts = False
        self.__ser.dsrdtr = False
        # self.__ser.writeTimeout = 2

        # Time to wait until the board becomes operational
        wakeup = 2
        try:
            self.__ser.open()
            print("Waking up board on " + self.__ser.port + "...")
            for i in range(1, wakeup):
                print(wakeup - i)
                time.sleep(1)
        except Exception as error:
            print("\n\nError opening " + self.__ser.port + " port.\n" + str(error) + "\n\n")

        print("Serial Communication Initialized")

    def __send(self, msg: MSP_Message) -> bool:
        """
        Crafts and sends the command packet to be sent over the serial interface to the Flight Controller.

        :param data_length: The number of 'shorts' required to transmit the data.
        :param code: The MessageID of the command to be sent
        :param data: The data (if required) to be transmitted. Can be left blank if the data_length = 0.
        :return: If the write was successful or not
        """
        try:
            # Send message to FC
            byte_msg = msg.serialize()

            return self.__ser.write(byte_msg) == len(byte_msg)
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
        header = self.__ser.read().decode('utf-8')
        if header == '$':
            header += self.__ser.read(1).decode('utf-8')
            header += self.__ser.read(1).decode('utf-8')

        # Determine if an error was thrown
        is_error = '!' in header

        # Get message length
        length = self.__ser.read(1)
        length = struct.unpack('<B', length)[0]

        # Get message code
        code = self.__ser.read(1)
        code = struct.unpack('<B', code)[0]

        # Get message data
        data = self.__ser.read(length)

        # Get message checksum
        checksum = struct.unpack('<B', self.__ser.read())[0]

        # TODO check Checksum
        # TODO Add logging

        if is_error:
            raise Exception("FC Responded with an error "
                            "Code: {}, Length:{}, Data: {}".format(code, length, data)
                            )

        return self.__code_action_map[code](data)

    # Public Methods

    def close(self):
        self.__ser.close()

    def command(self, msg: MSP_Message):
        if not self.__ser.is_open:
            raise Exception("Serial Port not open yet")

        value = self.__send(msg)
        response = self.__receive()
        return response

    # Setters
    def set_roll(self, value: float):
        """Gets the current roll value"""
        self.__rc_target.roll = value

    def set_pitch(self, value: float):
        """Gets the current pitch value"""
        self.__rc_target.pitch = value

    def set_yaw(self, value: float):
        """Gets the current yaw value"""
        self.__rc_target.yaw = value

    def set_throttle(self, value: float):
        """Gets the current throttle value"""
        self.__rc_target.throttle = value

    def arm(self):
        """Arms the protocol"""
        print("Arming...")
        self.__rc_target.arm = ARM_VALUE

    def disarm(self):
        """Disarms the protocol"""
        print("Disarming...")
        self.__rc_target.arm = Channel.MIN_VALUE

    def set_mma_values(self, values: List[int]):
        self.set_roll(values[0])
        self.set_pitch(values[1])
        self.set_yaw(values[2])
        self.set_throttle(values[3])
        if len(values) > 4:
            if values[4] == ARM_VALUE:
                self.arm()
            else:
                self.disarm()

    def get_pos(self) -> List[float]:
        return [self.__gps.lat, self.__gps.lon]

    def get_heading(self) -> int:
        return self.__attitude.heading

    def get_altitude(self) -> float:
        return self.__gps.altitude