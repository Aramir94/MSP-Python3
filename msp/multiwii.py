import serial
import struct
import time
import queue

from typing import List, Callable, Dict

from threading import Thread, Lock
from threading import current_thread

from msp.data_structures.analog import Analog
from msp.data_structures.misc import Misc
from msp.data_structures.rc_tuning import RCTuning
from msp.data_structures.wp import WP
from msp.message_ids import MessageIDs
from msp.data_structures.channels import Channel
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





class MultiWii(Thread):
    """
    MultiWii Protocol Class (MSP) used for interfacing with MSP flight controller boards
    """

    __FAILSAFE_VALUE = 2050
    __ANGLE_VALUE = 2050

    def __init__(self, ser_port:str, print_debug: bool = False):
        """
        Initializes the protocol to use the given serial port

        :param ser_port: /dev/ttyS0
        :param print_debug: Whether to print the debugging values
        """
        super(MultiWii, self).__init__(
            name="Comms_Tx"
        )

        #: Used for toggling debug output
        self.__print_debug = print_debug

        # Private Attributes
        self.__lock = Lock()

        #: bool: Determines whether the protocol continues to run or not
        self.__running = True

        #: bool: Determines whether the protocol is armed or not
        self.__is_armed = False

        #: Thread safe queue used for receiving feedback from the flight controller
        self.__q = queue.Queue()
        self.__timeout = 1.0/60

        #: The thread used to receive feedback from the flight controller
        self.__rx_thread = Thread(
            target=self.__receive,
            args=[],
            name="Comms_Rx",
            daemon=True
        )

        #: The 2-way serial port used to communicate with the flight controller
        self.__ser = None
        self.__init_comms(ser_port)
        self.__rc_actual = Channel()
        self.rc_target = Channel()
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
            MessageIDs.SERVO_CONF: None
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
            self.__print("Waking up board on " + self.__ser.port + "...")
            for i in range(1, wakeup):
                self.__print(wakeup - i)
                time.sleep(1)
        except Exception as error:
            print("\n\nError opening " + self.__ser.port + " port.\n" + str(error) + "\n\n")

        self.__print("Serial Communication Initialized")

    def __on_thread(self, function, *args: List[any], **kwargs: List[any]) -> None:
        """
        Used to pass actions to the protocol for transmission to the flight controller
        :param function: The function to be performed
        :param args: Any positional arguments the function might require
        :param kwargs: Any keyword arguments the function may require
        :return: None
        """
        self.__q.put((function, args, kwargs))

    def __print(self, data: str) -> None:
        """
        Prints the debugging output if :attr:`__print_debug` is enabled

        :param data: The data to be printed
        :return: None
        """
        if self.__print_debug:
            print(data)

    def __shutdown(self) -> None:
        """
        Terminates the protocol by setting :attr:`__running` to false

        :return: None
        """
        print(current_thread().name + " - Shutting Down")
        #: bool: Controls whether the protocol is running or not. If set to False the protocol shutdowns and will need to be reactivated
        self.__running = False

    def __idle(self) -> None:
        """
        When not handling tx or rx the protocol will request general information from the flight controller

        :return: None
        """
        # Request Data
        self.__send(MessageIDs.RAW_IMU)
        self.__send(MessageIDs.ALTITUDE)
        self.__send(MessageIDs.ATTITUDE)
        self.__send(MessageIDs.RC)
        self.__send(MessageIDs.RAW_GPS)
        self.__send(MessageIDs.COMP_GPS)
        self.__send(MessageIDs.PID)
        self.__send(MessageIDs.MOTOR)
        self.__send(MessageIDs.ANALOG)
        self.__send(MessageIDs.RC_TUNING)
        self.__send(MessageIDs.MISC)
        self.__send(MessageIDs.WP)


        # Set RC values
        data = self.rc_target.to_array()
        self.__send(
            MessageIDs.SET_RAW_RC,
            len(data)*2,
            data
        )

    def __arm(self) -> None:
        """
        Sets the protocol to indicate that it's in an armed state

        :return: None
        """
        self.__print("Arming...")
        self.__rc_actual.armed(True)

        data = self.__rc_actual.to_array()
        self.__send(
            MessageIDs.SET_RAW_RC,
            len(data)*2,
            data
        )

        self.__is_armed = True
        self.__print("Armed")

    def __disarm(self) -> None:
        """
        Sets the protocol to indicate that it's in an disarmed state

        :return: None
        """
        self.__print("Disarming...")
        # Roll, Pitch, Throttle, Yaw
        self.__rc_actual.armed(False)

        data = self.__rc_actual.to_array()
        self.__send(
            MessageIDs.SET_RAW_RC,
            len(data)*2,
            data
        )

        self.__is_armed = False
        self.__print("Disarmed")

    def __send(self, code: MessageIDs, data_length=0, data=None) -> None:
        """
        Crafts and sends the command packet to be sent over the serial interface to the Flight Controller.

        :param data_length: The number of 'shorts' required to transmit the data.
        :param code: The MessageID of the command to be sent
        :param data: The data (if required) to be transmitted. Can be left blank if the data_length = 0.
        :return: None
        """
        if data is None:
            data = []

        total_data = ['$'.encode('utf-8'), 'M'.encode('utf-8'), '<'.encode('utf-8'), data_length, code] + data
        structure = struct.pack('<2B%dH' % len(data), *total_data[3:len(total_data)])

        checksum = 0
        for i in structure:
            checksum = checksum ^ i
        total_data.append(checksum)

        try:
            b = self.__ser.write(struct.pack('<3c2B%dHB' % len(data), *total_data))
            # self.__ser.flushOutput()
        except Exception as error:
            import traceback
            print("\n\nError in send.")
            print("(" + str(error) + ")")
            traceback.print_exc()

    def __receive(self) -> None:
        """
        Waits for feedback from the flight controller, encodes them into useable python objects
        :return: None
        """
        self.__print("Starting " + current_thread().name)
        while self.__running:
            try:
                while True:
                    header = self.__ser.read().decode('utf-8')
                    if header == '$':
                        header = header + self.__ser.read(2).decode('utf-8')
                        break

                data_length = struct.unpack('<B', self.__ser.read())[0]
                code = struct.unpack('<B', self.__ser.read())[0]
                data = self.__ser.read(data_length)
                # TODO Add logging
                self.__print("Receiving - " + str(code))
                checksum = struct.unpack('<B', self.__ser.read())[0]
                # TODO check Checksum
                # total_data = ['$'.encode('utf-8'), 'M'.encode('utf-8'), '<'.encode('utf-8'), data_length, code] + data
                # structure = struct.pack('<2B%dH' % len(data), *total_data[3:len(total_data)])
                #
                # checksum = 0
                # for i in structure:
                #     checksum = checksum ^ i
                # total_data.append(checksum)

                # print("code: " + str(code))
                # print("data_length: " + str(data_length))
                # print("data: " + str(data))
                # print("checksum: " + str(checksum))

                self.__ser.flushInput()

            except Exception as error:
                import traceback
                print("\n\nError in receive.")
                print("(" + str(error) + ")")
                traceback.print_exc()
                return

            if not data_length > 0:
                return

            temp = struct.unpack('<' + 'h' * int(data_length / 2), data)
            try:
                self.__code_action_map[code](temp)
            except KeyError as err:
                print(err)

    # Public Methods
    def shutdown(self):
        self.__on_thread(self.__shutdown)

    def run(self) -> None:
        """
        The main running loop for the protocol

        Starts the receiver thread, and then waits for transmissions to be sent to the flight controller

        :return: None
        """

        try:
            self.__rx_thread.start()
            while self.__running:
                try:
                    function, args, kwargs = self.__q.get(timeout=self.__timeout)
                    function(*args, **kwargs)
                except queue.Empty:
                    self.__idle()

        finally:
            self.__print("Closing Serial Port")
            self.__ser.close()

    def is_armed(self) -> bool:
        """
        Returns whether the protocol is armed or not

        :return: Whether the protocol has been armed or not.
        """
        return self.__is_armed

    def arm(self) -> None:
        """
        Arms the protocol

        :return: None
        """
        self.__on_thread(self.__arm)

    def disarm(self) -> None:
        """
        Disarms the protocol

        :return: None
        """
        self.__on_thread(self.__disarm)

    def calibrate_acc(self):
        self.__on_thread(self.__send, [MessageIDs.ACC_CALIBRATION, []])

    # Setters
    def set_target_channels(self, channel):
        self.rc_target = channel

    # Getters
    def get_imu(self):
        return self.__imu.to_array()

    def get_attitude(self):
        return self.__attitude.to_array()

    def get_altitude(self):
        return self.__altitude.to_array()

    def get_rc_channels(self):
        return self.__rc_actual.to_array()

    def get_gps(self):
        return self.__gps.to_array()

    def get_comp_gps(self):
        return self.__comp_gps.to_array()

    def get_pid(self):
        return self.pid_coef.to_array()


