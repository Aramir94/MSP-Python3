import serial
import struct
import time
import queue
from threading import Thread, Lock
from threading import current_thread

from msp.message_ids import MessageIDs

OFF_VALUE = 850

class MultiWii(Thread):
    """Class initialization"""

    __ARM_VALUE = 2050
    __FAILSAFE_VALUE = 2050
    __ANGLE_VALUE = 2050

    def __init__(self, ser_port):
        super(MultiWii, self).__init__(
            name="Comms_Tx"
        )

        self.__print_debug = True

        # Private Attributes
        self.__lock = Lock()
        self.__running = True
        self.__is_armed = False
        self.__q = queue.Queue()
        self.__timeout = 1.0/60

        self.__rx_thread = Thread(
            target=self.__receive,
            args=[],
            name="Comms_Rx",
            daemon=True
        )

        self.__ser = None
        self.__init_comms(ser_port)

        self.__rc_channels = Channels()
        self.__attitude = Attitude()
        self.__altitude = Altitude()
        self.__imu = IMU()

        self.__code_action_map = self.__create_action_map()

        # Public Attributes
        self.identification = Identification()
        self.pid_coef = PIDCoefficients()



        self.motor = Motor()



        self.vtx_config = {
            'device': 0,
            'band': 0,
            'channel': 0,
            'power': 0,
            'pit': 0,
            'unknown': 0
        }

    # Private Methods
    def __create_action_map(self):
        code_action_map = {}
        code_action_map[MessageIDs.IDENT] = self.identification.parse
        """
            STATUS = 101
        """
        code_action_map[MessageIDs.RAW_IMU] = self.__imu.parse
        """
            SERVO = 103
            MOTOR = 104
        """
        code_action_map[MessageIDs.RC] = self.__rc_channels.parse
        """
            RAW_GPS = 106
            COMP_GPS = 107
        """
        code_action_map[MessageIDs.ATTITUDE] = self.__attitude.parse
        code_action_map[MessageIDs.ALTITUDE] = self.__altitude.parse
        """
            ANALOG = 110
            RC_TUNING = 111
            PID = 112
            BOX = 113
            MISC = 114
            MOTOR_PINS = 115
            BOXNAMES = 116
            PIDNAMES = 117
            WP = 118
            BOXIDS = 119
            SERVO_CONF = 120
        """

        return code_action_map

    def __init_comms(self, ser_port):
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

    def __on_thread(self, function, *args, **kwargs):
        self.__q.put((function, args, kwargs))

    def __print(self, data):
        if self.__print_debug:
            print(data)

    def __shutdown(self):
        print(current_thread().name + " - Shutting Down")
        self.__running = False

    def __idle(self):
        # TODO create looping control logic
        self.__send(MessageIDs.RAW_IMU)
        self.__send(MessageIDs.ALTITUDE)
        self.__send(MessageIDs.ATTITUDE)
        self.__send(MessageIDs.RC)

    def __arm(self):
        self.__print("Arming...")
        self.__rc_channels.arm = self.__ARM_VALUE
        self.__send(MessageIDs.SET_RAW_RC, 14, self.__rc_channels.get())
        self.__is_armed = True
        self.__print("Armed")

    def __disarm(self):
        self.__print("Disarming...")
        # Roll, Pitch, Throttle, Yaw
        self.__rc_channels.arm = OFF_VALUE
        self.__send(MessageIDs.SET_RAW_RC, 14, self.__rc_channels.get())
        self.__is_armed = False
        self.__print("Disarmed")

    def __send(self, code: MessageIDs, data_length=0, data=None):
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

    def __receive(self):
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

    def run(self):
        try:
            self.__rx_thread.start()
            while self.__running:
                try:
                    function, args, kwargs = self.__q.get(timeout=self.__timeout)
                    function(*args, **kwargs)
                except queue.Empty:
                    self.__print("Idling")
                    self.__idle()
        finally:
            self.__print("Closing Serial Port")
            self.__ser.close()

    def is_armed(self):
        return self.__is_armed

    def arm(self):
        self.__on_thread(self.__arm)

    def disarm(self):
        self.__on_thread(self.__disarm)

    def calibrate_acc(self):
        self.__on_thread(self.__send, [MessageIDs.ACC_CALIBRATION, []])

    # Getters
    def get_rc_channels(self):
        return self.__rc_channels.get()

    def get_imu(self):
        return self.__imu.get()

    def get_attitude(self):
        return self.__attitude.get()

    def get_altitude(self):
        return self.__altitude.get()



class Identification:
    def __init__(self):
        self.version = 0
        self.multi_type = 0
        self.msp_version = 0
        self.capability = 0

        self.timestamp = None

    def parse(self, data):
        self.version = data[0]
        self.multi_type = data[1]
        self.msp_version = data[2]
        self.capability = data[3]

        self.timestamp = None


class PIDCoefficients:
    def __init__(self):
        self.rp = 0
        self.ri = 0
        self.rd = 0

        self.pp = 0
        self.pi = 0
        self.pd = 0

        self.yp = 0
        self.yi = 0
        self.yd = 0

        self.timestamp = None


class Channels:
    def __init__(self):
        self.roll = OFF_VALUE
        self.pitch = OFF_VALUE
        self.yaw = OFF_VALUE
        self.throttle = OFF_VALUE
        self.arm = OFF_VALUE
        self.angle = OFF_VALUE
        self.failsafe = OFF_VALUE

        self.timestamp = 0

    def parse(self, data):
        self.roll = data[0]
        self.pitch = data[1]
        self.yaw = data[2]
        self.throttle = data[3]
        self.arm = data[4]
        self.angle = data[5]
        self.failsafe = data[6]

        self.timestamp = 0

    def get(self):
        channels = [
            self.roll,
            self.pitch,
            self.yaw,
            self.throttle,
            self.arm,
            self.angle,
            self.failsafe
        ]
        return channels


class IMU:
    def __init__(self):
        self.ax = 0
        self.ay = 0
        self.az = 0

        self.gx = 0
        self.gy = 0
        self.gz = 0

        self.mx = 0
        self.my = 0
        self.mz = 0

        self.timestamp = None

    def parse(self, data):
        self.ax = data[0]
        self.ay = data[1]
        self.az = data[2]

        self.gx = data[3]
        self.gy = data[4]
        self.gz = data[5]

        self.mx = data[6]
        self.my = data[7]
        self.mz = data[8]

        self.timestamp = None

    def get(self):
        imu = [
            self.ax,
            self.ay,
            self.az,
            self.gx,
            self.gy,
            self.gz,
            self.mx,
            self.my,
            self.mz
        ]

        return imu


class Motor:
    def __init__(self):
        self.m1 = 0
        self.m2 = 0
        self.m3 = 0
        self.m4 = 0

        self.timestamp = None


class Attitude:
    def __init__(self):
        self.angx = 0
        self.angy = 0
        self.heading = 0

        self.timestamp = None

    def parse(self, data):
        self.angx = data[0]
        self.angy = data[1]
        self.heading = data[2]

    def get(self):
        attitude = [
            self.angx,
            self.angy,
            self.heading
        ]
        return attitude


class Altitude:
    def __init__(self):
        self.estalt = 0
        self.vario = 0

        self.timestamp = None

    def parse(self, data):
        self.estalt = data[0]
        self.vario = data[1]

    def get(self):
        altitude = [
            self.estalt,
            self.vario
        ]

        return altitude
