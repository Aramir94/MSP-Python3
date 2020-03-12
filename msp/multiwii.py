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

    def __init__(self, ser_port, print_debug=False):
        super(MultiWii, self).__init__(
            name="Comms_Tx"
        )

        self.__print_debug = print_debug

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
        self.__rc_actual = Channels()
        self.__attitude = Attitude()
        self.__altitude = Altitude()
        self.__imu = IMU()
        self.__gps = GPS()
        self.__comp_gps = CompGPS()

        # Public Attributes
        self.identification = Identification()
        self.rc_target = Channels()
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

        self.__code_action_map = self.__create_action_map()

    # Private Methods
    def __create_action_map(self):
        code_action_map = {}
        code_action_map[MessageIDs.IDENT] = self.identification.parse
        code_action_map[MessageIDs.STATUS] = self.status.parse
        code_action_map[MessageIDs.RAW_IMU] = self.__imu.parse
        code_action_map[MessageIDs.SERVO] = None
        code_action_map[MessageIDs.MOTOR] = None
        code_action_map[MessageIDs.RC] = self.__rc_actual.parse
        code_action_map[MessageIDs.RAW_GPS] = self.__gps.parse
        code_action_map[MessageIDs.COMP_GPS] = self.__comp_gps.parse
        code_action_map[MessageIDs.ATTITUDE] = self.__attitude.parse
        code_action_map[MessageIDs.ALTITUDE] = self.__altitude.parse
        code_action_map[MessageIDs.ANALOG] = None
        code_action_map[MessageIDs.RC_TUNING] = None
        code_action_map[MessageIDs.PID] = None
        code_action_map[MessageIDs.BOX] = None
        code_action_map[MessageIDs.MISC] = None
        code_action_map[MessageIDs.MOTOR_PINS] = None
        code_action_map[MessageIDs.BOXNAMES] = None
        code_action_map[MessageIDs.PIDNAMES] = None
        code_action_map[MessageIDs.WP] = None
        code_action_map[MessageIDs.BOXIDS] = None
        code_action_map[MessageIDs.SERVO_CONF] = None

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
        # Request Data
        self.__send(MessageIDs.RAW_IMU)
        self.__send(MessageIDs.ALTITUDE)
        self.__send(MessageIDs.ATTITUDE)
        self.__send(MessageIDs.RC)
        self.__send(MessageIDs.RAW_GPS)
        self.__send(MessageIDs.COMP_GPS)
        self.__send(MessageIDs.PID)

        # Set RC values
        data = self.rc_target.get()
        self.__send(
            MessageIDs.SET_RAW_RC,
            len(data)*2,
            data
        )

    def __arm(self):
        self.__print("Arming...")
        self.__rc_actual.arm = self.__ARM_VALUE

        data = self.__rc_actual.get()
        self.__send(
            MessageIDs.SET_RAW_RC,
            len(data)*2,
            data
        )

        self.__is_armed = True
        self.__print("Armed")

    def __disarm(self):
        self.__print("Disarming...")
        # Roll, Pitch, Throttle, Yaw
        self.__rc_actual.arm = OFF_VALUE

        data = self.__rc_actual.get()
        self.__send(
            MessageIDs.SET_RAW_RC,
            len(data)*2,
            data
        )

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

    # Setters
    def set_target_channels(self, channel):
        self.rc_target = Channels.limit(channel)

    # Getters
    def get_imu(self):
        return self.__imu.get()

    def get_attitude(self):
        return self.__attitude.get()

    def get_altitude(self):
        return self.__altitude.get()

    def get_rc_channels(self):
        return self.__rc_actual.get()

    def get_gps(self):
        return self.__gps.get()

    def get_comp_gps(self):
        return self.__comp_gps.get()

    def get_pid(self):
        return self.pid_coef.get()



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

    def get(self):
        identification = [
            self.version,
            self.multi_type,
            self.msp_version,
            self.capability
        ]

        return identification


class Status:
    def __init__(self):
        self.cycleTime = 0
        self.i2c_errors_count = 0
        self.sensor = 0
        self.flag = 0
        self.global_conf = 0

        self.timestamp = 0

    def parse(self, data):
        self.cycleTime = data[0]
        self.i2c_errors_count = data[1]
        self.sensor = data[2]
        self.flag = data[3]
        self.global_conf = data[4]

        self.timestamp = 0

    def get(self):
        status = [
            self.cycleTime,
            self.i2c_errors_count,
            self.sensor,
            self.flag,
            self.global_conf
        ]


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


class Servo:
    # TODO
    pass


class Motor:
    def __init__(self):
        self.m1 = 0
        self.m2 = 0
        self.m3 = 0
        self.m4 = 0

        self.timestamp = None

    def parse(self, data):
        self.m1 = data[0]
        self.m2 = data[1]
        self.m3 = data[2]
        self.m4 = data[3]

        self.timestamp = 0

    def get(self):
        motor = [
            self.m1,
            self.m2,
            self.m3,
            self.m4
        ]

        return motor


class Channels:
    MAX_VALUE = 2100
    MIN_VALUE = 950
    MID_VALUE = 1500

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

    def limit(self, channel):
        if  Channels.MIN_VALUE > channel.roll:
            channel.roll = Channels.MIN_VALUE
        elif channel.roll > Channels.MAX_VALUE:
            channel.roll = Channels.MAX_VALUE

        if  Channels.MIN_VALUE > channel.pitch:
            channel.pitch = Channels.MIN_VALUE
        elif channel.pitch > Channels.MAX_VALUE:
            channel.pitch = Channels.MAX_VALUE

        if  Channels.MIN_VALUE > channel.yaw:
            channel.yaw = Channels.MIN_VALUE
        elif channel.yaw > Channels.MAX_VALUE:
            channel.yaw = Channels.MAX_VALUE

        if  Channels.MIN_VALUE > channel.throttle:
            channel.throttle = Channels.MIN_VALUE
        elif channel.throttle > Channels.MAX_VALUE:
            channel.throttle = Channels.MAX_VALUE

        return channel

    def __add__(self, other):
        self.roll += other.roll
        self.pitch += other.pitch
        self.yaw += other.yaw
        self.throttle += other.throttle

        return self

class GPS:
    def __init__(self):
        self.fix = False
        self.numSat = 0
        self.lat = 0
        self.lon = 0
        self.altitude = 0       # meter
        self.speed = 0          # cm/s
        self.ground_course = 0  # degree*10

        self.timestamp = 0

    def parse(self, data):
        self.fix = data[0]
        self.numSat = data[1]
        self.lat = data[2]
        self.lon = data[3]
        self.altitude = data[4]
        self.speed = data[5]
        self.ground_course = data[6]

        self.timestamp = 0

    def get(self):
        gps = [
            self.fix,
            self.numSat,
            self.lat,
            self.lon,
            self.altitude,
            self.speed,
            self.ground_course
        ]

        return gps

class CompGPS:
    def __init__(self):
        self.distance_to_home = 0   # meters
        self.direction_to_home = 0  # degree (-180,180)
        self.update = False         # boolean

        self.timestamp = None

    def parse(self, data):
        self.distance_to_home = data[0]
        self.direction_to_home = data[1]
        self.update = data[2]

    def get(self):
        comp_gps = [
            self.distance_to_home,
            self.direction_to_home,
            self.update
        ]
        return comp_gps


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
        """

        :return: [Estimated_altitude:cm,
        """
        altitude = [
            self.estalt,
            self.vario
        ]

        return altitude


class Analog:
    pass


class RCTuning:
    pass


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

        self.timestamp = 0

    def parse(self, data):
        self.rp = data[0]
        self.ri = data[1]
        self.rd = data[2]

        self.pp = data[3]
        self.pi = data[4]
        self.pd = data[5]

        self.yp = data[6]
        self.yi = data[7]
        self.yd = data[8]

        self.timestamp = 0
        pass

    def get(self):
        pid = [
            self.rp,
            self.ri,
            self.rd,
            self.pp,
            self.pi,
            self.pd,
            self.yp,
            self.yi,
            self.yd
        ]

        return pid


class Box:
    pass


class Misc:
    pass


class MotorPins:
    pass


class BoxNames:
    pass


class PIDNames:
    pass


class WP:
    pass


class BoxIDs:
    pass


class Servo_Conf:
    pass