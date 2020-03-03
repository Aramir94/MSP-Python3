import serial
import struct
import time
import queue
from threading import Thread, Lock
from threading import current_thread

from msp.message_ids import MessageIDs


class MultiWii(Thread):
    """Class initialization"""

    def __init__(self, ser_port):
        super(MultiWii, self).__init__(
            name="Comms_Tx"
        )

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

        self.__print = True
        self.__code_action_map = self.__create_action_map()

        # Public Attributes
        self.identification = Identification()
        self.pid_coef = PIDCoefficients()
        self.rc_channels = Channels()

        self.raw_imu = IMU()
        self.motor = Motor()
        self.attitude = Attitude()
        self.altitude = Altitude()

        self.vtx_config = {
            'device': 0,
            'band': 0,
            'channel': 0,
            'power': 0,
            'pit': 0,
            'unknown': 0
        }

        self.ser = None
        self.__init_comms(ser_port)

        # Time to wait until the board becomes operational
        wakeup = 2
        try:
            self.ser.open()
            if self.__print:
                print("Waking up board on " + self.ser.port + "...")
            for i in range(1, wakeup):
                if self.__print:
                    print(wakeup - i)
                time.sleep(1)
        except Exception as error:
            print("\n\nError opening " + self.ser.port + " port.\n" + str(error) + "\n\n")

    # Private Methods
    def __create_action_map(self):
        code_action_map = {}
        code_action_map[MessageIDs.IDENT] = self.get_ident
        """
            STATUS
        """
        code_action_map[MessageIDs.RAW_IMU] = self.get_imu
        """
            SERVO
            MOTOR
        """
        code_action_map[MessageIDs.RC] = self.get_rc
        """
            RAW_GPS
            COMP_GPS
        """
        code_action_map[MessageIDs.ATTITUDE] = self.get_attitude
        code_action_map[MessageIDs.ALTITUDE] = self.get_altitude
        """
            ANALOG
            RC_TUNING
            PID
            BOX
            MISC
            MOTOR_PINS
            BOXNAMES
            PIDNAMES
            WP
            BOXIDS
            RC_RAW_IMU
            SET_RAW_RC
            SET_RAW_GPS
            SET_PID
            SET_BOX
            SET_RC_TUNING
            ACC_CALIBRATION
            MAG_CALIBRATION
            SET_MISC
            RESET_CONF
            SET_WP
            SWITCH_RC_SERIAL
            IS_SERIAL
            DEBUG
            VTX_CONFIG
            VTX_SET_CONFIG
            EEPROM_WRITE
            REBOOT
        """

        return code_action_map

    def __init_comms(self, ser_port):
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
        # self.ser.writeTimeout = 2

        if self.__print:
            print("Serial Communication Initialized")

    def __on_thread(self, function, *args, **kwargs):
        self.__q.put((function, args, kwargs))

    def __shutdown(self):
        print(current_thread().name + " - Shutting Down")
        self.__running = False

    def __idle(self):
        if self.__print:
            print("Idling - IMU")
        # TODO create looping control logic
        self.__send(0, MessageIDs.RAW_IMU)
        self.__send(0, MessageIDs.ALTITUDE)
        if self.__print:
            print("Idling - ATTITUDE")
        self.__send(0, MessageIDs.ATTITUDE)
        time.sleep(.5)

    def __arm(self):
        # Roll, Pitch, Throttle, Yaw
        data = [1500, 1500, 1000, 2000]
        self.__send(8, MessageIDs.SET_RAW_RC, data)
        self.__is_armed = True
        # TODO use a different channel to arm the drone

    def __disarm(self):
        # Roll, Pitch, Throttle, Yaw
        data = [1500, 1500, 1000, 1000]
        self.__send(8, MessageIDs.SET_RAW_RC, data)
        self.__is_armed = False
        # TODO use a different channel to disarm the drone

    def __send(self, data_length, code: MessageIDs, data=None):
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
            b = self.ser.write(struct.pack('<3c2B%dHB' % len(data), *total_data))
            # self.ser.flushOutput()
        except Exception as error:
            import traceback
            print("\n\nError in send.")
            print("(" + str(error) + ")")
            traceback.print_exc()

    def __receive(self):
        if self.__print:
            print("Starting " + current_thread().name)
        while self.__running:
            try:
                while True:
                    header = self.ser.read().decode('utf-8')
                    if header == '$':
                        header = header + self.ser.read(2).decode('utf-8')
                        break

                data_length = struct.unpack('<B', self.ser.read())[0]
                code = struct.unpack('<B', self.ser.read())[0]
                data = self.ser.read(data_length)
                if self.__print:
                    print("Receiving - " + str(code))
                checksum = struct.unpack('<B', self.ser.read())[0]
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

                self.ser.flushInput()

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
        self.__rx_thread.start()

        while self.__running:
            try:
                function, args, kwargs = self.__q.get(timeout=self.__timeout)
                function(*args, **kwargs)
            except queue.Empty:
                self.__idle()

    def is_armed(self):
        return self.__is_armed

    def arm(self):
        self.__on_thread(self.__arm)

    def disarm(self):
        self.__on_thread(self.__disarm)

    def calibrate_acc(self):
        self.__on_thread(self.__send, [0, MessageIDs.ACC_CALIBRATION, []])

    def get_ident(self, data):
        self.identification.version = data[0]
        self.identification.multi_type = data[1]
        self.identification.msp_version = data[2]
        self.identification.capability = data[3]
        self.identification.timestamp = None

    def get_status(self, data):
        pass

    def get_rc(self, data):
        self.rc_channels.roll = data[0]
        self.rc_channels.pitch = data[1]
        self.rc_channels.yaw = data[2]
        self.rc_channels.throttle = data[3]
        self.rc_channels.timestamp = data[4]

    def get_imu(self, data):
        self.raw_imu.ax = data[0]
        self.raw_imu.ay = data[1]
        self.raw_imu.az = data[2]

        self.raw_imu.gx = data[3]
        self.raw_imu.gy = data[4]
        self.raw_imu.gz = data[5]

        self.raw_imu.mx = data[6]
        self.raw_imu.my = data[7]
        self.raw_imu.mz = data[8]

        self.raw_imu.timestamp = None

    def get_attitude(self, data):
        self.attitude.angx = data[0]
        self.attitude.angy = data[1]
        self.attitude.heading = data[2]

    def get_altitude(self, data):
        self.altitude.estalt = data[0]
        self.altitude.variometer = data[1]


class Identification:
    def __init__(self):
        self.version = 0
        self.multi_type = 0
        self.msp_version = 0
        self.capability = 0
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
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.throttle = 0
        self.timestamp = 0


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

    def __str__(self):
        result = "Attitude: \n\t"
        temp_dict = {'angx': self.angx, 'angy': self.angy, 'heading': self.heading}
        return result + str(temp_dict)


class Altitude:
    def __init__(self):
        self.estalt = 0
        self.vario = 0

        self.timestamp = None

    def __str__(self):
        result = "Altitude: \n\t"
        temp_dict = {'estalt': self.estalt, 'vario': self.vario}
        return result + str(temp_dict)
