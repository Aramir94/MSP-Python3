import serial
import struct
import time

from msp.message_ids import MessageIDs

class MultiWii:
    """Class initialization"""

    def __init__(self, ser_port):

        self.code_action_map = self.create_action_map()

        self.ident = Identification()
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

        self.PRINT = 1



        self.ser = None
        self.init_comms(ser_port)

        # Time to wait until the board becomes operational
        wakeup = 2
        try:
            self.ser.open()
            if self.PRINT:
                print("Waking up board on " + self.ser.port + "...")
            for i in range(1, wakeup):
                if self.PRINT:
                    print(wakeup - i)
                time.sleep(1)
        except Exception as error:
            print("\n\nError opening " + self.ser.port + " port.\n" + str(error) + "\n\n")

    def init_comms(self, ser_port):
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

    def arm(self):
        """
        Sends an arming command to the Flight Controller.

        :return: None
        """

        # Roll, Pitch, Throttle, Yaw
        data = [1500, 1500, 1000, 2000]
        self.send(8, MessageIDs.SET_RAW_RC, data)
        self.receive()

    def disarm(self):
        """
        Sends a disarming command to the Flight Controller.

        :return: None
        """

        # Roll, Pitch, Throttle, Yaw
        data = [1500, 1500, 1000, 1000]
        self.send(8, MessageIDs.SET_RAW_RC, data)
        self.receive()

    # def setPID(self, pd):
    #     nd = []
    #     for i in np.arange(1, len(pd), 2):
    #         nd.append(pd[i] + pd[i + 1] * 256)
    #     data = pd
    #     print("PID sending:", data)
    #     self.sendCMDreceiveATT(30, SET_PID, data)
    #     self.sendCMDreceiveATT(0, EEPROM_WRITE, [])

    # def setVTX(self, band, channel, power):
    #     band_channel = ((band - 1) << 3) | (channel - 1)
    #     t = None
    #     while t == None:
    #         t = self.getData(VTX_CONFIG)
    #     different = (self.vtxConfig['band'] != band) | (self.vtxConfig['channel'] != channel) | (
    #                 self.vtxConfig['power'] != power)
    #     data = [band_channel, power, self.vtxConfig['pit']]
    #     while different:
    #         self.sendCMDreceiveATT(4, VTX_SET_CONFIG, data, 'H2B')
    #         time.sleep(1)
    #         self.sendCMDreceiveATT(0, EEPROM_WRITE, [], '')
    #         self.ser.close()
    #         time.sleep(3)
    #         self.ser.open()
    #         time.sleep(3)
    #         t = None
    #         while t == None:
    #             t = self.getData(VTX_CONFIG)
    #         print(t)
    #         different = (self.vtxConfig['band'] != band) | (self.vtxConfig['channel'] != channel) | (
    #                     self.vtxConfig['power'] != power)

    def send(self, data_length, code: MessageIDs, data=None):
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

    def receive(self):
        try:
            while True:
                header = self.ser.read().decode('utf-8')
                if header == '$':
                    header = header + self.ser.read(2).decode('utf-8')
                    break

            data_length = struct.unpack('<B', self.ser.read())[0]
            code = struct.unpack('<B', self.ser.read())[0]
            data = self.ser.read(data_length)
            checksum = struct.unpack('<B', self.ser.read())[0]

            # print("code: " + str(code))
            print("data_length: " + str(data_length))
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
            self.code_action_map[code](temp)
        except KeyError as err:
            print(err)

    def calibrate_acc(self):
        self.send(0, MessageIDs.ACC_CALIBRATION, [])
        self.receive()

    def create_action_map(self):
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

    def get_ident(self, data):
        self.ident.version = data[0]
        self.ident.multi_type = data[1]
        self.ident.msp_version = data[2]
        self.ident.capability = data[3]
        self.ident.timestamp = None

    def get_status(self, data):
        pass

    def get_rc(self, data):
        """

        :param data:
        :return:
        """
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

    def request_identification(self):
        self.send(0, MessageIDs.IDENT)
        self.receive()

    def request_imu(self):
        self.send(0, MessageIDs.RAW_IMU)
        self.receive()

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
