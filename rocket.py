import Serial
import bleak
import struct
from enum import Enum

class state(Enum):
    GROUND_TESTING = 0
    PRE_FLIGHT = 1
    FLIGHT = 2
    ROLL_CONTROL = 3
    APOGEE = 3
    DISREEF = 4


class rocket:
    def __init__(self):
        self.pid = [0, 0, 0] #p, i, d
        self.pyros = [0] * 8
        self.servos = [0] * 8
        self.accelerometer = [0, 0, 0] #x, y, z, m/s^2
        self.barometer = 0
        self.barofilteredalt = 0  #m
        self.barofilteredvelo = 0  #m/s
        self.temp = 0
        self.magnetometer = [0, 0, 0] #x, y, z
        self.gps_fix = False
        self.lat = 0
        self.long = 0
        self.gpsalt = 0
        self.pdop = 0
        self.hdop = 0
        self.vdop = 0
        self.flight_time = 0
        self.last_rec = 0
        self.yaw_gyro_int = 0
        self.pitch_gyro_int = 0
        self.roll_gyro_int = 0
        self.batt_voltage = 0
        self.state = state.GROUND_TESTING

    """
    Returns: False if failed (no data/bad data), True if success
    """
    def telemetry_downlink_update(self):
        if self.ser.in_waiting == 0:
            return False
        else:
            while int.from_bytes(self.ser.read(2)) != 0xABAB: #Wait for 0xABAB
                pass
            packet = self.ser.read(128)

            #Verify checksum
            for i in range(0, 128):
                chksum += packet[i]
                if chksum > 255:
                    chksum %= 256
            
            if chksum != packet[127]:
                return False

            """
            Parse pyros
            For each pyro:
                0 = Fail (Still connected after fired)
                1 = Unconnected
                2 = Connected
                3 = Fired Successfully
            """
            pyro_info = (packet[0] << 8) + packet[1]
            for i in range(0, 8):
                #Bitmask the two bits we care about
                self.pyros[i] = (pyro_info >> (2 * i)) | 0b11

            #Parse servos
            servo_info = 0
            for i in range(0, 12):
                servo_info += packet[2 + i] << (8 * i)

            for i in range(0, 8):
                self.servos[i] = (servo_info << (12 * i)) | 0xFFF

            #Parse accelerometer
            self.accelerometer[0] = struct.unpack("<h", packet[14:16])[0]
            self.accelerometer[1] = struct.unpack("<h", packet[16:18])[0]
            self.accelerometer[2] = struct.unpack("<h", packet[18:20])[0]

            #Parse barometer
            #self.barometer = struct.unpack("<i",)[0] #TODO

            #Parse temp
            #self.temp = struct.unpack("<i", )[0] #TODO

            #Parse magnetometer
            #TODO

            #Parse GPS
            self.gps_fix = packet[32] & 0x01
            self.lat = struct.unpack("<f", packet[39:43])[0]
            self.long = struct.unpack("<f", packet[43:47])[0]
            self.gpsalt = struct.unpack("<f", packet[47:51])[0]
            self.pdop = struct.unpack("<f", packet[51:55])[0]
            self.hdop = struct.unpack("<f", packet[55:59])[0]
            self.vdop = struct.unpack("<f", packet[59:63])[0]

            #Parse timing
            self.flight_time = struct.unpack("<i", packet[63:67])[0]
            self.last_rec = struct.unpack("<i", packet[67:71])[0]

            #Parse Gyro Integrated
            self.yaw_gyro_int = struct.unpack("<f", packet[71:75])[0]
            self.pitch_gyro_int = struct.unpack("<f", packet[75:79])[0]
            self.roll_gyro_int = struct.unpack("<f", packet[79:83])[0]

            #Parse Battery Voltage
            self.batt_voltage = struct.unpack("<f", packet[83:87])[0]

            #Parse State
            #TODO

    def ground_downlink_update(self):
        pass


    def set_state(self, state):
        pass


    def set_pid(self, p, i, d):
        pass


    def pid_activate(self, channels):
        pass


    def fire_pyro(self, channel):
        pass


    def servo_actuate(self, channel, position):
        pass

