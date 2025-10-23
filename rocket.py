import serial
import bleak
import struct
from enum import Enum

class state(Enum):
    GROUND_TESTING = 0
    PRE_FLIGHT = 1
    FLIGHT = 2
    ROLL_CONTROL = 3
    APOGEE = 4
    DISREEF = 5

dummy_data = bytes([85,85,0,0,0,0,0,0,0,0,0,232,163,65,1,0,23,0,15,0,56,62,95,248,130,134,81,228,255,135,248,255,210,234,255,7,0,0,0,253,255,0,0,0,0,0,0,0,0,0,0,0,0,0,225,250,199,66,225,250,199,66,225,250,199,66,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])

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
        self.heading = 0
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
        if False:#self.ser.in_waiting == 0:
            return False
        else:
            #while int.from_bytes(self.ser.read(2)) != 0xABAB: #Wait for 0xABAB
            #    pass
            #packet = self.ser.read(128)

            #Verify checksum
            #for i in range(0, 128):
            #    chksum += packet[i]
            #    if chksum > 255:
            #        chksum %= 256
            
            #if chksum != packet[127]:
            #    return False

            """
            Parse pyros
            For each pyro:
                0 = Fail (Still connected after fired)
                1 = Unconnected
                2 = Connected
                3 = Fired Successfully
            """
            print("Pyros:")
            packet = dummy_data
            pyro_info = packet[0] + (packet[1] << 8)
            for i in range(0, 8):
                #Bitmask the two bits we care about
                self.pyros[i] = (pyro_info >> (2 * i)) & 0b11
                print(i, ": ", end="")
                print(self.pyros[i])
            
            #Parse servos
            print("Servos:")
            servo_info = 0
            for i in range(0, 12):
                servo_info += packet[2 + i] << (8 * i)

            for i in range(0, 8):
                self.servos[i] = (servo_info >> (12 * i)) & 0xFFF
                print(i, ": ", end="")
                print(self.servos[i])
            
            #Parse accelerometer
            self.accelerometer[0] = struct.unpack("<h", packet[14:16])[0] * 0.48052585
            self.accelerometer[1] = struct.unpack("<h", packet[16:18])[0] * 0.48052585
            self.accelerometer[2] = struct.unpack("<h", packet[18:20])[0] * 0.48052585
            print("Accelerometer:")
            print("X: ", self.accelerometer[0])
            print("Y: ", self.accelerometer[1])
            print("Z: ", self.accelerometer[2])

            #Parse barometer
            self.barometer = struct.unpack("<i", packet[20:23] + bytes([0x00]))[0]
            self.temp = struct.unpack("<i", packet[23:26] + bytes([0x00]))[0]
            print("Baro Raw:", self.barometer)
            print("Temp Raw:", self.temp)
            
            #Parse magnetometer
            self.magnetometer[0] = struct.unpack("<i", packet[26:29] + bytes([0x00]))[0]
            self.magnetometer[1] = struct.unpack("<i", packet[29:32] + bytes([0x00]))[0]
            self.magnetometer[2] = struct.unpack("<i", packet[32:35] + bytes([0x00]))[0]
            print("Magnetometer:")
            print("X: ", self.magnetometer[0])
            print("Y: ", self.magnetometer[1])
            print("Z: ", self.magnetometer[2])
            
            #Parse GPS
            self.gps_fix = packet[41]
            if(self.gps_fix):
                print("GPS FIX")
            else:
                print("NO FIX")
            self.lat = struct.unpack("<f", packet[42:46])[0]
            self.lon = struct.unpack("<f", packet[46:50])[0]
            self.gpsalt = struct.unpack("<f", packet[50:54])[0]
            self.pdop = struct.unpack("<f", packet[54:58])[0]
            self.hdop = struct.unpack("<f", packet[58:62])[0]
            self.vdop = struct.unpack("<f", packet[62:66])[0]
            print("LAT: ", self.lat)
            print("LONG: ", self.lon)
            print("PDOP: ", self.pdop)
            print("HDOP: ", self.hdop)
            print("VDOP: ", self.vdop)

            #Parse Timing
            self.flight_time = struct.unpack("<i", packet[66:70])[0]
            self.last_rec = struct.unpack("<i", packet[70:74])

            #Parse Gyro Integrated
            self.yaw_gyro_int = struct.unpack("<f", packet[74:78])[0]
            self.pitch_gyro_int = struct.unpack("<f", packet[78:82])[0]
            self.roll_gyro_int = struct.unpack("<f", packet[82:86])[0]
            print("GYRO:")
            print(self.yaw_gyro_int)
            print(self.pitch_gyro_int)
            print(self.roll_gyro_int)

            self.heading = struct.unpack("<f", packet[86:90])[0]
            print("MAG HEADING:", self.heading)

            
            #Parse Battery Voltage
            self.batt_voltage = struct.unpack("<f", packet[90:94])[0]

            self.state = packet[94]
            
            self.barofilteredalt = struct.unpack("<f", packet[95:99])[0]

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

a = rocket()
a.telemetry_downlink_update()