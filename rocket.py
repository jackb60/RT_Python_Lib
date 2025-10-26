import serial
import bleak
import struct
from enum import Enum
import time 
import asyncio

class state(Enum):
    GROUND_TESTING = 0
    PRE_FLIGHT = 1
    FLIGHT = 2
    ROLL_CONTROL = 3
    APOGEE = 4
    DISREEF = 5

dummy_data = bytes([85,89,0,0,0,0,0,0,0,0,0,232,163,65,249,255,30,0,9,0,184,10,95,56,5,136,16,207,255,232,7,0,101,250,255,6,0,249,255,252,255,1,60,109,41,66,239,48,142,194,154,153,173,193,102,102,70,64,51,51,19,64,184,30,5,64,114,19,3,0,0,0,0,0,68,8,23,66,21,190,147,193,197,188,205,65,162,196,192,66,122,152,68,65,0,86,250,117,66,163,242,40,192,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,149,46,23])

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
        self.gyro = [0, 0, 0] #x, y, z, deg/s
        self.magnetometer = [0, 0, 0] #x, y, z, Gauss
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
        self.pktnum = 0
        self.ser = serial.Serial('COM11', 115200, timeout=1)
        self.rssi = 0
        self.bleclient = None

    """
    Returns: False if failed (no data/bad data), True if success
    """
    def telemetry_downlink_update(self):
        if self.ser.in_waiting == 0:
            return False
        else:
            packet = dummy_data
            while self.ser.read(1) != bytes([0xAB]) and self.ser.read(1) != bytes([0xAB]): #Wait for 0xABAB
                pass
            while self.ser.read(1) != bytes([0xAB]):
                pass
            packet = self.ser.read(128)
            self.rssi = self.ser.read(1)[0] - 81
            #print(self.rssi)

            #Verify checksum
            chksum = 0
            for i in range(0, 127):
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
            #print("Pyros (0 = FAILURE, 1 = UNCONNECTED, 2 = CONNECTED, 3 = SUCCESS):")
            pyro_info = packet[0] + (packet[1] << 8)
            for i in range(0, 8):
                #Bitmask the two bits we care about
                self.pyros[i] = (pyro_info >> (2 * i)) & 0b11
                #print(i, ": ", end="")
                #print(self.pyros[i])
            
            #Parse servos
            #print("Servos (drive signal in microseconds):")
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
            #print("Accelerometer (m/s^2):")
            #print("X: ", self.accelerometer[0])
            #print("Y: ", self.accelerometer[1])
            #print("Z: ", self.accelerometer[2])

            #Parse barometer
            self.barometer = struct.unpack("<i", packet[20:23] + bytes([0x00]))[0]
            self.temp = struct.unpack("<i", packet[23:26] + bytes([0x00]))[0]
            ##print("Baro Raw:", self.barometer)
            ##print("Temp Raw:", self.temp)
            
            #Parse magnetometer
            def sign_extend(i):
                if(packet[i] & 0x80):
                    return bytes([0xFF])
                else:
                    return bytes([0x00])
            self.magnetometer[0] = struct.unpack("<i", packet[26:29] + sign_extend(28))[0] * 0.0625
            self.magnetometer[1] = struct.unpack("<i", packet[29:32] + sign_extend(31))[0] * 0.0625
            self.magnetometer[2] = struct.unpack("<i", packet[32:35] + sign_extend(34))[0] * 0.0625
            #print("Magnetometer (mG):")
            #print("X: ", self.magnetometer[0])
            #print("Y: ", self.magnetometer[1])
            #print("Z: ", self.magnetometer[2])

            #Parse gyro
            self.gyro[0] = struct.unpack("<h", packet[35:37])[0] * 0.03051757812
            self.gyro[1] = struct.unpack("<h", packet[37:39])[0] * -0.03051757812
            self.gyro[2] = struct.unpack("<h", packet[39:41])[0] * 0.03051757812
            #print("Gyro (deg/s):")
            #print("X: ", self.gyro[0])
            #print("Y: ", self.gyro[1])
            #print("Z: ", self.gyro[2])

            #Parse GPS
            self.gps_fix = packet[41]
            #print("GPS FIX: ", end="")
            if(self.gps_fix):
                #pass
                print("YES")
            else:
                #pass
                print("NO")
            self.lat = struct.unpack("<f", packet[42:46])[0]
            self.lon = struct.unpack("<f", packet[46:50])[0]
            self.gpsalt = struct.unpack("<f", packet[50:54])[0]
            self.pdop = struct.unpack("<f", packet[54:58])[0]
            self.hdop = struct.unpack("<f", packet[58:62])[0]
            self.vdop = struct.unpack("<f", packet[62:66])[0]
            print("LAT: ", self.lat)
            #print("LONG: ", self.lon)
            #print("PDOP: ", self.pdop)
            #print("HDOP: ", self.hdop)
            #print("VDOP: ", self.vdop)

            #Parse Timing
            self.flight_time = struct.unpack("<i", packet[66:70])[0]
            #print("Flight Time (ms):", self.flight_time)
            self.last_rec = struct.unpack("<i", packet[70:74])[0]
            #print("Last Record Time (ms):", self.last_rec)

            #Parse Gyro Integrated
            self.yaw_gyro_int = struct.unpack("<f", packet[74:78])[0]
            self.pitch_gyro_int = struct.unpack("<f", packet[78:82])[0]
            self.roll_gyro_int = struct.unpack("<f", packet[82:86])[0]
            #print("GYRO Integrated (DEG):")
            #print("X: ", self.yaw_gyro_int)
            #print("Y: ", self.pitch_gyro_int)
            #print("Z: ", self.roll_gyro_int)

            self.heading = struct.unpack("<f", packet[86:90])[0]
            #print("MAG HEADING (DEG):", self.heading)

            
            #Parse Battery Voltage
            self.batt_voltage = struct.unpack("<f", packet[90:94])[0]
            #print("Battery Voltage (V):", self.batt_voltage)

            self.state = packet[94]
            #print("State:", self.state)
            
            self.barofilteredalt = struct.unpack("<f", packet[95:99])[0]
            self.barofilteredvelo = struct.unpack("<f", packet[99:103])[0]
            #print("Baro Filtered Alt (m):", self.barofilteredalt)  
            #print("Baro Filtered Velo (m/s):", self.barofilteredvelo)
            
            self.pktnum = struct.unpack("<h", packet[125:127])[0]
            #print("Packet Number:", self.pktnum)

    def ground_downlink_update(self):
        pass


    def set_state(self, state):
        self.ser.write(bytes([0xFE, self.state]))


    def set_pid(self, p, i, d):
        pass


    def pid_activate(self, channels):
        pass


    def fire_pyro(self, channel):
        self.ser.write(bytes([0xFF, 0x01 << channel]))


    def servo_actuate(self, channel, position):
        pass

    def roll_angle_set(self, angle):
        async def roll_angle_set_async(self, angle):
            servo_uuid = "135ba421-a765-41c2-9b46-ab149ed6c933"
            msg = f"{angle}"
            await self.bleclient.write_gatt_char(servo_uuid, msg.encode())
        asyncio.run(roll_angle_set_async(self, angle))

    def ser_clear_buffer(self):
        self.ser.reset_input_buffer()

    def bleconnect(self):
        async def connect():
            address = "CB:37:98:8C:D3:E5"
            self.bleclient = bleak.BleakClient(address)
            await self.bleclient.connect()
        asyncio.run(connect())

a = rocket()
a.bleconnect()
a.roll_angle_set(30)
while True:
    a.roll_angle_set(20)
    #b = input("::")
    #a.roll_angle_set(b)
    #a.ser_clear_buffer()
    #time.sleep(.1)
    a.telemetry_downlink_update()
    #a.fire_pyro(1)
    #time.sleep(.1)
#a.fire_pyro(2)