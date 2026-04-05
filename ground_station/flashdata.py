import serial
import struct
from enum import Enum
import time 
import os
import csv

class state(Enum):
    GROUND_TESTING = 0
    PRE_FLIGHT = 1
    FLIGHT = 2
    APOGEE = 3
    DISREEF = 4

class rocket:
    def __init__(self):
        self.pid = [0, 0, 0] #p, i, d
        self.ser = serial.Serial('COM3', 9600, timeout=1)
        self.logging = False
        self.file = None
        self.csv_writer = None

        self.pyros = [0] * 8
        self.servos = [0] * 8
        self.accelerometer = [0, 0, 0] #x, y, z, m/s^2
        self.barometer = 0
        self.barofilteredalt = 0  #m
        self.barofilteredvelo = 0  #m/s
        self.temp = 0 #deg C
        self.gyro = [0, 0, 0] #x, y, z, deg/s
        self.magnetometer = [0, 0, 0] #x, y, z, Gauss
        self.heading = 0 #deg
        self.gps_fix = False
        self.lat = 0
        self.long = 0
        self.gpsalt = 0 #m
        self.pdop = 0
        self.hdop = 0
        self.vdop = 0
        self.flight_time = 0 #msec
        self.last_rec = 0 #msec
        self.yaw_gyro_int = 0 #deg
        self.pitch_gyro_int = 0 #deg
        self.roll_gyro_int = 0 #deg
        self.batt_voltage = 0 #V
        self.state = state.GROUND_TESTING
        self.pktnum = 0
        self.rssi = 0 #dBm
        self.armed = [0] * 8
        self.fired = [0] * 8
        self.badpackets = 0
        self.rxrssi = 0 #dBm
        self.accel_integrated_velo = 0 #m/s 
        self.baro_max_alt = 0 #m
        self.gps_max_alt = 0 #m
        
    
    def log_data_start(self):
        self.logging = True
        self.file = open(f"telemetry_{time.time()}.csv", "w", newline='')
        self.csv_writer = csv.writer(self.file)

    def log_data_stop(self):
        self.logging = False
        self.file.close()

    """
    Returns: False if failed (no data/bad data), True if success
    """
    def telemetry_downlink_update(self):
        if self.ser.in_waiting == 0:
            return False
        else:
            while self.ser.read(1) != bytes([0xAB]) and self.ser.read(1) != bytes([0xAB]): #Wait for 0xABAB
                pass
            while self.ser.read(1) != bytes([0xAB]):
                pass
            packet = self.ser.read(128)
            print(packet)
            #self.rssi = self.ser.read(1)[0] - 81
            #print(self.rssi)

            #Verify checksum
            chksum = 0
            for i in range(0, 127):
                chksum += packet[i]
                if chksum > 255:
                    chksum %= 256
            
            if chksum != packet[127]:
                return False

            self.rxrssi = packet[106] - 92
            #print("RX RSSI:", self.rxrssi)

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
            armed = packet[103]
            for i in range(0,8):
                self.armed[i] = (armed >> i) & 0x01
                #if self.armed[i]:
                    #print(f"WARNING: Pyro {i} ARMED")
            
            fired = packet[104]
            for i in range(0,8):
                self.fired[i] = (fired >> i) & 0x01
                #if self.fired[i]:
                    #print(f"EVENT: Pyro {i} FIRED")
            
            #Parse servos
            #print("Servos (drive signal in microseconds):")
            servo_info = 0
            for i in range(0, 12):
                servo_info += packet[2 + i] << (8 * i)

            for i in range(0, 8):
                self.servos[i] = (servo_info >> (12 * i)) & 0xFFF
                #print(i, ": ", end="")
                #print(self.servos[i])
            
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
                pass
                #print("YES")
            else:
                pass
                #print("NO")
            self.lat = struct.unpack("<f", packet[42:46])[0]
            self.lon = struct.unpack("<f", packet[46:50])[0]
            self.gpsalt = struct.unpack("<f", packet[50:54])[0]
            self.pdop = struct.unpack("<f", packet[54:58])[0]
            self.hdop = struct.unpack("<f", packet[58:62])[0]
            self.vdop = struct.unpack("<f", packet[62:66])[0]
            #print("LAT: ", self.lat)
            #print("LONG: ", self.lon)
            #print("GPS ALT (m): ", self.gpsalt)
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

            self.state = state(packet[94])
            #print("State:", self.state)
            
            self.barofilteredalt = struct.unpack("<f", packet[95:99])[0]
            self.barofilteredvelo = struct.unpack("<f", packet[99:103])[0]
            #print("Baro Filtered Alt (m):", self.barofilteredalt)  
            #print("Baro Filtered Velo (m/s):", self.barofilteredvelo)

            self.accel_integrated_velo = struct.unpack("<f", packet[107:111])[0]
            #print("Accel Integrated Velo (m/s):", self.accel_integrated_velo)

            self.baro_max_alt = struct.unpack("<f", packet[111:115])[0]
            #print("Baro Max Alt (m):", self.baro_max_alt)

            self.gps_max_alt = struct.unpack("<f", packet[115:119])[0]
            #print("GPS Max Alt (m):", self.gps_max_alt)
            
            self.pktnum = struct.unpack("<h", packet[125:127])[0]
            #print("Packet Number:", self.pktnum)

            self.badpackets = packet[105]
            #print("Bad Packets:", self.badpackets)
            if self.logging:
                data = [
                    time.time(), self.pyros, self.servos, self.accelerometer, self.barometer,
                    self.barofilteredalt, self.barofilteredvelo, self.temp, self.gyro,
                    self.magnetometer, self.heading, self.gps_fix, self.lat, self.long, self.gpsalt,
                    self.pdop, self.hdop, self.vdop, self.flight_time, self.last_rec,
                    self.yaw_gyro_int, self.pitch_gyro_int, self.roll_gyro_int,
                    self.batt_voltage, str(self.state), self.pktnum, self.rssi,
                    self.armed, self.fired, self.badpackets, self.rxrssi,
                    self.accel_integrated_velo, self.baro_max_alt, self.gps_max_alt
                ]
                self.csv_writer.writerow(data)
                self.file.flush()
                os.fsync(self.file.fileno())
            return True

a = rocket()
"""
a.log_data_start()
start = time.time()
while time.time() - start < 5:
    a.telemetry_downlink_update()
#a.arm_pyros([0,1,2,3,4,5])
#a.fire_pyros([0,1,2,3,4,5,6,7])

a = rocket()
#a.zero_roll()
#a.zero_alt()
#a.arm_pyros([2])
#a.fire_pyros([2])
#a.zero_roll()

#time.sleep(5)
#a.servos_set_angle(0)
"""
a.log_data_start()
a.ser.write("1000".encode())
while True:
    a.telemetry_downlink_update()