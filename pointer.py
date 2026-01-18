import math
import serial
import struct
import platform

gps_lat = 42.360370
gps_long = -71.093554

isWin = platform.system() == "Windows"

MAC_HARDWARE_PORT = "/dev/cu.Bluetooth-Incoming-Port"
WIN_HARDWARE_PORT = "COM11"

HARDWARE_PORT = WIN_HARDWARE_PORT if isWin else MAC_HARDWARE_PORT

class pointer:
    def __init__(self, port=HARDWARE_PORT, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.gps_lat = gps_lat
        self.gps_long = gps_long

    def calc_angles(self, rocket_fix, rocket_lat, rocket_long, rocket_alt):
        try:
            ground_station_distance = math.sqrt((rocket_lat - self.gps_lat)**2 + (rocket_long - self.gps_long)**2) * 111100
            print(f"Ground Distance: {ground_station_distance} meters")
            elevation_target = math.atan(rocket_alt/ground_station_distance) * (180 / math.pi) #degrees -90 to 90
            azimuth_target = math.atan2((rocket_long - self.gps_long), (rocket_lat - self.gps_lat)) * (180 / math.pi) #degrees -180 to 180
            azimuth_target = (azimuth_target + 360) % 360  #convert to 0-360 degrees
        except:
            elevation_target = 0
            azimuth_target = 0
        print(f"Azimuth: {azimuth_target}, Elevation: {elevation_target}")
        return azimuth_target, elevation_target

    
    def send_angles(self, azimuth, elevation):
        data = bytearray(11)
        data[0] = 0xAA
        data[1] = 0x00
        struct.pack_into('<f', data, 2, azimuth)
        struct.pack_into('<f', data, 6, elevation)
        data[10] = self.calc_checksum(data)
        self.ser.write(data)
        if self.ser.in_waiting:
            response = self.ser.read(self.ser.in_waiting)
            print(response)

    def calc_checksum(self, data):
        chksum = 0
        for i in range(1, 10): 
            chksum += data[i]
            if chksum > (256 - 1):
                chksum %= 256
        return chksum
    
    def up(self):
        data = bytearray(11)
        data[0] = 0xAA
        data[1] = 0x01
        data[10] = self.calc_checksum(data)
        self.ser.write(data)

    def down(self):
        data = bytearray(11)
        data[0] = 0xAA
        data[1] = 0x02
        data[10] = self.calc_checksum(data)
        self.ser.write(data)

    def left(self):
        data = bytearray(11)
        data[0] = 0xAA
        data[1] = 0x03
        data[10] = self.calc_checksum(data)
        self.ser.write(data)

    def right(self):
        data = bytearray(11)
        data[0] = 0xAA
        data[1] = 0x04
        data[10] = self.calc_checksum(data)
        self.ser.write(data)

    def zero(self):
        data = bytearray(11)
        data[0] = 0xAA
        data[1] = 0x05
        data[10] = self.calc_checksum(data)
        self.ser.write(data)