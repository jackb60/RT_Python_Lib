import math
import serial
import time
import struct

#you have lat, long, altitude of ground station and rocket

h = 0.0 #feet
x = 0.0 #feet
ground_station_distance = 4*5280 #feet

elevation_target = math.atan(h/ground_station_distance) #radians
azimuth_target = math.atan(x/ground_station_distance) #radians

port = "COM3"
baud = 115200

def send_angles(azimuth, elevation):
    data = bytearray(10)
    data[0] = 0xAA
    struct.pack_into('<f', data, 1, azimuth)
    struct.pack_into('<f', data, 5, elevation)
    data[9] = calc_checksum(data)
    serial.write(data)

def calc_checksum(self, data):
        chksum = 0
        for i in range(1, 9): 
            chksum += data[i]
            if chksum > (256 - 1):
                chksum %= 256
        return chksum

ser = serial.Serial(port, baud, timeout=0.1)

while True:
    send_angles(elevation_target, azimuth_target)