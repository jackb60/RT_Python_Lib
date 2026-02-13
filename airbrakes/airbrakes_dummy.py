import serial
import struct
import platform
import time


isWin = platform.system() == "Windows"

MAC_HARDWARE_PORT = "/dev/cu.usbmodem21302"
WIN_HARDWARE_PORT = "COM10"

HARDWARE_PORT = WIN_HARDWARE_PORT if isWin else MAC_HARDWARE_PORT

class aribrakes_dummy:
    def __init__(self, port=HARDWARE_PORT, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.deploy_times = []
        self.deploy_fracs = []
        self.sim_start_time = 0

    def calc_checksum(self, data):
        chksum = 0
        for i in range(1, len(data) - 1): 
            chksum += data[i]
            if chksum > (256 - 1):
                chksum %= 256
        return chksum
    
    def send_velo(self, velocity):
        data = bytearray(7)
        data[0] = 0xAA
        data[1] = 0x00
        struct.pack_into('<f', data, 2, velocity)
        data[6] = self.calc_checksum(data)
        self.ser.write(data)

    def send_accel(self, acceleration):
        data = bytearray(7)
        data[0] = 0xAA
        data[1] = 0x01
        struct.pack_into('<f', data, 2, acceleration)
        data[6] = self.calc_checksum(data)
        self.ser.write(data)

    def send_apogee(self, apogee):
        data = bytearray(7)
        data[0] = 0xAA
        data[1] = 0x02
        if apogee: 
            data[2] = 0x01
        data[6] = self.calc_checksum(data)
        self.ser.write(data)

    def send_altitude(self, altitude):
        data = bytearray(7)
        data[0] = 0xAA
        data[1] = 0x04
        struct.pack_into('<f', data, 2, altitude)
        data[6] = self.calc_checksum(data)
        self.ser.write(data)
    
    def start_sim(self):
        data = bytearray(7)
        data[0] = 0xAA
        data[1] = 0x03
        data[6] = self.calc_checksum(data)
        self.ser.write(data)
        self.sim_start_time = time.time()

    def read_response(self):
        if self.ser.in_waiting:
            response = self.ser.readline().decode('utf-8')
            try:
                float_response = float(response)
                self.deploy_times.append(round(time.time() - self.sim_start_time, 2))
                self.deploy_fracs.append(float_response)
            except: 
                print(response, end = '', flush=True)
