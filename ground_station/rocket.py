import serial
import struct
from enum import Enum
import time 
import os
import csv
import sys
import numpy as np

class state(Enum):
    GROUND_TESTING = 0
    PRE_FLIGHT = 1
    FLIGHT = 2
    APOGEE = 3
    MAIN = 4
    END = 5

class rocket:
    def __init__(self):
        #to-do: clean this up
        
        self.pid = [0, 0, 0] #p, i, d
        self.ser = None# serial.Serial('COM11', 115200, timeout=1)
        self.logging = False
        self.file = None
        self.file_badpackets = None
        self.csv_writer = None
        self.csv_writer_badpackets = None

        self.pyros = [0] * 6
        self.servos = [0] * 4
        self.servos_deg = [0] * 4
        self.accelerometer = [0, 0, 0] #x, y, z, m/s^2
        self.barometer = 0
        self.barofilteredalt = 0  #m
        self.barofilteredvelo = 0  #m/s
        self.temp = 0 #deg C # from baro
        self.gyro = [0, 0, 0] #x, y, z, deg/s
        self.gps_fix = False
        self.lat = 0
        self.lon = 0
        self.gpsalt = 0 #m
        self.gps_horiz_prec = -1
        self.gps_vert_prec = -1
        self.gps_num_sat = 0
        self.flight_time = 0 #msec
        self.yaw_gyro_int = 0 #deg
        self.pitch_gyro_int = 0 #deg
        self.roll_gyro_int = 0 #deg
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
        self.pyro_resistances = [0] * 8
        self.cell_voltages = [0] * 3
        self.total_current = 0
        self.converter_voltages = [0] * 6
        self.converter_currents = [0] * 6
        self.bms_protections_enabled = 0
        self.bms_protection_status = 0
        self.bms_temp = 0 # battery source


        self.enabled_status = [1] * 6 # 1 for True, 0 for False
        # order:   3 3.3 5 7.4 8.4 28
        # index    0  1  2  3   4  5
        # Todo: write method to update these
    
        self.angleFromVertical = 0 

        self.gnd_lat = 0
        self.gnd_lon = 0
        self.gnd_fix = False
        self.gnd_alt = 0

        self.debug = True




        ### FUDGING FOR TESTING

        """self.gnd_alt = 100
        self.gnd_lat = 0.01
        self.gnd_fix = True

        self.barofilteredalt = 1000
        self.FUDGED_BARO = True
        self.gps_fix = True"""
        
    
    def log_data_start(self):
        self.logging = True
        self.file = open(f"telemetry/telemetry_{time.time()}.csv", "w", newline='')
        self.file_badpackets = open(f"telemetry/telemetry_badpackets_{time.time()}.csv", "w", newline='')
        self.csv_writer = csv.writer(self.file)
        self.csv_writer_badpackets = csv.writer(self.file_badpackets)
        header = [
        "timestamp",
        "pyros",
        "servos",
        "servos_deg",
        "accelerometer",
        "barofilteredalt",
        "temp",
        "gyro",
        "gps_fix",
        "lat",
        "lon",
        "gpsalt",
        "gps_horiz_prec",
        "gps_vert_prec",
        "gps_num_sat",
        "flight_time",
        "yaw_gyro_int",
        "pitch_gyro_int",
        "roll_gyro_int",
        "state",
        "pktnum",
        "rssi",
        "armed_pyros",
        "fired_pyros",
        "badpackets",
        "rxrssi",
        "accel_integrated_velo",
        "baro_max_alt",
        "gps_max_alt",
        "pyro_resistances",
        "cell_voltages",
        "total_current",
        "converter_voltages",
        "converter_currents",
        "bms_protections_enabled",
        "bms_protection_status",
        "bms_temp",
        "enabled_status",
        "angleFromVertical",
        "gnd_lat",
        "gnd_lon",
        "gnd_fix",
        "gnd_alt"
        ]
        self.csv_writer.writerow(header)
        self.csv_writer_badpackets.writerow(header)
        self.file.flush()
        self.file_badpackets.flush()
        os.fsync(self.file.fileno())
        os.fsync(self.file_badpackets.fileno())

    def log_data_stop(self):
        self.logging = False
        self.file.close()
        self.file_badpackets.close()

    """
    Returns: False if failed (no data/bad data), True if success
    """
    def telemetry_downlink_update(self):
        flag_bad_packet = False
        if self.ser is None or not self.ser.is_open:
            return False
        if self.ser.in_waiting == 0:
            return False
        else:
            while self.ser.read(1) != bytes([0xAB]) and self.ser.read(1) != bytes([0xAB]): #Wait for 0xABAB
                pass
            while self.ser.read(1) != bytes([0xAB]):
                pass
            packet = self.ser.read(128)
            gnd_info = self.ser.read(14)
            self.rssi = struct.unpack("<b", gnd_info[0:1])[0] - 99
            self.gnd_fix = True if (gnd_info[1] == 1) else False
            self.gnd_lat = np.round(struct.unpack("<l", gnd_info[2:6])[0] * 1e-7,5)
            self.gnd_lon = np.round(struct.unpack("<l", gnd_info[6:10])[0] * 1e-7,5)
            self.gnd_alt = np.round(struct.unpack("<l", gnd_info[10:14])[0] / 1000,2)
            
            #Verify checksum
            chksum = 0
            for i in range(0, 127):
                chksum += packet[i]
                if chksum > 255:
                    chksum %= 256
            
            if chksum != packet[127]:
                flag_bad_packet = True
            
            """
            Parse pyros
            For each pyro:
                0 = Fail (Still connected after fired)
                1 = Unconnected
                2 = Connected
                3 = Fired Successfully
            """
            if not flag_bad_packet:
                pyro_info = packet[0] + (packet[1] << 8)
                for i in range(0, 6):
                    self.pyros[i] = (pyro_info >> (2 * i)) & 0b11
                
                armed = packet[2]
                for i in range(0, 6):
                    self.armed[i] = (armed >> i) & 0x01
                
                fired = packet[3]
                for i in range(0, 6):
                    self.fired[i] = (fired >> i) & 0x01

                for i in range(0, 6):
                    self.pyro_resistances[i] = packet[4 + i] / 10
                
                #Parse servos
                #to-do: convert to angles
                servo_info = 0
                for i in range(0, 6):
                    servo_info += packet[10 + i] << (8 * i)

                for i in range(0, 4):
                    self.servos[i] = (servo_info >> (12 * i)) & 0xFFF
                    func = self.microsecToDeg_airbrakes if i in [0,1] else self.microsecToDeg_rollCtrl
                    self.servos_deg[i] = np.round(func(self.servos[i]),2)
                    #print("[RKT] read servo {} to degrees {}".format(i,self.servos_deg[i]))
                
                #Parse accelerometer
                self.accelerometer[0] = int.from_bytes(packet[16:19], byteorder='little', signed=True) / 12800.0 * 9.80665
                self.accelerometer[1] = int.from_bytes(packet[19:22], byteorder='little', signed=True) / 12800.0 * 9.80665
                self.accelerometer[2] = int.from_bytes(packet[22:25], byteorder='little', signed=True) / 12800.0 * 9.80665

                #Parse gyro
                self.gyro[0] = struct.unpack("<h", packet[25:27])[0] * 0.03051757812
                self.gyro[1] = struct.unpack("<h", packet[27:29])[0] * -0.03051757812
                self.gyro[2] = struct.unpack("<h", packet[29:31])[0] * 0.03051757812

                #Parse GPS
                self.gps_fix = packet[31]
                self.lat = struct.unpack("<l", packet[32:36])[0] * 1e-7
                self.lon = struct.unpack("<l", packet[36:40])[0] * 1e-7
                self.gpsalt = struct.unpack("<f", packet[40:44])[0]
                self.gps_horiz_prec = struct.unpack("<L", packet[44:48])[0] / 1000
                self.gps_vert_prec = struct.unpack("<L", packet[48:52])[0] / 1000
                self.gps_num_sat = packet[52]

                #Parse barometer
                C1 = 0xA27A
                C2 = 0x92E4
                C3 = 0x6951
                C4 = 0x61EF
                C5 = 0x91E3
                C6 = 0x6FEC
                self.raw_press = int.from_bytes(packet[53:56], byteorder='little', signed=False)
                self.raw_temp = int.from_bytes(packet[56:59], byteorder='little', signed=False)

                #Temperature conversion
                dT = float(self.raw_temp) - (float(C5) * (1 << 8))
                TEMP = 2000.0 + dT * float(C6) / float(1 << 23)
                self.temp = TEMP / 100.0  # °C

                #to-do: Pressure/Alt conversion
                
                #Parse moving avg height

                self.barofilteredalt = struct.unpack("<f", packet[59:63])[0]

                #if not self.FUDGED_BARO:
                #    self.barofilteredalt = struct.unpack("<f", packet[59:63])[0]

                #Parse state
                self.state = state(packet[63])

                #Parse Gyro Integrated
                self.roll_gyro_int = struct.unpack("<f", packet[64:68])[0]
                self.pitch_gyro_int = struct.unpack("<f", packet[68:72])[0]
                self.yaw_gyro_int = struct.unpack("<f", packet[72:76])[0]

                #Parse Max Alts
                self.baro_max_alt = struct.unpack("<H", packet[76:78])[0]
                self.gps_max_alt = struct.unpack("<H", packet[78:80])[0]
                
                #Parse Timing
                self.flight_time = struct.unpack("<L", packet[80:84])[0]

                #Parse Packet Number
                self.pktnum = struct.unpack("<H", packet[84:86])[0]
                
                #Parse RSSI
                self.rxrssi = struct.unpack("<b", packet[86:87])[0] - 99

                #Parse BMS
                for i in range(0, 3):
                    self.cell_voltages[i] = struct.unpack("<h", packet[87 + 2 * i: 89 + 2 * i])[0] / 1000.0
                self.total_current = struct.unpack("<h", packet[93:95])[0] / -1000.0
                self.bms_temp = packet[95] / 2
                self.bms_protection_status = packet[96]
                self.bms_protections_enabled = packet[97]

                for i in range(0, 6):
                    self.converter_voltages[i] = struct.unpack("<h", packet[98 + 2 * i: 100 + 2 * i])[0] * 0.0016
                    self.converter_currents[i] = struct.unpack("<h", packet[110 + 2 * i: 112 + 2 * i])[0] * 0.000625

                correct_voltages = [3,3.3,5,7.4,8.4,28]
                for i in range(0, 6):
                    self.enabled_status[i] = np.abs(correct_voltages[i] - self.converter_voltages[i]) < 1
                
                #Parse Integrated Acceleration
                self.accel_integrated_velo = struct.unpack("<f", packet[122:126])[0]

                self.angleFromVertical = np.arccos(np.cos(self.pitch_gyro_int * np.pi / 180.0) * np.cos(self.yaw_gyro_int * np.pi / 180)) * 180.0 / np.pi


                if self.logging:
                    data = [
                        time.time(),
                        self.pyros,
                        self.servos,
                        self.servos_deg,
                        self.accelerometer,
                        self.barofilteredalt,
                        self.temp,
                        self.gyro,
                        self.gps_fix,
                        self.lat,
                        self.lon,
                        self.gpsalt,
                        self.gps_horiz_prec,
                        self.gps_vert_prec,
                        self.gps_num_sat,
                        self.flight_time,
                        self.yaw_gyro_int,
                        self.pitch_gyro_int,
                        self.roll_gyro_int,
                        str(self.state),
                        self.pktnum,
                        self.rssi,
                        self.armed,
                        self.fired,
                        self.badpackets,
                        self.rxrssi,
                        self.accel_integrated_velo,
                        self.baro_max_alt,
                        self.gps_max_alt,
                        self.pyro_resistances,
                        self.cell_voltages,
                        self.total_current,
                        self.converter_voltages,
                        self.converter_currents,
                        self.bms_protections_enabled,
                        self.bms_protection_status,
                        self.bms_temp,
                        self.enabled_status,
                        self.angleFromVertical,
                        self.gnd_lat,
                        self.gnd_lon,
                        self.gnd_fix,
                        self.gnd_alt
                    ]
                    self.csv_writer.writerow(data)
                    self.file.flush()
                    os.fsync(self.file.fileno())

            else: # Bad Packet
                pyro_info = packet[0] + (packet[1] << 8)
                pyros = [0,0,0,0,0,0]
                for i in range(0, 6):
                    pyros[i] = (pyro_info >> (2 * i)) & 0b11
                
                armed0 = packet[2]
                armed = [0,0,0,0,0,0]
                for i in range(0, 6):
                    armed[i] = (armed0 >> i) & 0x01
                
                fired0 = packet[3]
                fired = [0,0,0,0,0,0]
                for i in range(0, 6):
                    fired[i] = (fired0 >> i) & 0x01

                pyro_resistances = [0,0,0,0,0,0]
                for i in range(0, 6):
                    pyro_resistances[i] = packet[4 + i] / 10
                
                #Parse servos
                servo_info = 0
                for i in range(0, 6):
                    servo_info += packet[10 + i] << (8 * i)

                servos = [0,0,0,0]
                servos_deg = [0,0,0,0]
                for i in range(0, 4):
                    servos[i] = (servo_info >> (12 * i)) & 0xFFF
                    func = self.microsecToDeg_airbrakes if i in [0,1] else self.microsecToDeg_rollCtrl
                    servos_deg[i] = func(servos[i])
                    print("[RKT] [badpacket] read servo {} to degrees {}".format(i,servos_deg[i]))
                
                #Parse accelerometer
                accelerometer = [0,0,0]
                accelerometer[0] = int.from_bytes(packet[16:19], byteorder='little', signed=True) / 12800.0 * 9.80665
                accelerometer[1] = int.from_bytes(packet[19:22], byteorder='little', signed=True) / 12800.0 * 9.80665
                accelerometer[2] = int.from_bytes(packet[22:25], byteorder='little', signed=True) / 12800.0 * 9.80665

                #Parse gyro
                gyro = [0,0,0]
                gyro[0] = struct.unpack("<h", packet[25:27])[0] * 0.03051757812
                gyro[1] = struct.unpack("<h", packet[27:29])[0] * -0.03051757812
                gyro[2] = struct.unpack("<h", packet[29:31])[0] * 0.03051757812

                #Parse GPS
                gps_fix = packet[31]
                lat = struct.unpack("<l", packet[32:36])[0] * 1e-7
                lon = struct.unpack("<l", packet[36:40])[0] * 1e-7
                gpsalt = struct.unpack("<f", packet[40:44])[0]
                gps_horiz_prec = struct.unpack("<L", packet[44:48])[0] / 1000
                gps_vert_prec = struct.unpack("<L", packet[48:52])[0] / 1000
                gps_num_sat = packet[52]

                #Parse barometer
                C1 = 0xA27A
                C2 = 0x92E4
                C3 = 0x6951
                C4 = 0x61EF
                C5 = 0x91E3
                C6 = 0x6FEC
                raw_press = int.from_bytes(packet[53:56], byteorder='little', signed=False)
                raw_temp = int.from_bytes(packet[56:59], byteorder='little', signed=False)

                #Temperature conversion
                dT = float(raw_temp) - (float(C5) * (1 << 8))
                TEMP = 2000.0 + dT * float(C6) / float(1 << 23)
                temp = TEMP / 100.0  # °C

                #to-do: Pressure/Alt conversion
                
                #Parse moving avg height

                barofilteredalt = struct.unpack("<f", packet[59:63])[0]

                #if not FUDGED_BARO:
                #    barofilteredalt = struct.unpack("<f", packet[59:63])[0]

                #Parse state
                state0 = state(packet[63])

                #Parse Gyro Integrated
                roll_gyro_int = struct.unpack("<f", packet[64:68])[0]
                pitch_gyro_int = struct.unpack("<f", packet[68:72])[0]
                yaw_gyro_int = struct.unpack("<f", packet[72:76])[0]

                #Parse Max Alts
                gps_max_alt = struct.unpack("<H", packet[76:78])[0]
                baro_max_alt = struct.unpack("<H", packet[78:80])[0]
                
                #Parse Timing
                flight_time = struct.unpack("<L", packet[80:84])[0]

                #Parse Packet Number
                pktnum = struct.unpack("<H", packet[84:86])[0]
                
                #Parse RSSI
                rxrssi = struct.unpack("<b", packet[86:87])[0] - 99

                #Parse BMS
                cell_voltages = [0,0,0]
                for i in range(0, 3):
                    cell_voltages[i] = struct.unpack("<h", packet[87 + 2 * i: 89 + 2 * i])[0] / 1000.0
                total_current = struct.unpack("<h", packet[93:95])[0] / -1000.0
                bms_temp = packet[95] / 2
                bms_protection_status = packet[96]
                bms_protections_enabled = packet[97]

                converter_currents = [0,0,0,0,0,0]
                converter_voltages = [0,0,0,0,0,0]
                for i in range(0, 6):
                    converter_voltages[i] = struct.unpack("<h", packet[98 + 2 * i: 100 + 2 * i])[0] * 0.0016
                    converter_currents[i] = struct.unpack("<h", packet[110 + 2 * i: 112 + 2 * i])[0] * 0.000625
                
                #Parse Integrated Acceleration
                accel_integrated_velo = struct.unpack("<f", packet[122:126])[0]

                angleFromVertical = np.arccos(np.cos(pitch_gyro_int * np.pi / 180.0) * np.cos(yaw_gyro_int * np.pi / 180)) * 180.0 / np.pi


                if self.logging:
                    data = [
                        time.time(),
                        pyros,
                        servos,
                        servos_deg,
                        accelerometer,
                        barofilteredalt,
                        temp,
                        gyro,
                        gps_fix,
                        lat,
                        lon,
                        gpsalt,
                        gps_horiz_prec,
                        gps_vert_prec,
                        gps_num_sat,
                        flight_time,
                        yaw_gyro_int,
                        pitch_gyro_int,
                        roll_gyro_int,
                        str(state0),
                        pktnum,
                        rssi,
                        armed,
                        fired,
                        badpackets,
                        rxrssi,
                        accel_integrated_velo,
                        baro_max_alt,
                        gps_max_alt,
                        pyro_resistances,
                        cell_voltages,
                        total_current,
                        converter_voltages,
                        converter_currents,
                        bms_protections_enabled,
                        bms_protection_status,
                        bms_temp,
                        enabled_status,
                        angleFromVertical,
                        gnd_lat,
                        gnd_lon,
                        gnd_fix,
                        gnd_alt
                    ]
                    self.csv_writer_badpackets.writerow(data)
                    self.file_badpackets.flush()
                    os.fsync(self.file_badpackets.fileno())
                    if self.debug:
                        print("[RKT] WARNING Bad Packet Received, storing.")

            return True

    
    def set_vtx_power(self,power_level):
        a = power_level,["1 W", "3 W","5 W", "8 W"][power_level]   
        print("[RKT] Received req for new power level {} <=> {}".format(*a))
        data = bytearray(16)
        data[0] = 0xAA
        data[12] = power_level
        data[13] = 0x14
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)
        

    def zero_pitchYawRoll(self):
        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x09
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def zero_roll(self):
        # obsolete.
        print("[RKT] Warning. Obsolete method, resets pitch and yaw as well.")
        self.zero_pitchYawRoll()

    def zero_alt(self):
        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x0A
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def zero_velo(self):
        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x0B
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def advance_state(self):
        data = bytearray(16)
        data[0] = 0xAA 
        data[12] = self.state.value + 1
        data[13] = 0x02
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def zero_servos(self):
        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x08
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def pd_activate(self):
        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x04
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def arm_pyros(self, channels):
        data = bytearray(16)
        data[0] = 0xAA
        for i in channels:    
            data[12] |= 0x01 << i
        data[13] = 0x01
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def fire_pyros(self, channels):
        data = bytearray(16)
        data[0] = 0xAA
        for i in channels:    
            data[12] |= 0x01 << i
        data[13] = 0x06
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def disarm_pyros(self, channels):
        data = bytearray(16)
        data[0] = 0xAA
        for i in channels:    
            data[12] |= 0x01 << i
        data[13] = 0x07
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def set_roll_control_servo_angle(self, angle):
        data = bytearray(16)
        data[0] = 0xAA
        struct.pack_into("<f", data, 9, angle)
        data[13] = 0x05
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def set_airbrakes_angle(self, angle):
        data = bytearray(16)
        data[0] = 0xAA
        struct.pack_into("<f", data, 9, angle)
        data[13] = 0x03
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def calc_checksum(self, data):
        chksum = 0
        for i in range(1, 14):
            chksum += data[i]
            if chksum > (256 * 256 - 1):
                chksum %= (256 * 256)
        return chksum

    def connect_serial(self, port=None):
        """
        Try to open the serial port. Returns (True, None) on success,
        (False, errmsg) on failure.
        """
        print("[RKT] Trying to connect to port {}".format(port))
        if port is not None:
            self.serial_port = port
        if self.serial_port is None:
            print("[RKT] No port specified.")
            return False, "No serial port specified"

        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
            print("[RKT] Connection Successful")
            return True, None
        except Exception as e:
            self.ser = None
            print("[RKT] Connection Failed")
            return False, str(e)

    def disconnect_serial(self):
        """Close serial port if open."""
        print("[RKT] Trying to disconnect from port {}".format(self.serial_port))
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
            self.ser = None
            print("[RKT] Disconnection Successful")
        except Exception:
            self.ser = None
            print("[RKT] Problem; but disconnection Successful")



    def EMERG_DEPLOY_PISTON(self):
        print("[RKT] [EMERGENCY] Req deployment of piston...")

        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x10
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

        print("[RKT] [EMERGENCY] Piston Deployment Reqd.")

    def EMERG_DEPLOY_BP_WELLS(self):
        print("[RKT] [EMERGENCY] Req deployment of Black Powder Wells...")

        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x11
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

        print("[RKT] [EMERGENCY] BP Wells Deployment Reqd.")

    def EMERG_DEPLOY_TD(self):
        print("[RKT] [EMERGENCY] Req deployment of Tender Descender...")

        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x12
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

        print("[RKT] [EMERGENCY] TD Deployment Reqd.")

    def EMERG_DEPLOY_ALL(self):
        print("[RKT] [EMERGENCY] Req deployment of ALL RECOVERY MEASURES")

        data = bytearray(16)
        data[0] = 0xAA
        data[13] = 0x13
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

        print("[RKT] [EMERGENCY] All recovery measures deployment requested.")

    def update_converters(self, converters):
        # converters is of structure
        # [3v_enabled, 3p3v_enabled, 5v_enabled, 7v4_enabled, 8v4_enabled, 28v_enabled]
        data = bytearray(16)
        data[0] = 0xAA
        for i in range(len(converters)):    
            if converters[i]:
                data[12] |= 0x01 << i
        data[13] = 0x15
        print("[RKT] [Debug] data[12]: {}".format(data[12]))
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)

    def bmsprotections(self, enabled):
        data = bytearray(16)
        data[0] = 0xAA
        data[12] = enabled
        data[13] = 0x16
        struct.pack_into(">H", data, 14, self.calc_checksum(data))
        self.ser.write(data)


    def microsecToDeg_airbrakes(self, microsec):
        return (microsec - 1500)/500 * 60

    def microsecToDeg_rollCtrl(self, microsec):
        return (microsec - 1500)/500 * 50







""" old
                data = [
                    time.time(), self.pyros, self.servos, self.accelerometer, self.barometer,
                    self.barofilteredalt, self.barofilteredvelo, self.temp, self.gyro,
                    self.magnetometer, self.heading, self.gps_fix, self.lat, self.lon, self.gpsalt,
                    self.pdop, self.hdop, self.vdop, self.flight_time, self.last_rec,
                    self.yaw_gyro_int, self.pitch_gyro_int, self.roll_gyro_int,
                    self.batt_voltage, str(self.state), self.pktnum, self.rssi,
                    self.armed, self.fired, self.badpackets, self.rxrssi,
                    self.accel_integrated_velo, self.baro_max_alt, self.gps_max_alt
                ]
"""






