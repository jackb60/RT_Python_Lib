import math
import serial
import struct
import platform
import time
import numpy as np



gps_lat = 42.360370
gps_lon = -71.093554

isWin = platform.system() == "Windows"

MAC_HARDWARE_PORT = "/dev/cu.Bluetooth-Incoming-Port"
WIN_HARDWARE_PORT = "COM11"

HARDWARE_PORT = WIN_HARDWARE_PORT if isWin else MAC_HARDWARE_PORT

class pointer:
    def __init__(self, port=HARDWARE_PORT, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.isConnected = False
        self.gps_lat = gps_lat
        self.gps_lon = gps_lon
        self.gps_alt = 0 
        self.gps_update_time = 0 
        self.debug = True # global
        self.has_received_atLeastOne_pointer_gps_update = False
        self.last_elev = 0
        self.last_azim = 0

        # WGS84 constants
        self.a = 6378137.0          # semi-major axis (m)
        self.f = 1 / 298.257223563  # flattening
        self.e2 = self.f * (2 - self.f)       # eccentricity squared

    def connect(self,port=None,baud=115200):
        if self.debug:
            print("[PTR] Starting connection to {} at rate {}".format(port,baud))
        if self.isConnected:
            print("[PTR] Already connected to {} at rate {}".format(self.port,self.baud))
            return True, None

        if port is not None:
            self.port = port
        try:
            self.baud = baud
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.isConnected = True
            print("[PTR] Successful connection")
            return True, None

        except Exception as e:
            print("[PTR] Failed to connect to {} at rate {}".format(port,baud))
            if self.debug:
                print(e)
            self.isConnected = False
            return False, e



    def disconnect(self):
        if self.debug:
            print("[PTR] Starting disconnection from {} at rate {}".format(self.port,self.baud))
        if not self.isConnected:
            print("[PTR] Already disconnected.")
            if self.debug:
                print("[PTR] Last port was {} at rate {}".format(self.port,self.baud))
            return True, None
        else:
            e = None
            try:
                self.ser.close()
            except Exception as e:
                print("[PTR] Failed to close serial. Noncritical.")
            self.ser = None
            self.isConnected = False
            print("[PTR] Successful disconnection to {} at rate {}".format(self.port,self.baud))
            return True, e

    # Rocket Control Functions
    def updateGPS(self,lat,lon,alt):
        self.has_received_atLeastOne_pointer_gps_update = True
        self.gps_lat = lat
        self.gps_lon = lon
        self.gps_alt = 0 #FOR TEST LAUNCH
        self.gps_update_time = time.time()
        print("[PTR] Successfully updated GPS: ({} N,{} E,{} m)".format(lat,lon,alt))

    def refreshPointerState(self,rocketLat,rocketLon,rocketAlt):
        # Procedure:
        # Check has a gps <=> has_received_atLeastOne_pointer_gps_update
        # Calculate Angles
        # Send Angles
        if self.has_received_atLeastOne_pointer_gps_update:
            self.last_azim, self.last_elev = self.calc_angles(rocketLat,rocketLon,rocketAlt)
            self.send_angles(self.last_azim, self.last_elev)
            print("[PTR] [Controller] Sent Angles AZIM {} ELEV {}".format(self.last_azim, self.last_elev))



    # Workers
    
    def send_angles(self, azimuth, elevation):
        if self.isConnected:
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
            print("[PTR] [Comm] Sent Angle {} az {} elev".format(azimuth,elevation))
            return True
        else:
            print("[PTR] [Comm] Command failed: disconnected.")
            return False


    def calc_checksum(self, data):
        chksum = 0
        for i in range(1, 10): 
            chksum += data[i]
            if chksum > (256 - 1):
                chksum %= 256
        return chksum
    
    def up(self):
        if self.isConnected:
            data = bytearray(11)
            data[0] = 0xAA
            data[1] = 0x01
            data[10] = self.calc_checksum(data)
            self.ser.write(data)
            print("[PTR] Sent UP")
            return True
        else:
            print("[PTR] [Comm] Command failed: disconnected.")
            return False


    def down(self):
        if self.isConnected:
            data = bytearray(11)
            data[0] = 0xAA
            data[1] = 0x02
            data[10] = self.calc_checksum(data)
            self.ser.write(data)
            print("[PTR] Sent DOWN")
            return True
        else:
            print("[PTR] [Comm] Command failed: disconnected.")
            return False

    def left(self):
        if self.isConnected:
            data = bytearray(11)
            data[0] = 0xAA
            data[1] = 0x03
            data[10] = self.calc_checksum(data)
            self.ser.write(data)
            print("[PTR] Sent LEFT")
            return True
        else:
            print("[PTR] [Comm] Command failed: disconnected.")
            return False

    def right(self):
        if self.isConnected:
            data = bytearray(11)
            data[0] = 0xAA
            data[1] = 0x04
            data[10] = self.calc_checksum(data)
            self.ser.write(data)
            print("[PTR] Sent RIGHT")
            return True
        else:
            print("[PTR] [Comm] Command failed: disconnected.")
            return False

    def zero(self):
        if self.isConnected:
            data = bytearray(11)
            data[0] = 0xAA
            data[1] = 0x05
            data[10] = self.calc_checksum(data)
            self.ser.write(data)
            print("[PTR] Sent ZERO")
            return True
        else:
            print("[PTR] [Comm] Command failed: disconnected.")
            return False






    ### CALCULATION OF VECTOR



    def geodetic_to_ecef(self, lat, lon, alt):
        lat = np.radians(lat)
        lon = np.radians(lon)
        a = self.a
        f = self.f
        e2 = self.e2

        N = a / np.sqrt(1 - e2 * np.sin(lat)**2)

        x = (N + alt) * np.cos(lat) * np.cos(lon)
        y = (N + alt) * np.cos(lat) * np.sin(lon)
        z = (N * (1 - e2) + alt) * np.sin(lat)

        return np.array([x, y, z])


    def ecef_to_enu(self,ecef_vec, lat_ref, lon_ref):
        lat = np.radians(lat_ref)
        lon = np.radians(lon_ref)

        # Transformation matrix
        R = np.array([
            [-np.sin(lon),              np.cos(lon),             0],
            [-np.sin(lat)*np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)],
            [ np.cos(lat)*np.cos(lon),  np.cos(lat)*np.sin(lon), np.sin(lat)]
        ])

        return R @ ecef_vec


    def az_el_from_geodetic(self,lat1, lon1, alt1, lat2, lon2, alt2):
        print("[PTR] [Calc] [Internals] Call to `az_el_from_geodetic`")
        # Convert both points to ECEF
        p1 = self.geodetic_to_ecef(lat1, lon1, alt1)
        p2 = self.geodetic_to_ecef(lat2, lon2, alt2)
        print("[PTR] [Calc] [Internals] Converted from Geodetic to ECEF System")

        # Vector from antenna to rocket in ECEF
        vec_ecef = p2 - p1

        # Convert to ENU frame at antenna
        vec_enu = self.ecef_to_enu(vec_ecef, lat1, lon1)
        east, north, up = vec_enu

        # Azimuth (0° = North, increases clockwise)
        azimuth = np.degrees(np.arctan2(east, north))
        if azimuth < 0:
            azimuth += 360

        # Elevation angle
        horizontal_dist = np.sqrt(east**2 + north**2)
        print("[PTR] [Calc] Got Horizontal Dist {}".format(horizontal_dist))
        elevation = np.degrees(np.arctan2(up, horizontal_dist))
        return azimuth, elevation



    def calc_angles(self, rocket_lat, rocket_long, rocket_alt):

        lat1 = self.gps_lat
        lon1 = self.gps_lon
        alt1 = self.gps_alt
        print("[PTR] [Calc] Using stored antenna pointer GPS coordinates, last updated {} seconds ago".format(np.round(time.time()-self.gps_update_time,1)))
        print("[PTR] [Calc] Pointer location: ({} N, {} E, {} m)".format(lat1,lon1,alt1))

        lat2 = rocket_lat
        lon2 = rocket_long
        alt2 = rocket_alt

        print("[PTR] [Calc] Rocket location:  ({} N, {} E, {} m)".format(lat2,lon2,alt2))

        azimuth_target, elevation_target = self.az_el_from_geodetic(lat1, lon1, alt1, lat2, lon2, alt2)
        #except Exception as e:
        #    raise(e)
        #    elevation_target = 0
        #    azimuth_target = 0
        print(f"[PTR] [Calc] Calculated Azimuth: {azimuth_target}, Elevation: {elevation_target}")
        return azimuth_target, elevation_target





























