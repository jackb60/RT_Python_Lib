import Serial
import bleak
import struct

class rocket:
    def __init__(self):
        self.pyros = [0] * 8
        self.servos = [0] * 8
        self.accelerometer = [0, 0, 0] #x, y, z
        self.barometer = 0
        self.pid = [0, 0, 0] #p, i, d

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
                servo_info += packet[3 + i] << (8 * i)

            for i in range(0, 8):
                self.servos[i] = (servo_info << (12 * i)) | 0xFFF

            #Parse acceleration
            self.accelerometer[0] = struct.unpack("<h", )[0]
            self.accelerometer[1] = struct.unpack("<h", )[0]
            self.accelerometer[2] = struct.unpack("<h", )[0]

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

