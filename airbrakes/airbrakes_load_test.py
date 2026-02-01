import airbrakes_dummy
import time
import pandas as pd

#df = pd.read_csv("c:/Users/jackb/RT_Python_Lib/airbrakes/dat/6225/airbrakes_input.csv")
df = pd.read_csv("dat/6275/airbrakes_input.csv")

times = df["Time"].tolist()
velocities = df["Velocity"].tolist()
accelerations = df["Acceleration"].tolist()
apogees = df["Apogee"].tolist()
altitudes = df["Altitude"].tolist()

"""

header = [
                    "time", "pyrostat", "servostat", "accel", "barometer",
                    "baro_alt_filtered", "baro_vel_filtered", "temp", "gyro",
                    "magnetometer", "heading", "gps_fix_status", "lat", "lon", "gpsalt",
                    "pdop", "hdop", "vdop", "flight_time", "last_rec",
                    "yaw_gyro_int", "pitch_gyro_int", "roll_gyro_int",
                    "batt_voltage", "rocket_state", "pktnum", "rssi",
                    "armed_pyros", "fired_pyros", "badpackets", "rxrssi",
                    "accel_integrated_velo", "baro_max_alt", "gps_max_alt"
                ]

See rocket.py in /ground_station/

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



airbrakes = airbrakes_dummy.aribrakes_dummy()

airbrakes.start_sim()
begin = time.time()

t_index = 0

while time.time() - begin < times[-1] + 1:
    if not t_index >= len(times):
        if time.time() - begin >= times[t_index] or t_index == len(times) - 1:
            airbrakes.send_velo(velocities[t_index])
            airbrakes.send_accel(accelerations[t_index])
            airbrakes.send_apogee(apogees[t_index])
            airbrakes.send_altitude(altitudes[t_index])
            t_index += 1
    airbrakes.read_response()

df = pd.DataFrame({
    "Time": airbrakes.deploy_times,
    "Actual DP": airbrakes.deploy_fracs
})

df.to_csv("airbrakes_output.csv", index=False)