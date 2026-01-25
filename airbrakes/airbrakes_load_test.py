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