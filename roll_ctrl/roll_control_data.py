import csv
import ast

input_csv = "Roll_Control_Test_1_14.csv"
output_csv = "output.csv"

offset6 = -19
offset7 = -7
kP = 0.26
kD = 0.0520

def us_to_deg(us):
    return (us - 1500.0) * (60.0 / 500.0)

def deg_to_us(deg):
    return 1500.0 + (deg / 60.0) * 500.0

with open(input_csv, newline="") as infile, open(output_csv, "w", newline="") as outfile:
    reader = csv.reader(infile)
    writer = csv.writer(outfile)

    writer.writerow([
        "accel_integrated_velo",
        "gyro_y_dps",
        "roll_gyro_int_deg",
        "servo_6_us",
        "servo_7_us",
        "servo_6_cmd_deg",
        "servo_7_cmd_deg",
        "control_output_deg"
    ])

    for row in reader:
        servos = ast.literal_eval(row[2])
        gyro = ast.literal_eval(row[8])

        gyro_y = gyro[1]                     # deg/s
        roll_gyro_int = float(row[22])       # deg

        accel_integrated_velo = float(row[31])

        clipped_velo = accel_integrated_velo
        if clipped_velo > 300.0:
            clipped_velo = 300.0
        elif clipped_velo < 20:
            clipped_velo = 20.0

        servo_6_us = servos[6]
        servo_7_us = servos[7]

        servo_6_cmd_deg = us_to_deg(servo_6_us) - offset6
        servo_7_cmd_deg = us_to_deg(servo_7_us) - offset7

        control_output = kP * 10000 / (clipped_velo**2) * -roll_gyro_int + kD * 10000 / (clipped_velo**2)  * -gyro_y

        writer.writerow([
            accel_integrated_velo,
            gyro_y,
            roll_gyro_int,
            servo_6_us,
            servo_7_us,
            round(servo_6_cmd_deg, 2),
            round(servo_7_cmd_deg, 2),
            round(control_output, 2)
        ])

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons

df = pd.read_csv(output_csv)

signals = {
    "Accel Integrated Velo (m/s)": df["accel_integrated_velo"],
    "Gyro Y (deg/s)": df["gyro_y_dps"],
    "Roll Angle (deg)": df["roll_gyro_int_deg"],
    "Servo 6 Cmd (deg)": df["servo_6_cmd_deg"],
    "Control Output (deg)": df["control_output_deg"],
}

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.35)

lines = {}
for label, data in signals.items():
    line, = ax.plot(data, label=label)
    lines[label] = line

ax.set_xlabel("Sample Index")
ax.set_ylabel("Degrees / Deg per sec")
ax.set_title("Roll Control Signals")
ax.grid(True)

# ✅ Color legend (this is what you want)
ax.legend(loc="upper right")

# Checkbox panel
rax = plt.axes([0.02, 0.35, 0.30, 0.25])
labels = list(lines.keys())
visibility = [True] * len(labels)
check = CheckButtons(rax, labels, visibility)

def toggle(label):
    lines[label].set_visible(not lines[label].get_visible())
    plt.draw()

check.on_clicked(toggle)

plt.show()